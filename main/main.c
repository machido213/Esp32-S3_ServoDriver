/*
 * ESP32-S3 ServoDriver 主程式
 * 
 * 功能總覽：
 * 1. 核心控制：使用 FreeRTOS 多工處理，同時控制 7 軸馬達與 Web 伺服器。
 * 2. 網路功能：優先連線 WiFi，失敗則自動開啟 AP 熱點 (救援模式)。
 * 3. 記憶功能 (NVS)：斷電後仍記得 WiFi 帳密與固定 IP 設定。
 * 4. 介面 (SPIFFS)：網頁檔案存放在 Flash 中，不佔用程式碼空間。
 * 5. OTA 更新：支援從 GitHub 下載新韌體並自動更新。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h> // # 提供 MIN() 函式，用來比較數字大小
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h" // # 用於獲取高精度時間 (微秒級)，控制馬達速度用

// # 引入你寫好的腳位定義檔，這樣改腳位只要改那個檔案就好
#include "io_config.h"

static const char *TAG = "SERVO_DRIVER"; // Log 標籤，除錯時會顯示這個名字

// --- 風扇 PWM 設定 (脈波寬度調變) ---
// 用來控制風扇轉速，0=停，255=全速
#define FAN_LEDC_TIMER     LEDC_TIMER_0
#define FAN_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL   LEDC_CHANNEL_0
#define FAN_LEDC_RES       LEDC_TIMER_8_BIT // 解析度 8-bit (數值範圍 0~255)
#define FAN_LEDC_FREQ      5000             // 頻率 5kHz

// --- 全局狀態變數 ---
// 用來記錄目前的狀態，並回傳給網頁顯示
static int g_fan_duty = 0;      // 風扇目前的轉速
static bool g_servo_on = false; // 伺服始能狀態
static bool g_power_on = false; // 系統電源狀態

// --- 馬達控制結構體 ---
// # 這是每個馬達的「身分證」，記錄它現在該做什麼
typedef struct {
    bool run;               // 是否正在跑？ (true=跑, false=停)
    int steps_left;         // 還剩下幾步要跑？
    int target_steps;       // 原本設定要跑幾步？ (用來顯示在網頁上)
    int freq;               // 速度 (頻率 Hz)
    int dir;                // 方向 (1=正轉, 0=反轉)
    int64_t last_step_time; // 上次發出脈波的時間 (用來計算下一次何時發)
} MotorState;

// 建立 7 個馬達的狀態陣列
MotorState motors[MOTOR_AXIS_COUNT] = {0}; 

// --- WiFi 設定預設值 ---
// 如果 NVS 裡面沒資料，就會用這些預設值
#define DEFAULT_SSID      "SSID"
#define DEFAULT_PASS      "********"
#define DEFAULT_IP        "192.168.2.124"
#define DEFAULT_GW        "192.168.2.1"
#define DEFAULT_MASK      "255.255.255.0"

// AP 救援模式設定 (連不到 WiFi 時變成熱點)
#define AP_SSID           "ESP32-Servo-Rescue"
#define AP_PASS           "" // 空密碼
#define MAX_RETRY         5  // 嘗試連線 5 次失敗就放棄，改開 AP

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0 // 連線成功旗標
#define WIFI_FAIL_BIT      BIT1 // 連線失敗旗標
static int s_retry_num = 0;

// 系統設定結構 (用來讀寫 NVS)
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char static_ip[16];
    char static_gw[16];
    char static_mask[16];
} SystemConfig;
SystemConfig sys_cfg;

/* ================= NVS 讀寫功能 (記憶設定) ================= */
void load_settings() {
    nvs_handle_t my_handle;
    // 打開名為 "storage" 的儲存區
    if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) {
        // 如果打開失敗 (可能是第一次開機)，載入預設值
        strcpy(sys_cfg.wifi_ssid, DEFAULT_SSID); strcpy(sys_cfg.wifi_pass, DEFAULT_PASS);
        strcpy(sys_cfg.static_ip, DEFAULT_IP); strcpy(sys_cfg.static_gw, DEFAULT_GW); strcpy(sys_cfg.static_mask, DEFAULT_MASK);
        return;
    }
    // 嘗試讀取設定，讀不到就用預設值
    size_t size = sizeof(sys_cfg.wifi_ssid);
    if (nvs_get_str(my_handle, "ssid", sys_cfg.wifi_ssid, &size) != ESP_OK) strcpy(sys_cfg.wifi_ssid, DEFAULT_SSID);
    size = sizeof(sys_cfg.wifi_pass);
    if (nvs_get_str(my_handle, "pass", sys_cfg.wifi_pass, &size) != ESP_OK) strcpy(sys_cfg.wifi_pass, DEFAULT_PASS);
    size = sizeof(sys_cfg.static_ip);
    if (nvs_get_str(my_handle, "ip", sys_cfg.static_ip, &size) != ESP_OK) strcpy(sys_cfg.static_ip, DEFAULT_IP);
    size = sizeof(sys_cfg.static_gw);
    if (nvs_get_str(my_handle, "gw", sys_cfg.static_gw, &size) != ESP_OK) strcpy(sys_cfg.static_gw, DEFAULT_GW);
    size = sizeof(sys_cfg.static_mask);
    if (nvs_get_str(my_handle, "mask", sys_cfg.static_mask, &size) != ESP_OK) strcpy(sys_cfg.static_mask, DEFAULT_MASK);
    
    nvs_close(my_handle);
}

void save_settings(const char* ssid, const char* pass, const char* ip, const char* gw, const char* mask) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "ssid", ssid); nvs_set_str(my_handle, "pass", pass);
        nvs_set_str(my_handle, "ip", ip); nvs_set_str(my_handle, "gw", gw); nvs_set_str(my_handle, "mask", mask);
        nvs_commit(my_handle); // # 這一行最重要，這才是真正寫入晶片
        nvs_close(my_handle);
    }
}

/* ================= 硬體初始化 ================= */
void init_hardware(void) {
    // 1. 設定系統輸出 (繼電器 & Enable)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT; // 設定為輸出模式
    io_conf.pin_bit_mask = (1ULL << SYS_START_TRIGGER_GPIO) | (1ULL << SYS_POWER_CUTOFF_GPIO) | (1ULL << SERVO_ENABLE_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // 初始狀態設為 Low (關閉)
    gpio_set_level(SYS_START_TRIGGER_GPIO, 0);
    gpio_set_level(SYS_POWER_CUTOFF_GPIO, 0);
    gpio_set_level(SERVO_ENABLE_GPIO, 0);

    // 2. 設定系統輸入 (按鈕 & 回饋)
    io_conf.mode = GPIO_MODE_INPUT; // 設定為輸入模式
    io_conf.pin_bit_mask = (1ULL << BTN_STOP_GPIO) | (1ULL << SYS_STATE_FEEDBACK_GPIO);
    io_conf.pull_up_en = 1; // # 啟用上拉電阻，因為按鈕通常是按下接地 (Low)
    gpio_config(&io_conf);

    // 3. 設定馬達輸出 (7 軸的 Pulse 和 Direction)
    uint64_t motor_mask = 0;
    for(int i=0; i<MOTOR_AXIS_COUNT; i++) {
        motor_mask |= (1ULL << MOTOR_PINS[i].pul_pin);
        motor_mask |= (1ULL << MOTOR_PINS[i].dir_pin);
        gpio_set_level(MOTOR_PINS[i].pul_pin, 0);
        gpio_set_level(MOTOR_PINS[i].dir_pin, 0);
    }
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = motor_mask;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // 4. 設定風扇 PWM (LEDC)
    ledc_timer_config_t ledc_timer = { 
        .speed_mode = FAN_LEDC_MODE, .timer_num = FAN_LEDC_TIMER, 
        .duty_resolution = FAN_LEDC_RES, .freq_hz = FAN_LEDC_FREQ, .clk_cfg = LEDC_AUTO_CLK 
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = { 
        .speed_mode = FAN_LEDC_MODE, .channel = FAN_LEDC_CHANNEL, .timer_sel = FAN_LEDC_TIMER, 
        .intr_type = LEDC_INTR_DISABLE, .gpio_num = FAN_PWM_GPIO, .duty = 0, .hpoint = 0 
    };
    ledc_channel_config(&ledc_channel);
}

/* ================= WiFi 連線邏輯 ================= */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // WiFi 啟動就嘗試連線
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // 斷線重連邏輯
        if (s_retry_num < MAX_RETRY) { 
            esp_wifi_connect(); 
            s_retry_num++; 
        } else { 
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // 放棄連線，準備切換 AP
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0; 
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // 連線成功
    }
}

bool wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
    
    // 設定固定 IP (Static IP)
    esp_netif_dhcpc_stop(my_sta);
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = esp_ip4addr_aton(sys_cfg.static_ip);
    ip_info.gw.addr = esp_ip4addr_aton(sys_cfg.static_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(sys_cfg.static_mask);
    esp_netif_set_ip_info(my_sta, &ip_info);
    
    // 設定 DNS (Google DNS)，為了讓 OTA 能解析 GitHub 網址
    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = ipaddr_addr("8.8.8.8");
    dns_info.ip.type = IPADDR_TYPE_V4;
    esp_netif_set_dns_info(my_sta, ESP_NETIF_DNS_MAIN, &dns_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, sys_cfg.wifi_ssid, 32);
    strncpy((char*)wifi_config.sta.password, sys_cfg.wifi_pass, 64);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    // 等待連線結果
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    return (bits & WIFI_CONNECTED_BIT);
}

void wifi_init_ap(void) {
    esp_wifi_stop(); esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_netif_create_default_wifi_ap();
    wifi_config_t wifi_config = { 
        .ap = { 
            .ssid = AP_SSID, .ssid_len = strlen(AP_SSID), 
            .channel = 1, .password = AP_PASS, 
            .max_connection = 4, .authmode = WIFI_AUTH_OPEN 
        } 
    };
    esp_wifi_set_mode(WIFI_MODE_AP); 
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config); 
    esp_wifi_start();
}

/* ================= 核心：馬達控制任務 (真正輸出脈波) ================= */
// # 這是一個獨立的執行緒，專門負責產生脈波，不會被網頁卡住
void motor_task(void *arg) {
    while (1) {
        int64_t now = esp_timer_get_time(); // 獲取當前微秒時間
        bool any_running = false;

        // 檢查所有軸
        for (int i = 0; i < MOTOR_AXIS_COUNT; i++) {
            if (motors[i].run) {
                any_running = true;
                // 計算兩次脈波的時間間隔 (微秒) = 1秒 / 頻率
                int interval_us = 1000000 / motors[i].freq;

                // 時間到了嗎？
                if (now - motors[i].last_step_time >= interval_us) {
                    motors[i].last_step_time = now;

                    // 1. 設定方向
                    gpio_set_level(MOTOR_PINS[i].dir_pin, motors[i].dir);

                    // 2. 產生脈波 (High -> Delay -> Low)
                    gpio_set_level(MOTOR_PINS[i].pul_pin, 1);
                    esp_rom_delay_us(5); // # 脈寬 5us，確保驅動器讀得到
                    gpio_set_level(MOTOR_PINS[i].pul_pin, 0);
                    
                    // 3. 扣除步數
                    if (motors[i].steps_left > 0) {
                        motors[i].steps_left--;
                        if (motors[i].steps_left == 0) {
                            motors[i].run = false; // 步數跑完，自動停止
                        }
                    }
                }
            }
        }
        
        // # 如果有馬達在跑，只休息 1 tick (極短)，保證速度；如果沒馬達跑，休息久一點省電
        if (!any_running) vTaskDelay(pdMS_TO_TICKS(10));
        else vTaskDelay(1); 
    }
}

/* ================= 網頁伺服器處理函式 (API) ================= */

// GET / : 讀取 index.html
static esp_err_t root_get_handler(httpd_req_t *req) {
    FILE* f = fopen("/spiffs/index.html", "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
    char line[256];
    // 逐行讀取檔案並傳送給瀏覽器
    while (fgets(line, sizeof(line), f)) httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    fclose(f); 
    httpd_resp_send_chunk(req, NULL, 0); // 傳送結束信號
    return ESP_OK;
}

// POST /api/relay : 控制繼電器
static esp_err_t api_relay_handler(httpd_req_t *req) {
    char buf[100], type[20]={0}, state_str[5]={0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "type", type, sizeof(type));
        httpd_query_key_value(buf, "state", state_str, sizeof(state_str));
        int state = atoi(state_str);
        
        if (strcmp(type, "start")==0 && state==1) {
            // 點動啟動：按下 -> 延遲 -> 放開
            gpio_set_level(SYS_START_TRIGGER_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(SYS_START_TRIGGER_GPIO, 0);
            g_power_on = true;
            ESP_LOGW(TAG, "🔥 [WEB CMD] SYSTEM START Triggered!");
        } else if (strcmp(type, "cutoff")==0 && state==1) {
            // 斷電：按下 -> 延遲 -> 放開
            gpio_set_level(SYS_POWER_CUTOFF_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(SYS_POWER_CUTOFF_GPIO, 0);
            g_power_on = false; g_servo_on = false;
            ESP_LOGW(TAG, "⛔ [WEB CMD] SYSTEM CUTOFF Triggered!");
        } else if (strcmp(type, "enable")==0) {
            // 始能：保持狀態
            gpio_set_level(SERVO_ENABLE_GPIO, state);
            g_servo_on = (state == 1);
            ESP_LOGW(TAG, "⚡ [WEB CMD] SERVO ENABLE set to: %s", state ? "ON" : "OFF");
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// POST /api/fan : 控制風扇
static esp_err_t api_fan_handler(httpd_req_t *req) {
    char buf[50], val_str[10]={0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "val", val_str, sizeof(val_str));
        int duty = atoi(val_str);
        ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
        ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
        g_fan_duty = duty;
        ESP_LOGI(TAG, "🌀 [WEB CMD] Fan Speed set to: %d", duty);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// GET /status : 回傳狀態 JSON
static esp_err_t api_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    // 讀取真實 IO 狀態 (0=Low, 1=High)
    int stop_btn_val = gpio_get_level(BTN_STOP_GPIO); 
    int feedback_val = gpio_get_level(SYS_STATE_FEEDBACK_GPIO);

    cJSON_AddNumberToObject(root, "temp", 42.5); // 溫度暫時寫死
    cJSON_AddNumberToObject(root, "btn_stop", stop_btn_val);
    cJSON_AddNumberToObject(root, "feedback", feedback_val);
    cJSON_AddNumberToObject(root, "fan", g_fan_duty);
    cJSON_AddBoolToObject(root, "servo_on", g_servo_on);
    cJSON_AddBoolToObject(root, "power_on", g_power_on);
    
    // 網路資訊
    cJSON *net = cJSON_CreateObject();
    cJSON_AddStringToObject(net, "ssid", sys_cfg.wifi_ssid);
    cJSON_AddStringToObject(net, "ip", sys_cfg.static_ip);
    cJSON_AddStringToObject(net, "gw", sys_cfg.static_gw);
    cJSON_AddItemToObject(root, "net", net);

    // 馬達狀態陣列
    cJSON *arr = cJSON_CreateArray();
    for(int i=0; i<MOTOR_AXIS_COUNT; i++) {
        cJSON *m = cJSON_CreateObject();
        cJSON_AddBoolToObject(m, "run", motors[i].run);
        cJSON_AddNumberToObject(m, "steps", motors[i].target_steps);
        cJSON_AddNumberToObject(m, "freq", motors[i].freq);
        cJSON_AddNumberToObject(m, "dir", motors[i].dir);
        cJSON_AddItemToArray(arr, m);
    }
    cJSON_AddItemToObject(root, "motors", arr);

    const char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    free((void*)resp); cJSON_Delete(root);
    return ESP_OK;
}

// OTA 更新任務 (背景下載)
static void ota_task(void *arg) {
    char *url = (char *)arg;
    ESP_LOGI(TAG, "Starting OTA: %s", url);
    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
        .buffer_size = 16384, // # 加大 Buffer，防止 GitHub 下載失敗
        .buffer_size_tx = 4096,
        .timeout_ms = 30000,
    };
    esp_https_ota_config_t ota_config = { .http_config = &http_cfg };
    if (esp_https_ota(&ota_config) == ESP_OK) { 
        ESP_LOGW(TAG, "✅ OTA Update Successful! Rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        esp_restart(); 
    } else {
        ESP_LOGE(TAG, "❌ OTA Failed!");
    }
    free(url); vTaskDelete(NULL);
}

static esp_err_t ota_post_handler(httpd_req_t *req) {
    char buf[512]; int ret = httpd_req_recv(req, buf, MIN(req->content_len, 511));
    if(ret<=0) return ESP_FAIL; 
    buf[ret]=0; // 字串結尾
    
    cJSON *root = cJSON_Parse(buf);
    if(root){
        cJSON *url = cJSON_GetObjectItem(root, "url");
        if(cJSON_IsString(url)) { 
            char *p = strdup(url->valuestring); 
            xTaskCreate(ota_task, "ota_task", 8192, p, 5, NULL); // 啟動 OTA 任務
            httpd_resp_sendstr(req, "Started"); 
        }
        cJSON_Delete(root);
    } else httpd_resp_send_500(req);
    return ESP_OK;
}

// 儲存 WiFi 設定
static esp_err_t api_save_wifi_handler(httpd_req_t *req) {
    char buf[512]; int ret = httpd_req_recv(req, buf, MIN(req->content_len, 511));
    if(ret<=0) return ESP_FAIL; 
    buf[ret]=0;
    
    cJSON *root = cJSON_Parse(buf);
    if(root) {
        cJSON *s=cJSON_GetObjectItem(root,"ssid"), *p=cJSON_GetObjectItem(root,"pass");
        cJSON *i=cJSON_GetObjectItem(root,"ip"), *g=cJSON_GetObjectItem(root,"gw");
        if(cJSON_IsString(s) && cJSON_IsString(p)) {
            // 寫入 NVS
            save_settings(s->valuestring, p->valuestring, i?i->valuestring:DEFAULT_IP, g?g->valuestring:DEFAULT_GW, "255.255.255.0");
            httpd_resp_sendstr(req, "Saved"); 
            vTaskDelay(1000); 
            esp_restart(); // 存檔後重開機生效
        }
        cJSON_Delete(root);
    } else httpd_resp_send_500(req);
    return ESP_OK;
}

// POST /api/motor : 設定馬達運轉
static esp_err_t api_motor_handler(httpd_req_t *req) {
    char buf[100], ax[5], st[10], fr[10], di[5];
    // 解析網址參數
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "axis", ax, sizeof(ax));
        httpd_query_key_value(buf, "steps", st, sizeof(st));
        httpd_query_key_value(buf, "freq", fr, sizeof(fr));
        httpd_query_key_value(buf, "dir", di, sizeof(di));
        
        int axis = atoi(ax), steps = atoi(st), freq = atoi(fr), dir = atoi(di);
        if (axis >= 0 && axis < MOTOR_AXIS_COUNT) {
            motors[axis].steps_left = steps; 
            motors[axis].target_steps = steps; // 記錄原始設定值
            motors[axis].freq = freq;
            motors[axis].dir = dir; 
            motors[axis].last_step_time = esp_timer_get_time();
            motors[axis].run = true; // # 設為 true，motor_task 就會開始工作
            
            ESP_LOGI(TAG, "⚙️ [WEB CMD] Axis %d RUN: Steps=%d, Freq=%d, Dir=%d", axis+1, steps, freq, dir);
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// POST /api/motor/stop : 停止馬達
static esp_err_t api_motor_stop_handler(httpd_req_t *req) {
    char buf[50], ax[5];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "axis", ax, sizeof(ax));
        int axis = atoi(ax);
        if (axis >= 0 && axis < MOTOR_AXIS_COUNT) { 
            motors[axis].run = false; 
            motors[axis].steps_left = 0; 
            ESP_LOGW(TAG, "⛔ [WEB CMD] Axis %d STOP Triggered!", axis+1);
        }
    }
    httpd_resp_send(req, "Stopped", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// 註冊所有 API
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); config.max_uri_handlers = 12;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/", .method=HTTP_GET, .handler=root_get_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/status", .method=HTTP_GET, .handler=api_status_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/ota", .method=HTTP_POST, .handler=ota_post_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/api/save_wifi", .method=HTTP_POST, .handler=api_save_wifi_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/api/relay", .method=HTTP_POST, .handler=api_relay_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/api/fan", .method=HTTP_POST, .handler=api_fan_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/api/motor", .method=HTTP_POST, .handler=api_motor_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri="/api/motor/stop", .method=HTTP_POST, .handler=api_motor_stop_handler});
        return server;
    }
    return NULL;
}

// 主程式入口
void app_main(void) {
    // 1. 初始化網路底層 (LwIP) - # 必須最先做，不然 WiFi 會報錯
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    load_settings();

    // 3. 掛載 SPIFFS (網頁檔案)
    esp_vfs_spiffs_conf_t conf = { .base_path="/spiffs", .partition_label="storage", .max_files=5, .format_if_mount_failed=true };
    esp_vfs_spiffs_register(&conf);
    
    // 4. 初始化硬體
    init_hardware();
    
    // 5. 連線 WiFi (失敗則開熱點)
    bool connected = wifi_init_sta();
    if (!connected) { ESP_LOGE(TAG, "WiFi Failed! Starting AP..."); wifi_init_ap(); }

    // 6. 啟動 Web Server
    start_webserver();
    
    // 7. 啟動馬達任務 (在後台執行)
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 10, NULL);
}