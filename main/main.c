/*
 * ESP32-S3 ServoDriver - 韌體版本 (Embedded Web)
 * ==========================================================
 * 專案名稱：ESP32-S3 7軸馬達驅動與遠端控制
 * 
 * 核心架構說明：
 * 1. 【FreeRTOS 多工】：系統同時執行 "馬達控制"、"Web 伺服器"、"WiFi 管理" 三大任務。
 * 2. 【嵌入式網頁】：index.html 在編譯時直接包進韌體，OTA 更新時網頁也會一起更新。
 * 3. 【NVS 記憶】：即使斷電，WiFi 帳密和固定 IP 也不會消失。
 * 4. 【高精度定時】：使用 esp_timer (微秒級) 來產生馬達脈波，確保速度準確。
 * ==========================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h" // 用來控制 PWM 風扇
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
// #include "esp_spiffs.h"  <-- 已移除，我們現在直接用嵌入式網頁
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h" // 高精度計時器

// 引入腳位定義檔 (請確保你有 io_config.h)
#include "io_config.h"

static const char *TAG = "SERVO_DRIVER";

// --- 風扇 PWM 設定 (脈波寬度調變) ---
// 控制散熱風扇轉速 (0~255)
#define FAN_LEDC_TIMER     LEDC_TIMER_0
#define FAN_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL   LEDC_CHANNEL_0
#define FAN_LEDC_RES       LEDC_TIMER_8_BIT // 8-bit 解析度 (0~255)
#define FAN_LEDC_FREQ      5000             // 頻率 5kHz

// --- 全局狀態變數 (供 Web 讀取) ---
static int g_fan_duty = 0;      // 風扇轉速
static bool g_servo_on = false; // 伺服馬達電源是否開啟
static bool g_power_on = false; // 系統總電源是否開啟

// --- 馬達控制結構體 ---
// 每個馬達都有自己的狀態，互不干擾
typedef struct {
    bool run;               // 是否正在運轉?
    int steps_left;         // 還剩多少步要跑?
    int target_steps;       // 這次任務總共要跑幾步 (顯示用)
    int freq;               // 速度 (Hz = 每秒幾步)
    int dir;                // 方向 (1=CW, 0=CCW)
    int64_t last_step_time; // 上一次發出脈波的時間 (微秒)
} MotorState;

MotorState motors[MOTOR_AXIS_COUNT] = {0}; // 建立 7 個軸的狀態

/* ==========================================================
 * ★ 關鍵技術：引用嵌入的 HTML 檔案
 * 這些符號由連結器 (Linker) 自動生成，對應到 CMakeLists.txt 裡的 index.html
 * ========================================================== */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// --- WiFi 預設值 (當 NVS 空白時使用) ---
#define DEFAULT_SSID      "SSID"
#define DEFAULT_PASS      "********"
#define DEFAULT_IP        "192.168.2.124"
#define DEFAULT_GW        "192.168.2.1"
#define DEFAULT_MASK      "255.255.255.0"

// AP 救援模式 (連不上 WiFi 時開啟)
#define AP_SSID           "ESP32-Servo-Rescue"
#define AP_PASS           "" 
#define MAX_RETRY         5  

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0 
#define WIFI_FAIL_BIT      BIT1 
static int s_retry_num = 0;

// 系統設定結構 (暫存 NVS 讀出的資料)
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char static_ip[16];
    char static_gw[16];
    char static_mask[16];
} SystemConfig;
SystemConfig sys_cfg;

/* ================= NVS 讀寫功能 ================= */
// 從 Flash 讀取 WiFi 設定
void load_settings() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) {
        // 開啟失敗 (通常是第一次使用)，載入預設值
        strcpy(sys_cfg.wifi_ssid, DEFAULT_SSID); strcpy(sys_cfg.wifi_pass, DEFAULT_PASS);
        strcpy(sys_cfg.static_ip, DEFAULT_IP); strcpy(sys_cfg.static_gw, DEFAULT_GW); strcpy(sys_cfg.static_mask, DEFAULT_MASK);
        return;
    }
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

// 寫入設定到 Flash
void save_settings(const char* ssid, const char* pass, const char* ip, const char* gw, const char* mask) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "ssid", ssid); nvs_set_str(my_handle, "pass", pass);
        nvs_set_str(my_handle, "ip", ip); nvs_set_str(my_handle, "gw", gw); nvs_set_str(my_handle, "mask", mask);
        nvs_commit(my_handle); // ★ 必須 Commit 才會真正寫入
        nvs_close(my_handle);
    }
}

/* ================= 硬體初始化 ================= */
void init_hardware(void) {
    // 1. 設定系統控制腳位 (輸出)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << SYS_START_TRIGGER_GPIO) | (1ULL << SYS_POWER_CUTOFF_GPIO) | (1ULL << SERVO_ENABLE_GPIO);
    io_conf.pull_down_en = 0; io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // 預設關閉
    gpio_set_level(SYS_START_TRIGGER_GPIO, 0);
    gpio_set_level(SYS_POWER_CUTOFF_GPIO, 0);
    gpio_set_level(SERVO_ENABLE_GPIO, 0);

    // 2. 設定感測器與按鈕 (輸入)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BTN_STOP_GPIO) | (1ULL << SYS_STATE_FEEDBACK_GPIO);
    io_conf.pull_up_en = 1; // 啟用上拉電阻 (Input Pull-up)
    gpio_config(&io_conf);

    // 3. 設定馬達腳位 (Pulse + Dir)
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

    // 4. 設定風扇 PWM
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
        esp_wifi_connect(); // WiFi 啟動即連線
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) { 
            esp_wifi_connect(); s_retry_num++; 
            ESP_LOGW(TAG, "重試連線 (%d/%d)...", s_retry_num, MAX_RETRY);
        } else { 
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); 
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0; 
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
    
    // 設定固定 IP
    esp_netif_dhcpc_stop(my_sta);
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = esp_ip4addr_aton(sys_cfg.static_ip);
    ip_info.gw.addr = esp_ip4addr_aton(sys_cfg.static_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(sys_cfg.static_mask);
    esp_netif_set_ip_info(my_sta, &ip_info);
    
    // 設定 DNS (8.8.8.8)
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

/* ================= 核心：馬達控制任務 (獨立執行緒) ================= */
// 這個任務會一直跑，不會被網頁或 WiFi 卡住
void motor_task(void *arg) {
    while (1) {
        int64_t now = esp_timer_get_time(); // 獲取當前微秒時間
        bool any_running = false;

        // 輪詢所有 7 個軸
        for (int i = 0; i < MOTOR_AXIS_COUNT; i++) {
            if (motors[i].run) {
                any_running = true;
                // 計算脈波間隔 (微秒) = 1,000,000 / 頻率
                int interval_us = 1000000 / motors[i].freq;

                // 時間到了嗎？
                if (now - motors[i].last_step_time >= interval_us) {
                    motors[i].last_step_time = now;

                    // 1. 設定方向
                    gpio_set_level(MOTOR_PINS[i].dir_pin, motors[i].dir);

                    // 2. 產生脈波 (High -> 5us -> Low)
                    gpio_set_level(MOTOR_PINS[i].pul_pin, 1);
                    esp_rom_delay_us(5); // 確保驅動器讀得到
                    gpio_set_level(MOTOR_PINS[i].pul_pin, 0);
                    
                    // 3. 步數倒數
                    if (motors[i].steps_left > 0) {
                        motors[i].steps_left--;
                        if (motors[i].steps_left == 0) {
                            motors[i].run = false; // 跑完自動停止
                        }
                    }
                }
            }
        }
        
        // 智慧延遲：如果有馬達在跑，延遲極短 (1ms) 以保證速度；如果沒馬達跑，延遲久一點 (10ms) 省電
        if (!any_running) vTaskDelay(pdMS_TO_TICKS(10));
        else vTaskDelay(1); 
    }
}

/* ================= 網頁伺服器 API ================= */

// GET / : 回傳嵌入的 HTML 頁面
static esp_err_t root_get_handler(httpd_req_t *req) {
    // index_html_start 指向記憶體中的 HTML 開頭
    // index_html_end - index_html_start = 檔案長度
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

// POST /api/relay : 控制繼電器 (啟動/斷電/始能)
static esp_err_t api_relay_handler(httpd_req_t *req) {
    char buf[100], type[20]={0}, state_str[5]={0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "type", type, sizeof(type));
        httpd_query_key_value(buf, "state", state_str, sizeof(state_str));
        int state = atoi(state_str);
        
        if (strcmp(type, "start")==0 && state==1) {
            // 點動啟動: 按下 500ms 後放開
            gpio_set_level(SYS_START_TRIGGER_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(SYS_START_TRIGGER_GPIO, 0);
            g_power_on = true;
            ESP_LOGW(TAG, "🔥 系統啟動 (System Start)");
        } else if (strcmp(type, "cutoff")==0 && state==1) {
            // 斷電: 按下 500ms 後放開
            gpio_set_level(SYS_POWER_CUTOFF_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(SYS_POWER_CUTOFF_GPIO, 0);
            g_power_on = false; g_servo_on = false;
            ESP_LOGW(TAG, "⛔ 系統斷電 (System Cutoff)");
        } else if (strcmp(type, "enable")==0) {
            // 伺服始能: 保持狀態
            gpio_set_level(SERVO_ENABLE_GPIO, state);
            g_servo_on = (state == 1);
            ESP_LOGW(TAG, "⚡ 伺服始能 (Servo Enable): %s", state ? "ON" : "OFF");
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// POST /api/fan : 設定風扇轉速
static esp_err_t api_fan_handler(httpd_req_t *req) {
    char buf[50], val_str[10]={0};
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "val", val_str, sizeof(val_str));
        int duty = atoi(val_str);
        ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
        ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
        g_fan_duty = duty;
        ESP_LOGI(TAG, "🌀 風扇轉速設定: %d", duty);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// GET /status : 回傳 JSON 狀態
static esp_err_t api_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    // 讀取真實 IO 狀態
    int stop_btn_val = gpio_get_level(BTN_STOP_GPIO); 
    int feedback_val = gpio_get_level(SYS_STATE_FEEDBACK_GPIO);

    cJSON_AddNumberToObject(root, "temp", 42.5); // 模擬溫度
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

    // 馬達狀態
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

// OTA 更新任務
static void ota_task(void *arg) {
    char *url = (char *)arg;
    ESP_LOGI(TAG, "開始 OTA 更新: %s", url);
    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
        .buffer_size = 16384, // 16KB Buffer
        .buffer_size_tx = 4096,
        .timeout_ms = 30000,
    };
    esp_https_ota_config_t ota_config = { .http_config = &http_cfg };
    if (esp_https_ota(&ota_config) == ESP_OK) { 
        ESP_LOGW(TAG, "✅ OTA 更新成功! 系統重啟中...");
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        esp_restart(); 
    } else {
        ESP_LOGE(TAG, "❌ OTA 更新失敗!");
    }
    free(url); vTaskDelete(NULL);
}

// POST /ota : 觸發 OTA
static esp_err_t ota_post_handler(httpd_req_t *req) {
    char buf[512]; int ret = httpd_req_recv(req, buf, MIN(req->content_len, 511));
    if(ret<=0) return ESP_FAIL; 
    buf[ret]=0;
    
    cJSON *root = cJSON_Parse(buf);
    if(root){
        cJSON *url = cJSON_GetObjectItem(root, "url");
        if(cJSON_IsString(url)) { 
            char *p = strdup(url->valuestring); 
            xTaskCreate(ota_task, "ota_task", 8192, p, 5, NULL);
            httpd_resp_sendstr(req, "Started"); 
        }
        cJSON_Delete(root);
    } else httpd_resp_send_500(req);
    return ESP_OK;
}

// POST /api/save_wifi : 儲存 WiFi 設定
static esp_err_t api_save_wifi_handler(httpd_req_t *req) {
    char buf[512]; int ret = httpd_req_recv(req, buf, MIN(req->content_len, 511));
    if(ret<=0) return ESP_FAIL; 
    buf[ret]=0;
    
    cJSON *root = cJSON_Parse(buf);
    if(root) {
        cJSON *s=cJSON_GetObjectItem(root,"ssid"), *p=cJSON_GetObjectItem(root,"pass");
        cJSON *i=cJSON_GetObjectItem(root,"ip"), *g=cJSON_GetObjectItem(root,"gw");
        if(cJSON_IsString(s) && cJSON_IsString(p)) {
            save_settings(s->valuestring, p->valuestring, i?i->valuestring:DEFAULT_IP, g?g->valuestring:DEFAULT_GW, "255.255.255.0");
            httpd_resp_sendstr(req, "Saved"); 
            vTaskDelay(1000); 
            esp_restart(); 
        }
        cJSON_Delete(root);
    } else httpd_resp_send_500(req);
    return ESP_OK;
}

// POST /api/motor : 設定馬達參數並啟動
static esp_err_t api_motor_handler(httpd_req_t *req) {
    char buf[100], ax[5], st[10], fr[10], di[5];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "axis", ax, sizeof(ax));
        httpd_query_key_value(buf, "steps", st, sizeof(st));
        httpd_query_key_value(buf, "freq", fr, sizeof(fr));
        httpd_query_key_value(buf, "dir", di, sizeof(di));
        
        int axis = atoi(ax), steps = atoi(st), freq = atoi(fr), dir = atoi(di);
        if (axis >= 0 && axis < MOTOR_AXIS_COUNT) {
            motors[axis].steps_left = steps; 
            motors[axis].target_steps = steps;
            motors[axis].freq = freq;
            motors[axis].dir = dir; 
            motors[axis].last_step_time = esp_timer_get_time();
            motors[axis].run = true; // 啟動馬達 (motor_task 會偵測到此變數)
            
            ESP_LOGI(TAG, "⚙️ 馬達啟動 - 軸%d: %d步, %dHz, 方向%d", axis+1, steps, freq, dir);
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// POST /api/motor/stop : 緊急停止馬達
static esp_err_t api_motor_stop_handler(httpd_req_t *req) {
    char buf[50], ax[5];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "axis", ax, sizeof(ax));
        int axis = atoi(ax);
        if (axis >= 0 && axis < MOTOR_AXIS_COUNT) { 
            motors[axis].run = false; 
            motors[axis].steps_left = 0; 
            ESP_LOGW(TAG, "⛔ 馬達停止 - 軸%d", axis+1);
        }
    }
    httpd_resp_send(req, "Stopped", HTTPD_RESP_USE_STRLEN); return ESP_OK;
}

// 註冊所有 API 路由
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
    // 1. 初始化 NVS (NVS 必須最先初始化，因為 WiFi 會用到)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    load_settings();

    // 2. 初始化網路介面 (LwIP)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. 初始化硬體 IO
    init_hardware();
    
    // 4. 連線 WiFi (失敗則開熱點)
    bool connected = wifi_init_sta();
    if (!connected) { ESP_LOGE(TAG, "WiFi 連線失敗! 開啟救援熱點..."); wifi_init_ap(); }

    // 5. 啟動 Web Server
    start_webserver();
    
    // 6. 啟動馬達控制任務 (優先級 10，比 Web Server 高)
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "系統啟動完成! 訪問 IP: http://%s", sys_cfg.static_ip);
}