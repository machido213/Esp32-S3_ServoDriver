#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "cJSON.h"          // 引入 JSON 處理庫，用來打包狀態回傳給網頁
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h" // 用於 HTTPS 憑證驗證 (GitHub 需要)
#include "io_config.h"      // 這是我們自己寫的腳位定義檔

// 引用外部 HTML 檔案 (這是編譯器生成的二進位符號)
// 這樣我們就不用把 HTML 寫在 C 語言的字串裡，方便管理
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// ==========================================
// 1. 網路設定區
// ==========================================
#define WIFI_SSID      "ASUS-N18U"
#define WIFI_PASS      "0922293650"

// 固定 IP 設定 (請依照你的路由器網段修改)
#define STATIC_IP_ADDR "192.168.2.124"
#define STATIC_NETMASK "255.255.255.0"
#define STATIC_GATEWAY "192.168.2.1"

// ★★★ 除錯按鈕 ★★★
// 使用板子上的 BOOT 鍵 (通常是 GPIO 0)
// 按下時會在終端機印出系統狀態，確認程式沒當機
#define DEBUG_BTN_GPIO 0

static const char *TAG = "SERVO_DRIVER";

// 用來等待 WiFi 連線成功的旗標 (EventGroup)
// 這能讓程式暫停在啟動階段，直到網路通了才繼續往下跑
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ==========================================
// 2. 馬達控制結構與全域變數
// ==========================================
typedef struct {
    int steps_target; // 目標要走的步數
    int freq;         // 脈波頻率 (速度 Hz)
    int dir;          // 方向 (1:正轉/CW, 0:反轉/CCW)
    bool running;     // 狀態旗標：true 代表正在發脈波
    bool stop_req;    // 停止請求：設為 true 會強制中斷發脈波
} motor_ctrl_t;

// 建立 7 個軸的狀態陣列，用來儲存每個馬達的參數
motor_ctrl_t motor_states[MOTOR_AXIS_COUNT];

// ==========================================
// 3. 硬體初始化 (GPIO / PWM)
// ==========================================
void init_hardware() {
    gpio_config_t io_conf = {};
    
    // --- 1. 設定輸出腳位 (Output) ---
    // 包含：伺服始能 (Enable)、啟動繼電器、斷電繼電器、以及所有馬達的 Pulse/Dir
    uint64_t out_mask = (1ULL<<SERVO_ENABLE_GPIO) | (1ULL<<SYS_START_TRIGGER_GPIO) | (1ULL<<SYS_POWER_CUTOFF_GPIO);
    for(int i=0; i<MOTOR_AXIS_COUNT; i++){
        out_mask |= (1ULL<<MOTOR_PINS[i].pul_pin) | (1ULL<<MOTOR_PINS[i].dir_pin);
    }
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = out_mask;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // --- 2. 設定輸入腳位 (Input) ---
    // 包含：停止按鈕、接觸器回饋、以及除錯按鈕(BOOT)
    // 這裡開啟上拉電阻 (Pull-up)，這樣按鈕沒按時是 High(1)，按下接地變 Low(0)
    uint64_t in_mask = (1ULL<<BTN_STOP_GPIO) | (1ULL<<SYS_STATE_FEEDBACK_GPIO) | (1ULL<<DEBUG_BTN_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = in_mask;
    io_conf.pull_up_en = 1; 
    gpio_config(&io_conf);

    // --- 3. 設定風扇 PWM (使用 LEDC 模組) ---
    ledc_timer_config_t ledc_timer = { 
        .speed_mode=LEDC_LOW_SPEED_MODE, 
        .timer_num=LEDC_TIMER_0, 
        .duty_resolution=LEDC_TIMER_8_BIT, // 0~255
        .freq_hz=25000,                    // 25kHz (適合大多風扇)
        .clk_cfg=LEDC_AUTO_CLK 
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = { 
        .speed_mode=LEDC_LOW_SPEED_MODE, 
        .channel=LEDC_CHANNEL_0, 
        .timer_sel=LEDC_TIMER_0, 
        .gpio_num=FAN_PWM_GPIO, 
        .duty=0 // 預設關閉
    };
    ledc_channel_config(&ledc_channel);
}

// ==========================================
// 4. 馬達運轉任務 (每軸一個 Task)
// ==========================================
// 這個任務會一直跑，當 detecting 到 running=true 時發送脈波
void motor_task(void *arg) {
    int axis = (int)arg; // 取得這個任務負責第幾軸 (0~6)
    while(1) {
        if (motor_states[axis].running) {
            int pul = MOTOR_PINS[axis].pul_pin;
            int dir = MOTOR_PINS[axis].dir_pin;
            
            // 計算延遲時間：頻率越高，延遲越短
            // 週期 = 1/Freq，High/Low 各佔一半所以除以 2
            int delay_us = 1000000 / (motor_states[axis].freq * 2);
            int steps = motor_states[axis].steps_target;

            // 設定方向
            gpio_set_level(dir, motor_states[axis].dir);
            
            // 開始迴圈發送脈波
            for (int i=0; i<steps; i++) {
                // 緊急停止檢查：如果收到 stop_req，立刻跳出迴圈
                if (motor_states[axis].stop_req) break;
                
                gpio_set_level(pul, 1);
                esp_rom_delay_us(delay_us);
                gpio_set_level(pul, 0);
                esp_rom_delay_us(delay_us);
                
                // 每發 100 步讓出一點 CPU 時間，避免 Watchdog 認為程式卡死
                if(i%100==0) vTaskDelay(1); 
            }
            // 任務完成，重置狀態
            motor_states[axis].running = false;
            motor_states[axis].stop_req = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 閒置時休息，釋放 CPU 資源
    }
}

// ==========================================
// 5. OTA (線上韌體更新) 功能
// ==========================================
esp_err_t _http_event_handler(esp_http_client_event_t *evt) { return ESP_OK; }

void ota_task(void *pvParameter) {
    char *url = (char *)pvParameter;
    ESP_LOGI(TAG, "Starting OTA update task...");
    ESP_LOGI(TAG, "Target URL: %s", url);

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .keep_alive_enable = true,
        .crt_bundle_attach = esp_crt_bundle_attach, // 支援 GitHub HTTPS 憑證
        .skip_cert_common_name_check = true,
        .timeout_ms = 10000, // 10秒超時
    };

    esp_https_ota_config_t ota_config = { .http_config = &config };

    // 開始下載並更新
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "OTA Update Successful! Rebooting...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart(); // 更新成功，重啟生效
    } else {
        ESP_LOGE(TAG, "OTA Failed! Error Code: 0x%x", ret);
        ESP_LOGE(TAG, "Common reasons: DNS failed, 404 Not Found, WiFi unstable.");
    }
    
    free(url);         // 釋放記憶體
    vTaskDelete(NULL); // 任務結束，刪除自己
}

// ==========================================
// 6. Web Server API (處理網頁請求)
// ==========================================

// --- 首頁 Handler (GET /) ---
esp_err_t root_handler(httpd_req_t *req) {
    // 計算 HTML 檔案大小
    size_t html_len = index_html_end - index_html_start;
    // 發送 HTML 內容給瀏覽器
    httpd_resp_send(req, (const char *)index_html_start, html_len);
    return ESP_OK;
}

// --- 狀態回傳 Handler (GET /status) ---
esp_err_t status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    // 1. 回傳溫度 (目前寫死，之後可接感測器)
    cJSON_AddNumberToObject(root, "temp", 42.5); 
    // 2. 回傳 IO 狀態 (按鈕是否按下、接觸器是否吸合)
    cJSON_AddNumberToObject(root, "btn_stop", gpio_get_level(BTN_STOP_GPIO));
    cJSON_AddNumberToObject(root, "feedback", gpio_get_level(SYS_STATE_FEEDBACK_GPIO));

    // 3. 回傳所有馬達的狀態
    cJSON *motors = cJSON_CreateArray();
    for (int i = 0; i < MOTOR_AXIS_COUNT; i++) {
        cJSON *mtr = cJSON_CreateObject();
        cJSON_AddBoolToObject(mtr, "run", motor_states[i].running);
        cJSON_AddNumberToObject(mtr, "steps", motor_states[i].steps_target);
        cJSON_AddNumberToObject(mtr, "freq", motor_states[i].freq);
        cJSON_AddItemToArray(motors, mtr);
    }
    cJSON_AddItemToObject(root, "motors", motors);

    // 轉成 JSON 字串發送
    const char *sys_info = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, sys_info, strlen(sys_info));

    cJSON_Delete(root);
    free((void *)sys_info);
    return ESP_OK;
}

// --- OTA 請求 Handler (POST /ota) ---
esp_err_t ota_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    
    // 解析 JSON 取得 URL
    cJSON *root = cJSON_Parse(buf);
    if (root) {
        cJSON *url_item = cJSON_GetObjectItem(root, "url");
        if (cJSON_IsString(url_item)) {
            // 在終端機印出 Log，確認收到更新請求
            ESP_LOGW(TAG, "🚀 [WEB CMD] OTA Requested from: %s", url_item->valuestring);
            
            // 複製 URL 並啟動 OTA 任務
            char *url = strdup(url_item->valuestring);
            xTaskCreate(&ota_task, "ota_task", 8192, url, 5, NULL);
            httpd_resp_send(req, "OTA Started", HTTPD_RESP_USE_STRLEN);
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

// --- 馬達啟動 Handler (POST /api/motor) ---
esp_err_t motor_handler(httpd_req_t *req) {
    char buf[100], val[10];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        int axis = 0;
        if (httpd_query_key_value(buf, "axis", val, sizeof(val)) == ESP_OK) axis = atoi(val);
        motor_states[axis].stop_req = false;
        
        // 讀取 URL 參數
        if (httpd_query_key_value(buf, "steps", val, sizeof(val)) == ESP_OK) motor_states[axis].steps_target = atoi(val);
        if (httpd_query_key_value(buf, "freq", val, sizeof(val)) == ESP_OK) motor_states[axis].freq = atoi(val);
        if (httpd_query_key_value(buf, "dir", val, sizeof(val)) == ESP_OK) motor_states[axis].dir = atoi(val);
        
        // 設定旗標，讓馬達任務開始跑
        motor_states[axis].running = true;
        
        // ★★★ 終端機 Log (讓你看到按鈕按下後的反應) ★★★
        ESP_LOGI(TAG, "⚙️ [WEB CMD] Axis %d RUN: Steps=%d, Freq=%d, Dir=%d", 
                 axis + 1, motor_states[axis].steps_target, motor_states[axis].freq, motor_states[axis].dir);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// --- 馬達停止 Handler (POST /api/motor/stop) ---
esp_err_t motor_stop_handler(httpd_req_t *req) {
    char buf[50], val[5];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "axis", val, sizeof(val)) == ESP_OK) {
            int axis = atoi(val);
            motor_states[axis].stop_req = true;
            // ★★★ 終端機 Log ★★★
            ESP_LOGW(TAG, "⛔ [WEB CMD] Axis %d STOP Triggered!", axis + 1);
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// --- 繼電器控制 Handler (POST /api/relay) ---
esp_err_t relay_handler(httpd_req_t *req) {
    char buf[100], type[10], state[5];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "type", type, sizeof(type));
        httpd_query_key_value(buf, "state", state, sizeof(state));
        int lvl = atoi(state);
        
        if (strcmp(type, "start") == 0) {
            // 收到啟動指令
            ESP_LOGW(TAG, "🔥 [WEB CMD] SYSTEM START Triggered!"); 
            gpio_set_level(SYS_START_TRIGGER_GPIO, lvl);
            // 模擬點動開關：吸合 0.5 秒後自動放開
            if(lvl) { vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(SYS_START_TRIGGER_GPIO, 0); }
        } else if (strcmp(type, "cutoff") == 0) {
            // 收到斷電指令
            ESP_LOGW(TAG, "⛔ [WEB CMD] SYSTEM CUTOFF Triggered!");
            gpio_set_level(SYS_POWER_CUTOFF_GPIO, lvl);
        } else if (strcmp(type, "enable") == 0) {
            // 收到伺服始能指令
            ESP_LOGW(TAG, "⚡ [WEB CMD] SERVO ENABLE set to: %s", lvl ? "ON" : "OFF");
            gpio_set_level(SERVO_ENABLE_GPIO, lvl);
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// --- 風扇控制 Handler (POST /api/fan) ---
esp_err_t fan_handler(httpd_req_t *req) {
    char buf[50], val[10];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        httpd_query_key_value(buf, "val", val, sizeof(val));
        int pwm = atoi(val);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        // 如果想看風扇的 Log 也可以把下面註解打開
        // ESP_LOGI(TAG, "🌀 [WEB CMD] Fan PWM: %d", pwm);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 註冊所有 URI (路由)
void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; // 增加 Stack 防止 JSON 處理時記憶體不足
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
        httpd_uri_t uri_stat = { .uri = "/status", .method = HTTP_GET, .handler = status_handler };
        httpd_uri_t uri_ota  = { .uri = "/ota", .method = HTTP_POST, .handler = ota_handler };
        httpd_uri_t uri_motor = { .uri = "/api/motor", .method = HTTP_POST, .handler = motor_handler };
        httpd_uri_t uri_mstop = { .uri = "/api/motor/stop", .method = HTTP_POST, .handler = motor_stop_handler };
        httpd_uri_t uri_relay = { .uri = "/api/relay", .method = HTTP_POST, .handler = relay_handler };
        httpd_uri_t uri_fan = { .uri = "/api/fan", .method = HTTP_POST, .handler = fan_handler };
        
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_stat);
        httpd_register_uri_handler(server, &uri_ota);
        httpd_register_uri_handler(server, &uri_motor);
        httpd_register_uri_handler(server, &uri_mstop);
        httpd_register_uri_handler(server, &uri_relay);
        httpd_register_uri_handler(server, &uri_fan);
    }
}

// ==========================================
// 7. WiFi 設定
// ==========================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi Disconnected. Retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        // 連線成功時印出 IP 資訊
        ESP_LOGI(TAG, "WiFi Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_static() {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *my_netif = esp_netif_create_default_wifi_sta();

    // 1. 停止 DHCP (為了用固定 IP)
    esp_netif_dhcpc_stop(my_netif);

    // 2. 設定固定 IP
    esp_netif_ip_info_t ip_info;
    esp_netif_str_to_ip4(STATIC_IP_ADDR, &ip_info.ip);
    esp_netif_str_to_ip4(STATIC_GATEWAY, &ip_info.gw);
    esp_netif_str_to_ip4(STATIC_NETMASK, &ip_info.netmask);
    esp_netif_set_ip_info(my_netif, &ip_info);

    // 3. 設定 DNS (解決 GitHub 找不到的問題)
    // 這裡使用安全的寫法，相容 ESP-IDF v5.x
    esp_netif_dns_info_t dns = {};
    ip_addr_t dns_addr;
    ipaddr_aton("8.8.8.8", &dns_addr); // Google DNS
    dns.ip.u_addr.ip4.addr = dns_addr.u_addr.ip4.addr;
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(my_netif, ESP_NETIF_DNS_MAIN, &dns);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    
    // 設定 SSID / Password
    wifi_config_t wifi_config = { 
        .sta = { 
            .ssid = WIFI_SSID, 
            .password = WIFI_PASS, 
            .threshold.authmode = WIFI_AUTH_WPA2_PSK // 強制 WPA2 安全性
        } 
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

// ==========================================
// 8. 自動化邏輯 (狀態機)
// ==========================================
void automation_task(void *arg) {
    while (1) {
        int stop_btn = gpio_get_level(BTN_STOP_GPIO);           // 停止按鈕
        int feedback = gpio_get_level(SYS_STATE_FEEDBACK_GPIO); // 接觸器回饋

        // --- 1. 自動開啟伺服始能 ---
        // 條件：接觸器吸合 (feedback==0) 且 停止按鈕沒被按 (stop==1)
        if (feedback == 0 && stop_btn == 1) {
            gpio_set_level(SERVO_ENABLE_GPIO, 1);
        }

        // --- 2. 軟停止與斷電 ---
        // 條件：停止按鈕被按下 (stop==0)
        if (stop_btn == 0) {
            ESP_LOGW(TAG, "Stop Button Pressed! Executing Safety Sequence...");
            
            // A. 關閉伺服始能 (脫力)
            gpio_set_level(SERVO_ENABLE_GPIO, 0); 
            
            // B. 停止所有馬達
            for(int i=0; i<MOTOR_AXIS_COUNT; i++) motor_states[i].stop_req = true;
            
            // C. 切斷接觸器電源
            gpio_set_level(SYS_POWER_CUTOFF_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); // 保持斷電訊號 1 秒
            gpio_set_level(SYS_POWER_CUTOFF_GPIO, 0); // 復歸
            
            // D. 等待按鈕放開，防止重複觸發
            while(gpio_get_level(BTN_STOP_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 每 0.1 秒檢查一次
    }
}

// ==========================================
// 9. 除錯按鈕任務 (Alive Check)
// ==========================================
// 當按下 BOOT 鍵時，印出系統狀態
void debug_button_task(void *arg) {
    while(1) {
        if (gpio_get_level(DEBUG_BTN_GPIO) == 0) {
            ESP_LOGI(TAG, "==========================================");
            ESP_LOGI(TAG, "🟢 [ALIVE CHECK] System is RUNNING!");
            ESP_LOGI(TAG, "💾 Free Heap: %lu bytes", esp_get_free_heap_size());
            ESP_LOGI(TAG, "📡 WiFi Status: %s", 
                     (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) ? "Connected" : "Disconnected");
            ESP_LOGI(TAG, "==========================================");
            
            vTaskDelay(pdMS_TO_TICKS(500)); // 簡單的防彈跳
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 每 0.1 秒檢查一次
    }
}

// ==========================================
// 10. 主程式入口
// ==========================================
void app_main(void) {
    // 1. 初始化 NVS (Flash 存儲)，WiFi 需要用到
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化硬體 GPIO / PWM
    init_hardware();
    
    // 3. 啟動 WiFi (固定 IP)
    wifi_init_static();

    // 4. 等待 WiFi 連線成功
    // 這裡會卡住直到連上，期間每秒印一次 "Waiting..."
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(1000));
        if (bits & WIFI_CONNECTED_BIT) break;
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
    }

    // 5. 啟動任務 (Tasks)
    // 馬達任務 (7個)
    for(int i=0; i<MOTOR_AXIS_COUNT; i++) xTaskCreate(motor_task, "mtr", 2048, (void*)i, 5, NULL);
    // 自動化邏輯任務
    xTaskCreate(automation_task, "auto", 4096, NULL, 5, NULL);
    // 除錯按鈕任務
    xTaskCreate(debug_button_task, "debug_btn", 2048, NULL, 5, NULL);

    // 6. 啟動網頁伺服器
    start_webserver();
}