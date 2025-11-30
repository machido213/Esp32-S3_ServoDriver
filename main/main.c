#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

// --- ESP-IDF 核心元件 ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" // 新增：用於等待 WiFi 連線結果
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

static const char *TAG = "SERVO_DRIVER";

// ==========================================================
// 1. 系統設定與全域變數
// ==========================================================

// 預設值 (若 NVS 空的，先嘗試連這個，若失敗會轉 AP)
#define DEFAULT_WIFI_SSID "Gulitong"
#define DEFAULT_WIFI_PASS "0932293650"
#define DEFAULT_IP        "192.168.1.124"
#define DEFAULT_GW        "192.168.1.1"
#define DEFAULT_MASK      "255.255.255.0"

// AP 救援模式的設定
#define AP_SSID           "ESP32-ServoDriver-Rescue-Mode" // 熱點名稱
#define AP_PASS           ""                  // 熱點密碼 (空代表不設密碼)
#define AP_MAX_CONN       4

// 事件群組 bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define MAX_RETRY 5  // 最大重試次數

// 設定結構
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char static_ip[16];
    char static_gw[16];
    char static_mask[16];
} SystemConfig;

SystemConfig sys_cfg;

// ==========================================================
// 2. NVS 讀寫功能
// ==========================================================
void load_settings() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed, loading defaults.");
        strcpy(sys_cfg.wifi_ssid, DEFAULT_WIFI_SSID);
        strcpy(sys_cfg.wifi_pass, DEFAULT_WIFI_PASS);
        strcpy(sys_cfg.static_ip, DEFAULT_IP);
        strcpy(sys_cfg.static_gw, DEFAULT_GW);
        strcpy(sys_cfg.static_mask, DEFAULT_MASK);
        return;
    }
    size_t size = sizeof(sys_cfg.wifi_ssid);
    if (nvs_get_str(my_handle, "ssid", sys_cfg.wifi_ssid, &size) != ESP_OK) strcpy(sys_cfg.wifi_ssid, DEFAULT_WIFI_SSID);
    size = sizeof(sys_cfg.wifi_pass);
    if (nvs_get_str(my_handle, "pass", sys_cfg.wifi_pass, &size) != ESP_OK) strcpy(sys_cfg.wifi_pass, DEFAULT_WIFI_PASS);
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
        nvs_set_str(my_handle, "ssid", ssid);
        nvs_set_str(my_handle, "pass", pass);
        nvs_set_str(my_handle, "ip", ip);
        nvs_set_str(my_handle, "gw", gw);
        nvs_set_str(my_handle, "mask", mask);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Settings Saved to NVS");
    }
}

// ==========================================================
// 3. WiFi 事件處理 (Event Handler)
// ==========================================================
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // 當 WiFi 開始時
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    // 當 WiFi 斷線時
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAX_RETRY);
        } else {
            // 超過重試次數，標記為失敗
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    // 當成功取得 IP 時
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ==========================================================
// 4. WiFi 初始化模式：Station (連線模式)
// ==========================================================
// 回傳 true 代表連線成功，false 代表失敗
static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();

    // 設定 Static IP
    esp_netif_dhcpc_stop(my_sta);
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = esp_ip4addr_aton(sys_cfg.static_ip);
    ip_info.gw.addr = esp_ip4addr_aton(sys_cfg.static_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(sys_cfg.static_mask);
    esp_netif_set_ip_info(my_sta, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, sys_cfg.wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, sys_cfg.wifi_pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s ...", sys_cfg.wifi_ssid);

    // 等待連線結果 (WIFI_CONNECTED_BIT or WIFI_FAIL_BIT)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", sys_cfg.wifi_ssid);
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", sys_cfg.wifi_ssid);
        return false;
    }
    return false;
}

// ==========================================================
// 5. WiFi 初始化模式：Access Point (熱點模式)
// ==========================================================
static void wifi_init_ap(void)
{
    // 如果之前已經初始化過 netif (在 STA 失敗時)，這裡需要小心處理
    // 為了簡單起見，我們通常在 STA 失敗後直接用同一套 stack 切換模式，
    // 但因為有 Static IP 的影響，最好是重置或直接使用新的 AP 邏輯。
    
    // 停止 STA
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_NULL);

    // 重新配置為 AP
    esp_netif_t *my_ap = esp_netif_create_default_wifi_ap();
    // 注意：AP 模式預設 IP 為 192.168.4.1 (由 DHCP Server 分配)

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // 如果前面 init 過，這裡可以直接用，或者重新 init
    // 由於 esp_wifi_init 只能呼叫一次，這裡我們假設前面 STA 雖然失敗但 driver 已載入
    // 所以我們只需要設 Config

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN // 如果有密碼改 WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGW(TAG, "!!! SWITCHED TO AP MODE !!!");
    ESP_LOGW(TAG, "SSID: %s, IP: 192.168.4.1", AP_SSID);
}

// ==========================================================
// 6. HTTP Server Handlers
// ==========================================================

// 讀取 index.html
static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE* f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0); 
    return ESP_OK;
}

// 儲存 WiFi 設定 API
static esp_err_t api_save_wifi_handler(httpd_req_t *req)
{
    char buf[512];
    int ret = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf) - 1));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return ESP_FAIL;

    cJSON *j_ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *j_pass = cJSON_GetObjectItem(root, "pass");
    cJSON *j_ip = cJSON_GetObjectItem(root, "ip");
    cJSON *j_gw = cJSON_GetObjectItem(root, "gw");

    if (cJSON_IsString(j_ssid) && cJSON_IsString(j_pass) && cJSON_IsString(j_ip)) {
        save_settings(
            j_ssid->valuestring,
            j_pass->valuestring,
            j_ip->valuestring,
            (cJSON_IsString(j_gw)) ? j_gw->valuestring : DEFAULT_GW,
            "255.255.255.0"
        );
        httpd_resp_send(req, "Saved. Rebooting...", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        httpd_resp_send_500(req);
    }
    cJSON_Delete(root);
    return ESP_OK;
}

// 狀態 API
static esp_err_t api_status_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    
    cJSON_AddNumberToObject(root, "temp", 42.5);
    cJSON_AddNumberToObject(root, "btn_stop", 1); 
    cJSON_AddNumberToObject(root, "feedback", 1); 
    
    cJSON *net = cJSON_CreateObject();
    cJSON_AddStringToObject(net, "ssid", sys_cfg.wifi_ssid);
    cJSON_AddStringToObject(net, "ip", sys_cfg.static_ip);
    cJSON_AddStringToObject(net, "gw", sys_cfg.static_gw);
    cJSON_AddItemToObject(root, "net", net);

    cJSON *motors = cJSON_CreateArray();
    for(int i=0; i<7; i++) {
        cJSON *m = cJSON_CreateObject();
        cJSON_AddBoolToObject(m, "run", false); 
        cJSON_AddItemToArray(motors, m);
    }
    cJSON_AddItemToObject(root, "motors", motors);

    const char *resp = cJSON_PrintUnformatted(root);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    free((void*)resp);
    cJSON_Delete(root);
    return ESP_OK;
}

// 馬達 API (Placeholder)
static esp_err_t api_motor_handler(httpd_req_t *req) {
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); return ESP_OK; 
}
static esp_err_t api_motor_stop_handler(httpd_req_t *req) {
    httpd_resp_send(req, "Stopped", HTTPD_RESP_USE_STRLEN); return ESP_OK; 
}

// 註冊 URI
static const httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
static const httpd_uri_t uri_api_status = { .uri = "/status", .method = HTTP_GET, .handler = api_status_handler };
static const httpd_uri_t uri_api_save_wifi = { .uri = "/api/save_wifi", .method = HTTP_POST, .handler = api_save_wifi_handler };
static const httpd_uri_t uri_api_motor = { .uri = "/api/motor", .method = HTTP_POST, .handler = api_motor_handler };
static const httpd_uri_t uri_api_stop = { .uri = "/api/motor/stop", .method = HTTP_POST, .handler = api_motor_stop_handler };

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    // 為了在 AP 模式下也能同時多人連線，可以適當調整
    config.max_open_sockets = 7;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_api_status);
        httpd_register_uri_handler(server, &uri_api_save_wifi);
        httpd_register_uri_handler(server, &uri_api_motor);
        httpd_register_uri_handler(server, &uri_api_stop);
        return server;
    }
    return NULL;
}

// ==========================================================
// 7. 主程式入口
// ==========================================================
void app_main(void)
{
    // 1. Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Load Settings
    load_settings();

    // 3. Mount SPIFFS
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = true
    };
    esp_vfs_spiffs_register(&conf);

    // 4. Try Connect WiFi (STA)
    // 嘗試連線，如果回傳 true 代表成功，false 代表失敗(超過重試次數)
    bool connected = wifi_init_sta();

    if (!connected) {
        // 連線失敗，啟動 AP 救援模式
        ESP_LOGE(TAG, "WiFi Connection Failed! Starting AP Mode for Provisioning...");
        wifi_init_ap();
    }

    // 5. Start Web Server
    // 無論是 STA 模式 (有網路) 還是 AP 模式 (救援)，都要啟動網頁伺服器
    start_webserver();

    if (connected) {
        ESP_LOGI(TAG, "System Running in Normal Mode. IP: %s", sys_cfg.static_ip);
    } else {
        ESP_LOGW(TAG, "System Running in RESCUE Mode.");
        ESP_LOGW(TAG, "Please connect to WiFi 'ESP32-Rescue-Mode' and visit http://192.168.4.1");
    }
}