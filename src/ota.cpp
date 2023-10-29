#include "ota.h"

const char * logtag_ota = "OTA";


/**
 * @brief init ota
 * call once
 * @param ssid 
 * @param pw 
 * @return int 1 error, 0 success
 */

int init_ota(const char *ssid, const char *pw)
{
    ESP_LOGI(logtag_ota, "Init OTA");
    ESP_LOGD(logtag_ota, "Set WiFi mode");
    WiFi.mode(WIFI_STA);
    ESP_LOGD(logtag_ota, "Begin WiFi");
    WiFi.begin(ssid, pw);
    unsigned long old_time = millis();
    unsigned long new_time = millis();
    while (WiFi.waitForConnectResult() != WL_CONNECTED && new_time - old_time < TIME_WAITING_FOR_CONNECTION_IN_MS)
    {
        ESP_LOGD(logtag_ota, "Waiting for connection.");
        new_time = millis();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        ESP_LOGE(logtag_ota, "Connection Failed!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(logtag_ota, "Rebooting");
        esp_restart();
    }
    else
        ESP_LOGD(logtag_ota, "Connection established!");

    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    // ArduinoOTA.setHostname("myesp32");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            ESP_LOGI(logtag_ota, "Start updating %s", type);
        })
        .onEnd([]() {
            ESP_LOGI(logtag_ota, "\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            ESP_LOGI(logtag_ota, "Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            ESP_LOGE(logtag_ota, "Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                ESP_LOGE(logtag_ota, "Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                ESP_LOGE(logtag_ota, "Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                ESP_LOGE(logtag_ota, "Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                ESP_LOGE(logtag_ota, "Receive Failed");
            else if (error == OTA_END_ERROR)
                ESP_LOGE(logtag_ota, "End Failed");
        });
    ESP_LOGD(logtag_ota, "Begin OTA");
    ArduinoOTA.begin();

    ESP_LOGI(logtag_ota, "Ready");
    ESP_LOGI(logtag_ota, "IP address: %ui", WiFi.localIP());
    return 0;
}

/**
 * @brief handle ota
 * call in loop 
 */
void handle_ota()
{
    ArduinoOTA.handle();
}