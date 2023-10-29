/**
 * @file ir_and_trigger.cpp
 * @author gldhnchn (gldhnchn@posteo.de)
 * @brief infrared stuff and trigger handler
 * @date 2019-01-23
 *
 */

#include "ir_and_trigger.h"

const  char * logtag_ir = "IR";

/**
 * @brief handle incoming infrared
 * This function reads the incoming infrared data and sends it via bluetooth.
 * @param parameter just a dummy parameter
 */
void handle_ir(void *parameter)
{
    ESP_LOGD(logtag_ir, "handle IR task started");
    ESP_LOGD(logtag_ir, "init ir recv front");
    ir_recv_front.setPreferred(IR_PROTOCOL);
    ir_recv_front.start(IR_RECV_FRONT_PIN);
    while (true)
    {
        if (ir_recv_front.available())
        {
            ESP_LOGD(logtag_ir, "IR available");
            char *rcvGroup;
            uint32_t result = ir_recv_front.read(rcvGroup);
            if (result)
            {
                ESP_LOGD(logtag_ir, "Received: %s/0x%x", rcvGroup, result);
                if (device_connected)
                {
                    ir_receive_char->setValue(result);
                    ble_notify(ir_receive_char);
                }
                else if (result != ir_msg[team])
                {
                    ESP_LOGI(logtag_ir, "message from other team");
                    vTaskResume(xHandle_handle_player_status);
                }
                else if (result == ir_msg[team])
                {
                    ESP_LOGI(logtag_ir, "message from own team, doing nothing");
                }
            }
        }
    }
    return;
}

void handle_trigger()
{
    last_bounce_time = xTaskGetTickCount();
    portENTER_CRITICAL_ISR(&mux);
    count_trigger_interrupts++;
    portEXIT_CRITICAL_ISR(&mux);
    vTaskResume(xHandle_refresh_trigger_status);
    return;
}

void refresh_trigger_status(void *parameter)
{
    const char *logtag_ir = "trigger";

    ESP_LOGD(logtag_ir, "refresh trigger status task started");
    while (true)
    {
        vTaskSuspend(NULL); //suspend task until reactivated by handle_trigger()
        //wait until the last bounce is longer ago than DEBOUNCETIME
        while (xTaskGetTickCount() - last_bounce_time < DEBOUNCETIME)
            vTaskDelay(10 / portTICK_PERIOD_MS);
        //refresh trigger.pressed
        trigger.read_pin();
        ESP_LOGI(logtag_ir, "Button Interrupt Triggered times: %u", count_trigger_interrupts);
        ESP_LOGI(logtag_ir, "time in ms since last trigger: %u", xTaskGetTickCount() - last_bounce_time);
        ESP_LOGI(logtag_ir, "trigger status: %u", trigger.pressed);
        last_time_button_pressed_timestamp = millis();
        //Three different tagger situations are handled here. The place for doing that feels a bit weird to me, so it maybe should be refacored.
        //1: Tagger is connected via BT
        if (device_connected)
        {
            ESP_LOGD(logtag_ir, "sending trigger status via bt");
            trigger_char->setValue((int &)trigger.pressed);
            ble_notify(trigger_char);
        }
        //2: Team selection mode
        else if (team_selection && trigger.pressed)
        {
            team++;
            if (team >= 7)
                team = 0;
            ESP_LOGI(logtag_ir, "increasing team. Team: %u", team);
            leds[LED_INDEX_TEAM].setColorCode(color_team[team]);
            FastLED.show();
        }
        //3: BT-less playing mode
        else if (trigger.pressed && player_is_on)
        {
            leds[LED_INDEX_SHOOT].setColorCode(0xFFFFFF);
            FastLED.show();
            ESP_LOGI(logtag_ir, "Device not connected. Sending team message via IR: 0x%x", ir_msg[team]);
            ir_led.send(ir_msg[team]);
            leds[LED_INDEX_SHOOT].setColorCode(0);
            FastLED.show();
        }
        portENTER_CRITICAL_ISR(&mux);
        count_trigger_interrupts = 0;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

void handle_player_status(void *parameter)
{
    ESP_LOGD(logtag_ir, "handle player status task started");
    while (true)
    {
        ESP_LOGI(logtag_ir, "setting player status to active");
        player_is_on = true;
        leds[LED_INDEX_PLAYER_STATUS].setColorCode(COLOR_PLAYER_STATUS_ON);
        FastLED.show();
        vTaskSuspend(NULL); //suspend task until reactivated by handle_ir()
        ESP_LOGI(logtag_ir, "setting player status to down");
        player_is_on = false;
        leds[LED_INDEX_PLAYER_STATUS].setColorCode(COLOR_PLAYER_STATUS_OFF);
        FastLED.show();
        vTaskDelay(PLAYER_DOWNTIME_IN_MS / portTICK_PERIOD_MS);
    }
    return;
}