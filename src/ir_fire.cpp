#include "ir_fire.h"

#include "IR32/src/IRSend.h"
#include <stdexcept>

#ifndef IR_PROTOCOL
#define IR_PROTOCOL "NEC"
#endif

#define IR_RMT_TX_CHANNEL RMT_CHANNEL_2
#define IR_RMT_TX_GPIO_NUM GPIO_NUM_2

static IRSend ir_send(IR_RMT_TX_CHANNEL);

static TaskHandle_t xHandle_fire = nullptr;
static volatile uint32_t pendingShotCode = 0;

static IrFireCanFireCallback canFireCallback = nullptr;
static IrFireDidFireCallback didFireCallback = nullptr;

static void handleFire(void *parameter)
{
	(void)parameter;
	ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);

	while (true) {
		vTaskSuspend(NULL);

		uint32_t code = pendingShotCode;
		if (code == 0) {
			continue;
		}

		try {
			bool allowed = true;
			if (canFireCallback != nullptr) {
				allowed = canFireCallback();
			}

			if (allowed) {
				Serial.println(">>> Shot!");
				ir_send.send(code);
				if (didFireCallback != nullptr) {
					didFireCallback(code);
				}
				vTaskDelay(500);
			} else {
				Serial.println(">>> Shot cooldown");
			}
		} catch (const std::invalid_argument &e) {
			Serial.print("Error occurred: ");
			Serial.println(e.what());
			ir_send.stop();
			ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);
		}
	}
}

void ir_fire_begin(IrFireCanFireCallback canFire, IrFireDidFireCallback didFire)
{
	canFireCallback = canFire;
	didFireCallback = didFire;

	xTaskCreatePinnedToCore(
		handleFire,
		"handle fire",
		1024,
		nullptr,
		3,
		&xHandle_fire,
		1);
}

void ir_fire_request(uint32_t code)
{
	pendingShotCode = code;
	if (xHandle_fire != nullptr) {
		vTaskResume(xHandle_fire);
	}
}
