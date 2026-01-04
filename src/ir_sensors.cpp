#include "ir_sensors.h"

#include "IR32/src/IRRecv.h"
#include "soc/rtc_wdt.h"
#include <stdexcept>

// Keep all IR sensor hardware details and FreeRTOS wiring here.

#ifndef IR_PROTOCOL
#define IR_PROTOCOL "NEC"
#endif

// RMT channels
#define IR_RMT_RX_FRONT_CHANNEL RMT_CHANNEL_0
#define IR_RMT_RX_BACK_CHANNEL RMT_CHANNEL_1

// GPIO pins
#define IR_RECV_FRONT_PIN 27
#define IR_RECV_BACK_1_PIN 26

// Debounce (ms): only accepted hits advance this timer
#define IR_DEBOUNCE_TIME_MS 1000

// IRRecv instances
static IRRecv ir_rec_front(IR_RMT_RX_FRONT_CHANNEL);
static IRRecv ir_rec_back(IR_RMT_RX_BACK_CHANNEL);

// Internal plumbing
static QueueHandle_t irEventQueue = nullptr;
static SemaphoreHandle_t irProcessingSemaphore = nullptr;
static volatile unsigned long lastIRAcceptedTime = 0;
static IrSensorsOnHitCallback onHitCallback = nullptr;

static TaskHandle_t xHandle_handleIRFront = nullptr;
static TaskHandle_t xHandle_handleIRBack = nullptr;
static TaskHandle_t xHandle_processIRQueue = nullptr;

// Structure to pass IR sensor configuration to generic handler
struct IRSensorConfig {
	IRRecv *receiver;
	uint8_t pin;
	uint8_t sensorId;
	const char *sensorName;
};

static void processIRQueue(void *parameter)
{
	(void)parameter;
	Serial.println("IR processing task started");

	IREvent event;
	while (true) {
		if (xQueueReceive(irEventQueue, &event, portMAX_DELAY)) {
			if (xSemaphoreTake(irProcessingSemaphore, pdMS_TO_TICKS(100))) {
				unsigned long currentTime = millis();
				bool isDebounced = (currentTime - lastIRAcceptedTime >= IR_DEBOUNCE_TIME_MS);
				if (isDebounced) {
					Serial.printf("Processing IR from sensor %d: %s/0x%x\n", event.sensor,
							(event.rcvGroup ? event.rcvGroup : "?"), event.result);

					bool accepted = false;
					if (onHitCallback != nullptr) {
						accepted = onHitCallback(&event);
					}

					if (accepted) {
						lastIRAcceptedTime = currentTime;
					}
				} else {
					Serial.printf(">>> IR signal dropped (debounce): sensor %d, time since last accepted: %lu ms\n",
							event.sensor, currentTime - lastIRAcceptedTime);
				}

				xSemaphoreGive(irProcessingSemaphore);
			}
		}
	}
}

static void handleIRSensor(void *parameter)
{
	IRSensorConfig *config = (IRSensorConfig *)parameter;

	Serial.printf("handle IR %s task started\n", config->sensorName);
	Serial.printf("init ir rec %s\n", config->sensorName);
	config->receiver->setPreferred(IR_PROTOCOL);
	config->receiver->start(config->pin);
	Serial.printf("init ir rec %s complete\n", config->sensorName);

	uint32_t lastFeed = millis();

	while (true) {
		try {
			if ((millis() - lastFeed) > 1000) {
				rtc_wdt_feed();
				vTaskDelay(50);
				lastFeed = millis();
			}

			if (config->receiver->available()) {
				Serial.printf("IR %s available\n", config->sensorName);
				char *rcvGroup = nullptr;
				uint32_t result = config->receiver->read(rcvGroup);

				if (result) {
					IREvent event;
					event.result = result;
					event.rcvGroup = rcvGroup;
					event.sensor = config->sensorId;
					event.timestamp = millis();

					if (xQueueSend(irEventQueue, &event, 0) != pdTRUE) {
						Serial.printf(">>> IR queue full, dropping %s sensor event\n", config->sensorName);
					}
				}

				vTaskDelay(200);
			}
		} catch (const std::invalid_argument &) {
			Serial.printf("Error occurred in %s IR handler\n", config->sensorName);
		}
	}
}

static void handleIRFront(void *parameter)
{
	(void)parameter;
	static IRSensorConfig frontConfig = {
		&ir_rec_front,
		IR_RECV_FRONT_PIN,
		0,
		"Front",
	};

	handleIRSensor(&frontConfig);
}

static void handleIRBack(void *parameter)
{
	(void)parameter;
	static IRSensorConfig backConfig = {
		&ir_rec_back,
		IR_RECV_BACK_1_PIN,
		1,
		"Back",
	};

	handleIRSensor(&backConfig);
}

void ir_sensors_begin(IrSensorsOnHitCallback onHit)
{
	onHitCallback = onHit;

	irEventQueue = xQueueCreate(10, sizeof(IREvent));
	irProcessingSemaphore = xSemaphoreCreateMutex();

	if (irEventQueue == nullptr || irProcessingSemaphore == nullptr) {
		Serial.println("Failed to create IR queue or semaphore!");
		return;
	}

	xTaskCreatePinnedToCore(
		processIRQueue,
		"processIRQueue",
		4096,
		nullptr,
		3,
		&xHandle_processIRQueue,
		0);

	xTaskCreatePinnedToCore(
		handleIRFront,
		"handleIRFront",
		4096,
		nullptr,
		2,
		&xHandle_handleIRFront,
		0);

	xTaskCreatePinnedToCore(
		handleIRBack,
		"handleIRBack",
		4096,
		nullptr,
		2,
		&xHandle_handleIRBack,
		1);
}
