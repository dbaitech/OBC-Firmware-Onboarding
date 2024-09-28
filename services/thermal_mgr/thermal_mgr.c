#include "thermal_mgr.h"
#include "errors.h"
#include "lm75bd.h"
#include "console.h"
#include "logging.h"

#include <FreeRTOS.h>
#include <os_task.h>
#include <os_queue.h>

#include <string.h>

#define THERMAL_MGR_STACK_SIZE 256U

static TaskHandle_t thermalMgrTaskHandle;
static StaticTask_t thermalMgrTaskBuffer;
static StackType_t thermalMgrTaskStack[THERMAL_MGR_STACK_SIZE];

#define THERMAL_MGR_QUEUE_LENGTH 10U
#define THERMAL_MGR_QUEUE_ITEM_SIZE sizeof(thermal_mgr_event_t)

static QueueHandle_t thermalMgrQueueHandle;
static StaticQueue_t thermalMgrQueueBuffer;
static uint8_t thermalMgrQueueStorageArea[THERMAL_MGR_QUEUE_LENGTH * THERMAL_MGR_QUEUE_ITEM_SIZE];

static void thermalMgr(void *pvParameters);

void initThermalSystemManager(lm75bd_config_t *config) {
  memset(&thermalMgrTaskBuffer, 0, sizeof(thermalMgrTaskBuffer));
  memset(thermalMgrTaskStack, 0, sizeof(thermalMgrTaskStack));
  
  thermalMgrTaskHandle = xTaskCreateStatic(
    thermalMgr, "thermalMgr", THERMAL_MGR_STACK_SIZE,
    config, 1, thermalMgrTaskStack, &thermalMgrTaskBuffer);

  memset(&thermalMgrQueueBuffer, 0, sizeof(thermalMgrQueueBuffer));
  memset(thermalMgrQueueStorageArea, 0, sizeof(thermalMgrQueueStorageArea));

  thermalMgrQueueHandle = xQueueCreateStatic(
    THERMAL_MGR_QUEUE_LENGTH, THERMAL_MGR_QUEUE_ITEM_SIZE,
    thermalMgrQueueStorageArea, &thermalMgrQueueBuffer);

}

error_code_t thermalMgrSendEvent(thermal_mgr_event_t *event) {
  /* Send an event to the thermal manager queue */
  if (thermalMgrQueueHandle == NULL) {
    return ERR_CODE_INVALID_STATE;
  }

  if (event == NULL) {
    return ERR_CODE_INVALID_ARG;
  }

  if (xQueueSend(thermalMgrQueueHandle, event, 0) != pdTRUE) {
    return ERR_CODE_QUEUE_FULL;
  }
  return ERR_CODE_SUCCESS;
}

void osHandlerLM75BD(void) {
  /* Implement this function */
  thermal_mgr_event_t event = { .type =  THERMAL_MGR_EVENT_OS_INTERRUPT };
  error_code_t errCode = thermalMgrSendEvent(&event);
  LOG_IF_ERROR_CODE(errCode);
}

static void thermalMgr(void *pvParameters) {
  /* Implement this task */
  if (pvParameters == NULL) {
    LOG_ERROR_CODE(ERR_CODE_INVALID_ARG);
    while(1);
  }

  lm75bd_config_t config = *(lm75bd_config_t *) pvParameters;  // obtaining lm75db config

  while (1) {
	thermal_mgr_event_t event;
    if (xQueueReceive(thermalMgrQueueHandle, &event, portMAX_DELAY) == pdTRUE) {
      if ((event.type == THERMAL_MGR_EVENT_MEASURE_TEMP_CMD) ||
          (event.type == THERMAL_MGR_EVENT_OS_INTERRUPT)) {

        float temperature;
        error_code_t errCode = readTempLM75BD(config.devAddr, &temperature);  // measuring the current temperature

        if (errCode != ERR_CODE_SUCCESS) {
          LOG_ERROR_CODE(errCode);
          continue;
        }

        addTemperatureTelemetry(temperature);  // send the temperature

        if (event.type == THERMAL_MGR_EVENT_OS_INTERRUPT) {
          if (temperature > config.hysteresisThresholdCelsius) {
            overTemperatureDetected();
          } else {
            safeOperatingConditions();
          }
        }
      } else {
        LOG_ERROR_CODE(ERR_CODE_INVALID_QUEUE_MSG);
      }
    } else {
      LOG_ERROR_CODE(ERR_CODE_FAILED_QUEUE_OPERATION);
    }
  }
}

void addTemperatureTelemetry(float tempC) {
  printConsole("Temperature telemetry: %f deg C\n", tempC);
}

void overTemperatureDetected(void) {
  printConsole("Over temperature detected!\n");
}

void safeOperatingConditions(void) { 
  printConsole("Returned to safe operating conditions!\n");
}
