#include "application.hpp"

void setup()
{
  Serial.begin(115200);
  // GSM_UART.begin(115200);
  GSM_UART.begin(115200, SERIAL_8N1, 32, 14);

  // if (!util.init())
    DEBUG_e("Failed uitl init");

  if (xTaskCreate(dev_core_task, "dev_core_task", 1024 * 5, NULL, 2, NULL) != pdTRUE)
  DEBUG_e("Failed: dev_core_task ");

  // vTaskDelay(2000);
  //  if (xTaskCreate(CAN_TASK, "can_task", 1024 * 5, NULL, 3, NULL) != pdTRUE)
  //  DEBUG_e("Failed: can_task ");
}

void loop()
{
  // while (Serial.available())
  //   GSM_UART.write(Serial.read());
  // while (GSM_UART.available())
  //   Serial.write(GSM_UART.read());
  // vTaskDelay(pdMS_TO_TICKS(100));

  vTaskDelete(NULL);
}
