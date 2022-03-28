#include "application.hpp"
TaskHandle_t gsmTask_h;
bool echomode = 1;
char ch;
int cmd;




void setup()
{
  Serial.begin(115200);
  // Serial2.begin(115200);
  GSM_UART.begin(115200, SERIAL_8N1, 32, 14);

  // if (xTaskCreate(dev_core_task, "dev_core_task", 1024 * 5, NULL, 2, NULL) != pdTRUE)
  DEBUG_e("Failed: dev_core_task ");

  // vTaskDelay(2000);
  // if (xTaskCreate(CAN_TASK, "can_task", 1024 * 5, NULL, 3, NULL) != pdTRUE)
  // debug_e("Failed: can_task ");
}

void loop()
{
  while (Serial.available())
    GSM_UART.write(Serial.read());
  while (GSM_UART.available())
    Serial.write(GSM_UART.read());
  vTaskDelay(pdMS_TO_TICKS(100));
  

  /*if (Serial.available())
  {
    ch = Serial.read();

    switch (ch)
    {
    case '@':
      if ((cmd = Serial.parseInt()))
      {
        switch (cmd)
        {
        case 2:
          DEBUG_i("Starting GMS Task");
          xTaskCreate(dev_core_task, "dev_core_task", 1024 * 5, NULL, 2, &gsmTask_h);
          echomode = false;
          break;
        case 3:
          DEBUG_i("Stoping GSM Task");
          vTaskDelete(gsmTask_h);
          echomode = true;
          break;

        // case 4:
        //   STATUS_CHECK(gsm.begin(GSM_UART))
        //   break;
        // case 5:
        //   STATUS_CHECK(gsm.connect("165.232.184.128", 8900))
        //   DEBUG_d("{tcp connected}");
        //   break;
        // case 6:
        //   STATUS_CHECK(gsm.print("$FXLPGO001-LIN,AUTOPEEPAL,MH12GR2515,867322034553800,0.0.1,AIS140,00.0000,N,00.0000,N*50"))
        //   DEBUG_d("{packet send}");
        //   break;
        // case 7:
        //   Serial.print("[GSM: ");
        //   while (gsm.available())
        //   {
        //     Serial.write(gsm.read());
        //   }
        //   Serial.println("]");
        //   break;

        default:
          DEBUG_e("INVALID CMD");
          break;
        }
      }
      break;

    case '*':
      vTaskDelay(pdMS_TO_TICKS(100));
      while (Serial.available())
        GSM_UART.write(Serial.read());
      break;

    default:
      DEBUG_e("INVALID OP");
      break;
    }
  }

  if (echomode)
  {
    while (GSM_UART.available())
      Serial.write(GSM_UART.read());
  }

  vTaskDelay(pdMS_TO_TICKS(100));*/

  // vTaskDelete(NULL);
}
