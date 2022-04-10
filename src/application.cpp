#include "application.hpp"
#include "GSM.hpp"
#include "app_ota.hpp"

uint8_t RDG_RXBuff[2000];
char RdgRsp[2000];
uint8_t RDG_FR_CNT = 0, rdgTmpCnt = 0;

AIS140 ais;
WiFiClient wClient;
WiFiServer SocketServer(5252);
QuectelCellular telemSoc;
TrackingPacket trackPack;
HealthPacket healthPack;
EmrgncyPacket EmrPack;
uint8_t DRxBuff[1500];

void dev_core_task(void *args)
{
    ais.Init();

    for (;;)
    {
        ais.DataProcess();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    log_e("telemetics_server_gsm_task DELELE");
    vTaskDelete(NULL);
}

void wifiHandlerTask(void *args)
{
    SocketServer.begin();

    while (1)
    {
        wClient = SocketServer.available();

        if (wClient.connected())
        {
            while (wClient.connected())
            {

                if (wClient.available())
                {
                    ais.SrvData = (char)wClient.read();

                    if (ais.SrvData[0] == '*')
                    {
                        while (wClient.available())
                            ais.SrvData += (char)wClient.read();
                        ais.ParseCommands(ais.SrvData, COM_WIFI);
                    }
                    else
                    {
                        log_w("invalid cmd - %u", wClient.available());
                        int x;
                        while ((x = wClient.available()) > 0)
                        {
                            while (x--)
                                Serial.print(((char)wClient.read()));
                        }
                        Serial.println();
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

/************************ PUBLIC METHODS ************************/
void AIS140::Init()
{

    devConfigInit();
    DEBUG_i("[DEV MODE: %u]", Access);

    if ((Access == USR))
    {
        userModeinit();
    }
    else if ((Access == VMEOL) || (Access == DLR))
    {
        if (wlan)
        {
            dataStruct.Mem.begin(APP_CNF, true);
            Serial.printf("SSID <%s> PSWD <%s>\r\n", dataStruct.Mem.getString("SSID").c_str(), dataStruct.Mem.getString("PSWD").c_str());

            WiFi.begin(dataStruct.Mem.getString("SSID").c_str(), dataStruct.Mem.getString("PSWD").c_str());
            WiFi.waitForConnectResult();
            Serial.println(WiFi.localIP());
            dataStruct.Mem.end();

            if (MDNS.begin("fotax") == true)
                MDNS.addService("_http", "_tcp", 80);

            xTaskCreate(wifiHandlerTask, "wifiHandlerTask", 1024 * 5, NULL, 2, NULL);
        }
        else
        {
            DEBUG_e("[WIFI Creds not Saved]");
        }
    }
}

void AIS140::DataProcess(void)
{
    if ((Access == USR))
    {
        userModeDataProcess(srv_telematics);
        if (telemSoc.connected(srv_telematics))
        {
            periodicTransmit();
        }
        else
        {
            srvConnectionHandler(srv_telematics);
        }

        if (Emrgncy && IsTimerElapsed(EmrPackTmr))
        {
            ResetTimer(EmrPackTmr, 5000);
            if (telemSoc.connected(srv_emergency))
            {
                if (!telemSoc.print(EmrPack.Packetize(), srv_emergency))
                {
                    if (EmrPack.pushFrame())
                    {
                        dataStruct.AISData.Mem2Cnt++;
                        DEBUG_w("[EMR] [pushFrame]");
                    }
                }

                if (EmrPack.avaiableFrame())
                {
                    if (!telemSoc.print(EmrPack.popFrame(), srv_telematics))
                        DEBUG_w("[EMR] [Send]");
                }
            }
            else
            {
                srvConnectionHandler(srv_emergency);
            }
        }
    }

    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '*')
        {
            SrvData = c;
            do
            {
                c = (char)Serial.read();
                SrvData += c;
            } while (Serial.available() && c != '#');
            ParseCommands(SrvData, COM_USB);
        }
        else if (c == '{')
        {
            SrvData.clear();
            do
            {
                c = (char)Serial.read();
                IF(c == '}', break;)
                SrvData += c;
            } while (Serial.available());
            remoteDiag(SrvData, COM_USB);
        }
        else if (c == '<')
        {
            SrvData = c;
            do
            {
                c = (char)Serial.read();
                SrvData += c;
            } while (Serial.available() && c != '>');
            AdminCmdParse(SrvData, COM_USB);
        }
        else
        {
            DEBUG_w("invalid Frame: [", Serial.available());
            int x;
            while ((x = Serial.available()) > 0)
            {
                while (x--)
                    Serial.print((char)Serial.read());
            }
            Serial.println("]");
        }
    }
}

void AIS140::ParseCommands(String &str, int comCh)
{
    char *ptr;
    char buff[512];
    char Data[512];
    uint8_t FieldCnt = 0;
#define _PTR_(x) PTR_CHECK_GOTO(x, _EXET)

    sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_33);

    if (str.indexOf("SEM") > 0)
    {
        Emrgncy = false;
        StopTimer(EmrPackTmr);
        DEBUG_i("[SOS STOP]");
        telemSoc.disconnect(srv_emergency);
        vTaskDelay(pdMS_TO_TICKS(1000));
        sprintf(buff, "#SEM#OK#");
        SendResp(buff, comCh);
        return;
    }
    else if ((str.indexOf("GETSEED") > 0))
    {
        str.toCharArray(Data, str.length());
        ptr = strtok(strstr(Data, "GETSEED"), ",");
        ptr = strtok(NULL, ",");
        if (strstr(ptr, "APEOL") && (Access == APEOL))
        {
            for (int i = 0; i < 16; i++)
                seedKey[i] = (uint8_t)rand();

            sprintf(buff, "#GETSEED#APEOL#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, seedKey[i]);
            sprintf(buff, "%s#", buff);

            AES_init_ctx(&ctx, dataStruct.DEVData.APK);
            AES_ECB_encrypt(&ctx, seedKey);

            seedKeyStr[0] = 0;
            for (int i = 0; i < 16; i++)
                sprintf(seedKeyStr, "%s%02x", seedKeyStr, seedKey[i]);
        }
        else if (strstr(ptr, "VMEOL") && (Access == VMEOL))
        {
            for (int i = 0; i < 16; i++)
                seedKey[i] = (uint8_t)rand();

            sprintf(buff, "#GETSEED#VMEOL#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, seedKey[i]);
            sprintf(buff, "%s#", buff);

            AES_init_ctx(&ctx, dataStruct.DEVData.VMEK);
            AES_ECB_encrypt(&ctx, seedKey);

            seedKeyStr[0] = 0;
            for (int i = 0; i < 16; i++)
                sprintf(seedKeyStr, "%s%02x", seedKeyStr, seedKey[i]);

            // Serial.println(seedKeyStr);
        }
        else if (strstr(ptr, "DLR") && (Access == DLR))
        {
            for (int i = 0; i < 16; i++)
                seedKey[i] = (uint8_t)rand();

            sprintf(buff, "#GETSEED#DLR#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, seedKey[i]);
            sprintf(buff, "%s#", buff);

            AES_init_ctx(&ctx, dataStruct.DEVData.DCSK);
            AES_ECB_encrypt(&ctx, seedKey);

            seedKeyStr[0] = 0;
            for (int i = 0; i < 16; i++)
                sprintf(seedKeyStr, "%s%02x", seedKeyStr, seedKey[i]);

            // Serial.println(seedKeyStr);
        }
        else if (strstr(ptr, "USR") && (Access == USR))
        {
            for (int i = 0; i < 16; i++)
                seedKey[i] = (uint8_t)rand();

            sprintf(buff, "#GETSEED#USR#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, seedKey[i]);
            sprintf(buff, "%s#", buff);

            AES_init_ctx(&ctx, dataStruct.DEVData.NSK);
            AES_ECB_encrypt(&ctx, seedKey);

            seedKeyStr[0] = 0;
            for (int i = 0; i < 16; i++)
                sprintf(seedKeyStr, "%s%02x", seedKeyStr, seedKey[i]);

            // Serial.println(seedKeyStr);
        }
        else
        {
            sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_13);
        }

        SendResp(buff, comCh);
        return;
    }
    else if (str.indexOf("SENDKEY") > 0)
    {
        str.toCharArray(Data, str.length());
        ptr = strtok(strstr(Data, "SENDKEY"), ",");
        ptr = strtok(NULL, ",");
        if (strstr(ptr, "APEOL") && (Access == APEOL))
        {
            ptr = strtok(NULL, ",");
            if (memcmp(seedKeyStr, ptr, 16) == 0)
            {
                sprintf(buff, "#SENDKEY#APEOL#");
                Access = APEOL;
                Security = true;
            }
            else
            {
                Security = false;
                sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_35);
            }
        }
        else if (strstr(ptr, "VMEOL") && (Access == VMEOL))
        {
            ptr = strtok(NULL, ",");
            if (memcmp(seedKeyStr, ptr, 16) == 0)
            {
                sprintf(buff, "#SENDKEY#VMEOL#");
                Access = VMEOL;
                Security = true;
            }
            else
            {
                Security = false;
                sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_35);
            }
        }
        else if (strstr(ptr, "DLR") && (Access == DLR))
        {
            ptr = strtok(NULL, ",");
            if (memcmp(seedKeyStr, ptr, 16) == 0)
            {
                sprintf(buff, "#SENDKEY#DLR#");
                Access = DLR;
                Security = true;
            }
            else
            {
                Security = false;
                sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_35);
            }
        }
        else if (strstr(ptr, "USR") && (Access == USR))
        {
            ptr = strtok(NULL, ",");
            if (memcmp(seedKeyStr, ptr, 16) == 0)
            {
                sprintf(buff, "#SENDKEY#USR#");
                Access = USR;
                Security = true;
            }
            else
            {
                Security = false;
                sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_35);
            }
        }
        else
        {
            sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_13);
        }
        SendResp(buff, comCh);
        return;
    }
    else if ((str.indexOf("GETMODE") > 0))
    {
        sprintf(buff, "#GETMODE#%d#", Access);
        SendResp(buff, comCh);
        return;
    }

    if (!Security)
    {
        SendResp(buff, comCh);
        return;
    }

    sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_7E);

    if ((str.indexOf("IP1") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if ((str.indexOf("GET") > 0))
        {
            sprintf(buff, "#GET#IP1#%s,%u#", dataStruct.SRVData.IP1, dataStruct.SRVData.IP1P);
        }
        else if ((str.indexOf("SET") > 0))
        {
            sprintf(buff, "#SET#IP1#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "IP1"), ","))
            _PTR_(ptr = strtok(NULL, ","))
            strcpy(dataStruct.SRVData.IP1, ptr);
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.SRVData.IP1P = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("IP1", dataStruct.SRVData.IP1);
            dataStruct.Mem.putUShort("IP1P", dataStruct.SRVData.IP1P);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#IP1#OK#");
        }
        else if ((str.indexOf("CLR") > 0))
        {
            strcpy(dataStruct.SRVData.IP1, "0.0.0.0");
            dataStruct.SRVData.IP1P = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("IP1", dataStruct.SRVData.IP1);
            dataStruct.Mem.putUShort("IP1P", dataStruct.SRVData.IP1P);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#IP1#OK#");
        }
    }
    else if ((str.indexOf("IP2") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if ((str.indexOf("GET") > 0))
        {
            sprintf(buff, "#GET#IP2#%s,%u#", dataStruct.SRVData.IP2, dataStruct.SRVData.IP2P);
        }
        else if ((str.indexOf("SET") > 0))
        {
            sprintf(buff, "#SET#IP2#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "IP2"), ","))
            _PTR_(ptr = strtok(NULL, ","))
            strcpy(dataStruct.SRVData.IP2, ptr);
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.SRVData.IP2P = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("IP2", dataStruct.SRVData.IP2);
            dataStruct.Mem.putUShort("IP2P", dataStruct.SRVData.IP2P);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#IP2#OK#");
        }
        else if ((str.indexOf("CLR") > 0))
        {
            strcpy(dataStruct.SRVData.IP2, "0.0.0.0");
            dataStruct.SRVData.IP2P = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("IP2", dataStruct.SRVData.IP2);
            dataStruct.Mem.putUShort("IP2P", dataStruct.SRVData.IP2P);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#IP2#OK#");
        }
    }
    else if ((str.indexOf("OVSPDLMT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#OVSPDLMT#%u#", dataStruct.AISData.OVSPDLMT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#OVSPDLMT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "OVSPDLMT"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.OVSPDLMT = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("OVSPDLMT", dataStruct.AISData.OVSPDLMT);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#OVSPDLMT#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.OVSPDLMT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("OVSPDLMT", dataStruct.AISData.OVSPDLMT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#OVSPDLMT#OK#");
        }
    }
    else if ((str.indexOf("HRSHBRK") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#HRSHBRK#%u#", dataStruct.AISData.HRSHBRK);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#HRSHBRK#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "HRSHBRK"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.HRSHBRK = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("HRSHBRK", dataStruct.AISData.HRSHBRK);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#HRSHBRK#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.HRSHBRK = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("HRSHBRK", dataStruct.AISData.HRSHBRK);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#HRSHBRK#OK#");
        }
    }
    else if ((str.indexOf("HRSHACL") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#HRSHACL#%u#", dataStruct.AISData.HRSHACL);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#HRSHACL#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "HRSHACL"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.HRSHACL = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("HRSHACL", dataStruct.AISData.HRSHACL);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#HRSHACL#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.HRSHACL = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("HRSHACL", dataStruct.AISData.HRSHACL);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#HRSHACL#OK#");
        }
    }
    else if ((str.indexOf("RSHTURN") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#RSHTURN#%u#", dataStruct.AISData.RSHTURN);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#RSHTURN#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "RSHTURN"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.RSHTURN = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("RSHTURN", dataStruct.AISData.RSHTURN);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#RSHTURN#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.RSHTURN = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("RSHTURN", dataStruct.AISData.RSHTURN);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#RSHTURN#OK#");
        }
    }
    else if ((str.indexOf("SOSNUM1") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#SOSNUM1#%s#", dataStruct.AISData.SOSNUM1);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#SOSNUM1#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "SOSNUM1"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            strcpy(dataStruct.AISData.SOSNUM1, ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("SOSNUM1", dataStruct.AISData.SOSNUM1);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#SOSNUM1#OK#");
        }
    }
    else if ((str.indexOf("VEHREGNUM") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#VEHREGNUM#%s#", dataStruct.DEVData.VEHREGNUM);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#VEHREGNUM#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "VEHREGNUM"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.HRSHACL = atoi(ptr);
            strcpy(dataStruct.DEVData.VEHREGNUM, ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("VEHREGNUM", dataStruct.DEVData.VEHREGNUM);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#VEHREGNUM#OK#");
        }
    }
    else if ((str.indexOf("IGNINT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if ((str.indexOf("GET") > 0))
        {
            sprintf(buff, "#GET#IGNINT#%u#", dataStruct.AISData.IGNINT);
        }
        else if ((str.indexOf("SET") > 0))
        {
            sprintf(buff, "#SET#IGNINT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "IGNINT"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.IGNINT = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("IGNINT", dataStruct.AISData.IGNINT);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#IGNINT#OK#");
        }
        else if ((str.indexOf("CLR") > 0))
        {
            dataStruct.AISData.IGNINT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("IGNINT", dataStruct.AISData.IGNINT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#IGNINT#OK#");
        }
    }
    else if ((str.indexOf("IGFINT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#Get#IGFINT#%u#", dataStruct.AISData.IGFINT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#IGFINT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "IGFINT"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.IGFINT = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("IGFINT", dataStruct.AISData.IGFINT);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#IGFINT#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.IGFINT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("IGFINT", dataStruct.AISData.IGFINT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#IGFINT#OK#");
        }
    }
    else if ((str.indexOf("PANINT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#PANINT#%u#", dataStruct.AISData.PANINT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#PANINT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "PANINT"), ","))
            _PTR_(ptr = strtok(NULL, "#"))
            dataStruct.AISData.PANINT = atoi(ptr);
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("PANINT", dataStruct.AISData.PANINT);
            dataStruct.Mem.end();
            sprintf(buff, "#SET#PANINT#OK#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.PANINT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("PANINT", dataStruct.AISData.PANINT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#PANINT#OK#");
        }
    }
    else if (str.indexOf("IMEI") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#IMEI#%s#", dataStruct.GSMData.IMEI);
        }
    }
    else if ((str.indexOf("HLTINT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#HLTINT#%u#", dataStruct.AISData.HLTINT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#HLTINT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "HLTINT"), ""))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                dataStruct.AISData.HLTINT = atoi(ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putUShort("HLTINT", dataStruct.AISData.HLTINT);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#HLTINT#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.HLTINT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("HLTINT", dataStruct.AISData.HLTINT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#HLTINT#OK#");
        }
    }
    else if ((str.indexOf("HLTPKT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#HLTPKT#%s#", healthPack.Packetize());
        }
    }
    else if (str.indexOf("GSMSIGNAL") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#GSMSIGNAL#%u#", dataStruct.GSMData.GSMSIGNAL);
        }
    }
    else if ((str.indexOf("SLPTM") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#SLPTM#%u#", dataStruct.AISData.SLPTM);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#SLPTM#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "SLPTM"), ""))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                dataStruct.AISData.SLPTM = atoi(ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putUShort("SLPTM", dataStruct.AISData.SLPTM);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#SLPTM#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.SLPTM = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("SLPTM", dataStruct.AISData.SLPTM);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#SLPTM#OK#");
        }
    }
    else if ((str.indexOf("BATT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#BATT#%u#", dataStruct.AISData.BATALT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#BATT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "BATT"), ""))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                dataStruct.AISData.BATALT = atoi(ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putUShort("BATALT", dataStruct.AISData.BATALT);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#BATT#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.AISData.BATALT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUShort("BATALT", (uint16_t)dataStruct.AISData.BATALT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#BATT#OK#");
        }
    }
    else if (str.indexOf("RESET") > 0)
    {
        sprintf(buff, "#RESET#OK#");
        SendResp(buff, comCh);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    else if (str.indexOf("FRMWR") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#FRMWR#%s#", dataStruct.DEVData.FWVER);
        }
    }
    else if (str.indexOf("OTA") > 0)
    {
        sprintf(buff, "#OTA#FAILED#");
        str.toCharArray(Data, str.length());
        _PTR_(ptr = strtok(strstr(Data, "OTA"), ""))
        if ((ptr = strtok(NULL, "#")) != NULL)
        {
            // debug_d("OTA: %s", ptr);
            xTaskCreate(ota_task, "ota_task", 5 * 1024, (void *)ptr, 5, NULL);
            sprintf(buff, "#OTA#OK#");
        }
    }
    else if (str.indexOf("IVN") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            if (str.indexOf("IVNINT") > 0)
            {
                sprintf(buff, "#GET#IVNINT#%u,%u,%u,%u,%u#", dataStruct.DEVData.IVN1INT,
                        dataStruct.DEVData.IVN2INT, dataStruct.DEVData.IVN3INT,
                        dataStruct.DEVData.IVN4INT, dataStruct.DEVData.IVN5INT);
            }
            else if (str.indexOf("IVN1") > 0)
            {
                sprintf(buff, "#GET#IVN1,%x,%x,%x,%x,%x#", IVNs[0].ivnframes[0].FrameID,
                        IVNs[0].ivnframes[1].FrameID, IVNs[0].ivnframes[2].FrameID,
                        IVNs[0].ivnframes[3].FrameID, IVNs[0].ivnframes[4].FrameID);
            }
            else if (str.indexOf("IVN2") > 0)
            {
                sprintf(buff, "#GET#IVN2,%x,%x,%x,%x,%x#", IVNs[1].ivnframes[0].FrameID,
                        IVNs[1].ivnframes[1].FrameID, IVNs[1].ivnframes[2].FrameID,
                        IVNs[1].ivnframes[3].FrameID, IVNs[1].ivnframes[4].FrameID);
            }
            else if (str.indexOf("IVN3") > 0)
            {
                sprintf(buff, "#GET#IVN3,%x,%x,%x,%x,%x#", IVNs[2].ivnframes[0].FrameID,
                        IVNs[2].ivnframes[1].FrameID, IVNs[2].ivnframes[2].FrameID,
                        IVNs[2].ivnframes[3].FrameID, IVNs[2].ivnframes[4].FrameID);
            }
            else if (str.indexOf("IVN4") > 0)
            {
                sprintf(buff, "#GET#IVN4,%x,%x,%x,%x,%x#", IVNs[3].ivnframes[0].FrameID,
                        IVNs[3].ivnframes[1].FrameID, IVNs[3].ivnframes[2].FrameID,
                        IVNs[3].ivnframes[3].FrameID, IVNs[3].ivnframes[4].FrameID);
            }
            else if (str.indexOf("IVN5") > 0)
            {
                sprintf(buff, "#GET#IVN5,%x,%x,%x,%x,%x#", IVNs[4].ivnframes[0].FrameID,
                        IVNs[4].ivnframes[1].FrameID, IVNs[4].ivnframes[2].FrameID,
                        IVNs[4].ivnframes[3].FrameID, IVNs[4].ivnframes[4].FrameID);
            }
        }
        else if (str.indexOf("SET") > 0)
        {
            str.toCharArray(Data, str.length());
            // DEBUG_i("CMD: %s", Data);
            dataStruct.Mem.begin("CMD", false);
            if (str.indexOf("IVNINT") > 0)
            {
                uint8_t cnt = 0;
                _PTR_(ptr = strtok(strstr(Data, "IVNINT"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.baud = atoi(ptr); // DEBUG_i("ivn1: %u", dataStruct.DEVData.baud);
                dataStruct.Mem.putUChar("baud", dataStruct.DEVData.baud);
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.address = atoi(ptr); // DEBUG_i("ivn1: %u", dataStruct.DEVData.address);
                dataStruct.Mem.putUChar("address", dataStruct.DEVData.address);
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.IVN1INT = atoi(ptr); // DEBUG_i("ivn1: %u", dataStruct.DEVData.IVN1INT);
                dataStruct.Mem.putUShort("IVN1INT", dataStruct.DEVData.IVN1INT);
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.IVN2INT = atoi(ptr); // DEBUG_i("ivn2: %u", dataStruct.DEVData.IVN2INT);
                dataStruct.Mem.putUShort("IVN2INT", dataStruct.DEVData.IVN2INT);
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.IVN3INT = atoi(ptr); // DEBUG_i("ivn3: %u", dataStruct.DEVData.IVN3INT);
                dataStruct.Mem.putUShort("IVN3INT", dataStruct.DEVData.IVN3INT);
                _PTR_(ptr = strtok(NULL, ","))
                cnt++;
                dataStruct.DEVData.IVN4INT = atoi(ptr); // DEBUG_i("ivn4: %u", dataStruct.DEVData.IVN4INT);
                dataStruct.Mem.putUShort("IVN4INT", dataStruct.DEVData.IVN4INT);
                _PTR_(ptr = strtok(NULL, "#"))
                cnt++;
                dataStruct.DEVData.IVN5INT = atoi(ptr); // DEBUG_i("ivn5: %u", dataStruct.DEVData.IVN5INT);
                dataStruct.Mem.putUShort("IVN5INT", dataStruct.DEVData.IVN5INT);
                sprintf(buff, (cnt == 7) ? "#SET#IVNINT#OK#" : "#SET#IVNINT#FAIL#");
                StartTimer(ivn1Tmr, dataStruct.DEVData.IVN1INT);
                StartTimer(ivn2Tmr, dataStruct.DEVData.IVN2INT);
                StartTimer(ivn3Tmr, dataStruct.DEVData.IVN3INT);
                StartTimer(ivn4Tmr, dataStruct.DEVData.IVN4INT);
                StartTimer(ivn5Tmr, dataStruct.DEVData.IVN5INT);
                dataStruct.Mem.end();
            }
            else if (str.indexOf("IVN1") > 0)
            {
                _PTR_(ptr = strtok(strstr(Data, "IVN1"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[0].ivnframes[0].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN1[0]", IVNs[0].ivnframes[0].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[0].ivnframes[1].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN1[1]", IVNs[0].ivnframes[1].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[0].ivnframes[2].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN1[2]", IVNs[0].ivnframes[2].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[0].ivnframes[3].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN1[3]", IVNs[0].ivnframes[3].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[0].ivnframes[4].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN1[4]", IVNs[0].ivnframes[4].FrameID);
                }
                sprintf(buff, "#SET#IVN1#OK#");
            }
            else if (str.indexOf("IVN2") > 0)
            {
                _PTR_(ptr = strtok(strstr(Data, "IVN2"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[1].ivnframes[0].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[0]", IVNs[1].ivnframes[0].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[1].ivnframes[1].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[1]", IVNs[1].ivnframes[1].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[1].ivnframes[2].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[2]", IVNs[1].ivnframes[2].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[1].ivnframes[3].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[3]", IVNs[1].ivnframes[3].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[1].ivnframes[4].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[4]", IVNs[1].ivnframes[4].FrameID);
                }
                sprintf(buff, "#SET#IVN2#OK#");
            }
            else if (str.indexOf("IVN3") > 0)
            {
                _PTR_(ptr = strtok(strstr(Data, "IVN3"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[2].ivnframes[0].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN3[0]", IVNs[2].ivnframes[0].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[2].ivnframes[1].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[1]", IVNs[2].ivnframes[1].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[2].ivnframes[2].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[2]", IVNs[2].ivnframes[2].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[2].ivnframes[3].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[3]", IVNs[2].ivnframes[3].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[2].ivnframes[4].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN2[4]", IVNs[2].ivnframes[4].FrameID);
                }
                sprintf(buff, "#SET#IVN3#OK#");
            }
            else if (str.indexOf("IVN4") > 0)
            {
                _PTR_(ptr = strtok(strstr(Data, "IVN4"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[3].ivnframes[0].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN4[0]", IVNs[3].ivnframes[0].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[3].ivnframes[1].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN4[1]", IVNs[3].ivnframes[1].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[3].ivnframes[2].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN4[2]", IVNs[3].ivnframes[2].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[3].ivnframes[3].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN4[3]", IVNs[3].ivnframes[3].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[3].ivnframes[4].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN4[4]", IVNs[3].ivnframes[4].FrameID);
                }
                sprintf(buff, "#SET#IVN4#OK#");
            }
            else if (str.indexOf("IVN5") > 0)
            {
                _PTR_(ptr = strtok(strstr(Data, "IVN5"), ","))
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[4].ivnframes[0].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN5[0]", IVNs[4].ivnframes[0].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[4].ivnframes[1].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN5[1]", IVNs[4].ivnframes[1].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[4].ivnframes[2].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN5[2]", IVNs[4].ivnframes[2].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[4].ivnframes[3].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN5[3]", IVNs[4].ivnframes[3].FrameID);
                }
                _PTR_(ptr = strtok(NULL, ","))
                if (ptr != NULL)
                {
                    IVNs[4].ivnframes[4].FrameID = HexAsciToU32(ptr);
                    dataStruct.Mem.putULong("IVN5[4]", IVNs[4].ivnframes[4].FrameID);
                }
                sprintf(buff, "#SET#IVN5#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.Mem.begin("CMD", false);
            if (str.indexOf("IVNINT") > 0)
            {
                dataStruct.DEVData.IVN1INT = 0;
                dataStruct.DEVData.IVN2INT = 0;
                dataStruct.DEVData.IVN3INT = 0;
                dataStruct.DEVData.IVN4INT = 0;
                dataStruct.DEVData.IVN5INT = 0;
                dataStruct.Mem.putUShort("IVN1INT", dataStruct.DEVData.IVN1INT);
                dataStruct.Mem.putUShort("IVN2INT", dataStruct.DEVData.IVN2INT);
                dataStruct.Mem.putUShort("IVN3INT", dataStruct.DEVData.IVN3INT);
                dataStruct.Mem.putUShort("IVN4INT", dataStruct.DEVData.IVN4INT);
                dataStruct.Mem.putUShort("IVN5INT", dataStruct.DEVData.IVN5INT);
                sprintf(buff, "#CLR#IVNINT#OK#");
            }
            else if (str.indexOf("IVN1") > 0)
            {
                IVNs[0].ivnframes[0].FrameID = 0;
                IVNs[0].ivnframes[1].FrameID = 0;
                IVNs[0].ivnframes[2].FrameID = 0;
                IVNs[0].ivnframes[3].FrameID = 0;
                IVNs[0].ivnframes[4].FrameID = 0;
                dataStruct.Mem.putULong("IVN1[0]", IVNs[0].ivnframes[0].FrameID);
                dataStruct.Mem.putULong("IVN1[1]", IVNs[0].ivnframes[1].FrameID);
                dataStruct.Mem.putULong("IVN1[2]", IVNs[0].ivnframes[2].FrameID);
                dataStruct.Mem.putULong("IVN1[3]", IVNs[0].ivnframes[3].FrameID);
                dataStruct.Mem.putULong("IVN1[4]", IVNs[0].ivnframes[4].FrameID);
                sprintf(buff, "#CLR#IVN1#OK#");
            }
            else if (str.indexOf("IVN2") > 0)
            {
                IVNs[1].ivnframes[0].FrameID = 0;
                IVNs[1].ivnframes[1].FrameID = 0;
                IVNs[1].ivnframes[2].FrameID = 0;
                IVNs[1].ivnframes[3].FrameID = 0;
                IVNs[1].ivnframes[4].FrameID = 0;
                dataStruct.Mem.putULong("IVN2[0]", IVNs[1].ivnframes[0].FrameID);
                dataStruct.Mem.putULong("IVN2[1]", IVNs[1].ivnframes[1].FrameID);
                dataStruct.Mem.putULong("IVN2[2]", IVNs[1].ivnframes[2].FrameID);
                dataStruct.Mem.putULong("IVN2[3]", IVNs[1].ivnframes[3].FrameID);
                dataStruct.Mem.putULong("IVN2[4]", IVNs[1].ivnframes[4].FrameID);
                sprintf(buff, "#CLR#IVN2#OK#");
            }
            else if (str.indexOf("IVN3") > 0)
            {
                IVNs[2].ivnframes[0].FrameID = 0;
                IVNs[2].ivnframes[1].FrameID = 0;
                IVNs[2].ivnframes[2].FrameID = 0;
                IVNs[2].ivnframes[3].FrameID = 0;
                IVNs[2].ivnframes[4].FrameID = 0;
                dataStruct.Mem.putULong("IVN3[0]", IVNs[2].ivnframes[0].FrameID);
                dataStruct.Mem.putULong("IVN3[1]", IVNs[2].ivnframes[1].FrameID);
                dataStruct.Mem.putULong("IVN3[2]", IVNs[2].ivnframes[2].FrameID);
                dataStruct.Mem.putULong("IVN3[3]", IVNs[2].ivnframes[3].FrameID);
                dataStruct.Mem.putULong("IVN3[4]", IVNs[2].ivnframes[4].FrameID);
                sprintf(buff, "#CLR#IVN3#OK#");
            }
            else if (str.indexOf("IVN4") > 0)
            {
                IVNs[3].ivnframes[0].FrameID = 0;
                IVNs[3].ivnframes[1].FrameID = 0;
                IVNs[3].ivnframes[2].FrameID = 0;
                IVNs[3].ivnframes[3].FrameID = 0;
                IVNs[3].ivnframes[4].FrameID = 0;
                dataStruct.Mem.putULong("IVN4[0]", IVNs[3].ivnframes[0].FrameID);
                dataStruct.Mem.putULong("IVN4[1]", IVNs[3].ivnframes[1].FrameID);
                dataStruct.Mem.putULong("IVN4[2]", IVNs[3].ivnframes[2].FrameID);
                dataStruct.Mem.putULong("IVN4[3]", IVNs[3].ivnframes[3].FrameID);
                dataStruct.Mem.putULong("IVN4[4]", IVNs[3].ivnframes[4].FrameID);
                sprintf(buff, "#CLR#IVN4#OK#");
            }
            else if (str.indexOf("IVN5") > 0)
            {
                IVNs[4].ivnframes[0].FrameID = 0;
                IVNs[4].ivnframes[1].FrameID = 0;
                IVNs[4].ivnframes[2].FrameID = 0;
                IVNs[4].ivnframes[3].FrameID = 0;
                IVNs[4].ivnframes[4].FrameID = 0;
                dataStruct.Mem.putULong("IVN5[0]", IVNs[4].ivnframes[0].FrameID);
                dataStruct.Mem.putULong("IVN5[1]", IVNs[4].ivnframes[1].FrameID);
                dataStruct.Mem.putULong("IVN5[2]", IVNs[4].ivnframes[2].FrameID);
                dataStruct.Mem.putULong("IVN5[3]", IVNs[4].ivnframes[3].FrameID);
                dataStruct.Mem.putULong("IVN5[4]", IVNs[4].ivnframes[4].FrameID);
                sprintf(buff, "#CLR#IVN5#OK#");
            }
            dataStruct.Mem.end();
        }
    }
    /* else if (str.indexOf("RDIAGNOSTICS") > 0)
     {
         if (str.indexOf("GET") > 0)
         {
             sprintf(buff, "#Get#RDIAGNOSTICS#%x|%x|%x#", dataStruct.Rdiag.protocol, dataStruct.Rdiag.TxId, dataStruct.Rdiag.RxId);
         }
         else if (str.indexOf("SET") > 0)
         {
             str.toCharArray(Data, str.length());
             _PTR_(ptr = strtok(strstr(Data, "RDIAGNOSTICS"), ",");
             ptr = strtok(NULL, "|");
             dataStruct.Rdiag.protocol = HexAsciToU32(ptr);
             ptr = strtok(NULL, "|");
             dataStruct.Rdiag.TxId = HexAsciToU32(ptr);
             ptr = strtok(NULL, "#");
             dataStruct.Rdiag.RxId = HexAsciToU32(ptr);
             dataStruct.Mem.begin("CMD");
             dataStruct.Mem.putUChar("PROT", dataStruct.Rdiag.protocol);
             dataStruct.Mem.putULong("TXID", dataStruct.Rdiag.TxId);
             dataStruct.Mem.putULong("RXID", dataStruct.Rdiag.RxId);
             dataStruct.Mem.end();
             sprintf(buff, "#Set#RDIAGNOSTICS#OK#");
         }
         else if (str.indexOf("CLR") > 0)
         {
             dataStruct.Rdiag.protocol = 0;
             dataStruct.Rdiag.TxId = 0;
             dataStruct.Rdiag.RxId = 0;
             dataStruct.Mem.begin("CMD");
             dataStruct.Mem.putUChar("PROT", dataStruct.Rdiag.protocol);
             dataStruct.Mem.putUChar("TXID", dataStruct.Rdiag.TxId);
             dataStruct.Mem.putUChar("RXID", dataStruct.Rdiag.RxId);
             dataStruct.Mem.end();
             sprintf(buff, "#CLR# RDIAGNOSTICS#OK#");
         }
         else
         {
             sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
         }
     }*/
    else if ((str.indexOf("APN") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if ((str.indexOf("GET") > 0) && ((Access == APEOL) || (Access == VMEOL) || (Access == DLR) || (Access == USR)))
        {
            sprintf(buff, "#GET#APN#%s#", dataStruct.GSMData.APN);
        }
        else if ((str.indexOf("SET") > 0) && (Access == APEOL))
        {
            sprintf(buff, "#SET#APN#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "APN"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                strcpy(dataStruct.GSMData.APN, ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putString("APN", dataStruct.GSMData.APN);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#APN#OK#");
            }
        }
    }
    else if ((str.indexOf("SSID") > 0))
    {
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#Get#SSID#%s#", dataStruct.DEVData.ssid);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#SSID#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "SSID"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                strcpy(dataStruct.DEVData.ssid, ptr); // Serial.println(dataStruct.DEVData.ssid);
                dataStruct.Mem.begin(APP_CNF, false);
                dataStruct.Mem.putString("SSID", dataStruct.DEVData.ssid);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#SSID#OK#");
            }
        }
    }
    else if ((str.indexOf("PSWD") > 0))
    {
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#Get#PSWD#%s#", dataStruct.DEVData.pswd);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#PSWD#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "PSWD"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                strcpy(dataStruct.DEVData.pswd, ptr); // Serial.println(dataStruct.DEVData.pswd);
                dataStruct.Mem.begin(APP_CNF, false);
                dataStruct.Mem.putString("PSWD", dataStruct.DEVData.pswd);
                dataStruct.Mem.putBool("wlan", true);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#PSWD#OK#");
            }
        }
    }
    else if ((str.indexOf("ECUMODEAP") > 0))
    {
        dataStruct.Mem.begin(APP_CNF, false);
        Access = APEOL;
        dataStruct.Mem.putInt("ACCESS", Access);
        dataStruct.Mem.end();
        sprintf(buff, "#SET#ECUMODEAP#OK#");
        SendResp(buff, comCh);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    else if ((str.indexOf("ECUMODEVM") > 0))
    {
        dataStruct.Mem.begin(APP_CNF, false);
        Access = VMEOL;
        dataStruct.Mem.putInt("ACCESS", Access);
        dataStruct.Mem.end();
        sprintf(buff, "#SET#ECUMODEVM#OK#");
        SendResp(buff, comCh);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    else if ((str.indexOf("ECUMODEDLR") > 0))
    {
        dataStruct.Mem.begin(APP_CNF, false);
        Access = DLR;
        dataStruct.Mem.putInt("ACCESS", Access);
        dataStruct.Mem.end();
        sprintf(buff, "#SET#ECUMODEDLR#OK#");
        SendResp(buff, comCh);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    else if ((str.indexOf("ECUMODEUSR") > 0))
    {
        dataStruct.Mem.begin(APP_CNF, false);
        Access = USR;
        dataStruct.Mem.putInt("ACCESS", Access);
        dataStruct.Mem.end();
        sprintf(buff, "#SET#ECUMODEUSR#OK#");
        SendResp(buff, comCh);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    }
    else if (str.indexOf("APMODEKEY") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#APMODEKEY#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, dataStruct.DEVData.APK[i]);
            sprintf(buff, "%s#", buff);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#APMODEKEY#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "APMODEKEY"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                Serial.println(ptr);
                util.HexAsciToU8array(ptr, dataStruct.DEVData.APK);
                sprintf(buff, "#SET#APMODEKEY#OK#");
            }
        }
    }
    else if (str.indexOf("VMMODEKEY") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#VMMODEKEY#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, dataStruct.DEVData.VMEK[i]);
            sprintf(buff, "%s#", buff);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#VMMODEKEY#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "VMMODEKEY"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                Serial.println(ptr);
                util.HexAsciToU8array(ptr, dataStruct.DEVData.VMEK);
                sprintf(buff, "#SET#VMMODEKEY#OK#");
            }
        }
    }
    else if (str.indexOf("DLRMODEKEY") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#DLRMODEKEY#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, dataStruct.DEVData.DCSK[i]);
            sprintf(buff, "%s#", buff);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#DLRMODEKEY#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "DLRMODEKEY"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                Serial.println(ptr);
                util.HexAsciToU8array(ptr, dataStruct.DEVData.DCSK);
                sprintf(buff, "#SET#DLRMODEKEY#OK#");
            }
        }
    }
    else if (str.indexOf("USRMODEKEY") > 0)
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#USRMODEKEY#");
            for (int i = 0; i < 16; i++)
                sprintf(buff, "%s%02x", buff, dataStruct.DEVData.NSK[i]);
            sprintf(buff, "%s#", buff);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#USRMODEKEY#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "USRMODEKEY"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                Serial.println(ptr);
                util.HexAsciToU8array(ptr, dataStruct.DEVData.NSK);
                sprintf(buff, "#SET#USRMODEKEY#OK#");
            }
        }
    }
    else if ((str.indexOf("SOSNUM2") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#SOSNUM2#%s#", dataStruct.AISData.SOSNUM2);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#SOSNUM2#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "SOSNUM2"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                strcpy(dataStruct.AISData.SOSNUM2, ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putString("SOSNUM2", dataStruct.AISData.SOSNUM2);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#SOSNUM2#OK#");
            }
        }
    }
    else if ((str.indexOf("VEHVIN") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if ((str.indexOf("GET") > 0) && ((Access == VMEOL) || (Access == DLR) || (Access == USR)))
        {
            sprintf(buff, "#GET#VEHVIN#%s#", dataStruct.DEVData.VEHVIN);
        }
        else if ((str.indexOf("SET") > 0) && (Access == VMEOL))
        {
            sprintf(buff, "#SET#VEHVIN#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "VEHVIN"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                strcpy(dataStruct.DEVData.VEHVIN, ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putString("VEHVIN", dataStruct.DEVData.VEHVIN);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#VEHVIN#OK#");
            }
        }
        else if ((str.indexOf("CLR") > 0) && (Access == VMEOL))
        {
            strcpy(dataStruct.DEVData.VEHVIN, "00");
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putString("VEHVIN", dataStruct.DEVData.VEHVIN);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#VEHVIN#OK#");
        }
    }
    else if ((str.indexOf("GFENCE1") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#GFENCE1#%d,%d,%d,%lf,%lf#", dataStruct.GPSData.Gstatus[0], dataStruct.GPSData.Gkind[0],
                    dataStruct.GPSData.Grad[0], dataStruct.GPSData.GLAT[0], dataStruct.GPSData.GLONG[0]);
        }
        else if (str.indexOf("SET") > 0)
        {
            FieldCnt = 0;
            sprintf(buff, "#SET#GFENCE1#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "GFENCE1"), ","))
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gstatus[0] = atoi(ptr); // DEBUG_d("Gstatus[0]: %d", dataStruct.GPSData.Gstatus[0]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gkind[0] = atoi(ptr); // DEBUG_d("Gkind[0]: %d", dataStruct.GPSData.Gkind[0]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Grad[0] = atoi(ptr); // DEBUG_d("Grad[0]: %d", dataStruct.GPSData.Grad[0]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.GLAT[0] = atof(ptr); // DEBUG_d("GLAT[0]: %lf", dataStruct.GPSData.GLAT[0]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, "#")) != NULL)
            {
                dataStruct.GPSData.GLONG[0] = atof(ptr); // DEBUG_d("GLONG[0]: %lf", dataStruct.GPSData.GLONG[0]);
                FieldCnt++;
            }

            if (FieldCnt == 5)
            {
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putInt("Gstatus[0]", dataStruct.GPSData.Gstatus[0]);
                dataStruct.Mem.putInt("Gkind[0]", dataStruct.GPSData.Gkind[0]);
                dataStruct.Mem.putInt("Grad[0]", dataStruct.GPSData.Grad[0]);
                dataStruct.Mem.putDouble("GLAT[0]", dataStruct.GPSData.GLAT[0]);
                dataStruct.Mem.putDouble("GLONG[0]", dataStruct.GPSData.GLONG[0]);
                dataStruct.Mem.end();
            }
            sprintf(buff, (FieldCnt == 5) ? "#SET#GFENCE1#OK#" : "#SET#GFENCE1#FAILED#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.GPSData.Gkind[0] = 0;
            dataStruct.GPSData.Gstatus[0] = 0;
            dataStruct.GPSData.Grad[0] = 0;
            dataStruct.GPSData.GLAT[0] = 0;
            dataStruct.GPSData.GLONG[0] = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putInt("Gstatus[0]", dataStruct.GPSData.Gstatus[0]);
            dataStruct.Mem.putInt("Gkind[0]", dataStruct.GPSData.Gkind[0]);
            dataStruct.Mem.putInt("Grad[0]", dataStruct.GPSData.Grad[0]);
            dataStruct.Mem.putDouble("GLAT[0]", dataStruct.GPSData.GLAT[0]);
            dataStruct.Mem.putDouble("GLONG[0]", dataStruct.GPSData.GLONG[0]);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#GFENCE1#OK#");
        }
    }
    else if ((str.indexOf("GFENCE2") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#GFENCE2#%d,%d,%d,%lf,%lf#", dataStruct.GPSData.Gstatus[1], dataStruct.GPSData.Gkind[1],
                    dataStruct.GPSData.Grad[1], dataStruct.GPSData.GLAT[1], dataStruct.GPSData.GLONG[1]);
        }
        else if (str.indexOf("SET") > 0)
        {
            FieldCnt = 0;
            sprintf(buff, "#SET#GFENCE2#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "GFENCE2"), ","))
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gstatus[1] = atoi(ptr); // DEBUG_d("Gstatus[1]: %d", dataStruct.GPSData.Gstatus[1]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gkind[1] = atoi(ptr); // DEBUG_d("Gkind[1]: %d", dataStruct.GPSData.Gkind[1]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Grad[1] = atoi(ptr); // DEBUG_d("Grad[1]: %d", dataStruct.GPSData.Grad[1]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.GLAT[1] = atof(ptr); // DEBUG_d("GLAT[1]: %lf", dataStruct.GPSData.GLAT[1]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, "#")) != NULL)
            {
                dataStruct.GPSData.GLONG[1] = atof(ptr); // DEBUG_d("GLONG[1]: %lf", dataStruct.GPSData.GLONG[1]);
                FieldCnt++;
            }

            if (FieldCnt == 5)
            {
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putInt("Gstatus[1]", dataStruct.GPSData.Gstatus[1]);
                dataStruct.Mem.putInt("Gkind[1]", dataStruct.GPSData.Gkind[1]);
                dataStruct.Mem.putInt("Grad[1]", dataStruct.GPSData.Grad[1]);
                dataStruct.Mem.putDouble("GLAT[1]", dataStruct.GPSData.GLAT[1]);
                dataStruct.Mem.putDouble("GLONG[1]", dataStruct.GPSData.GLONG[1]);
                dataStruct.Mem.end();
            }
            sprintf(buff, (FieldCnt == 5) ? "#SET#GFENCE2#OK#" : "#SET#GFENCE2#FAILED#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.GPSData.Gkind[1] = 0;
            dataStruct.GPSData.Gstatus[1] = 0;
            dataStruct.GPSData.Grad[1] = 0;
            dataStruct.GPSData.GLAT[1] = 0;
            dataStruct.GPSData.GLONG[1] = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putInt("Gstatus[1]", dataStruct.GPSData.Gstatus[1]);
            dataStruct.Mem.putInt("Gkind[1]", dataStruct.GPSData.Gkind[1]);
            dataStruct.Mem.putInt("Grad[1]", dataStruct.GPSData.Grad[1]);
            dataStruct.Mem.putDouble("GLAT[1]", dataStruct.GPSData.GLAT[1]);
            dataStruct.Mem.putDouble("GLONG[1]", dataStruct.GPSData.GLONG[1]);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#GFENCE2#OK#");
        }
    }
    else if ((str.indexOf("GFENCE3") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#GFENCE3#%d,%d,%d,%lf,%lf#", dataStruct.GPSData.Gstatus[2], dataStruct.GPSData.Gkind[2],
                    dataStruct.GPSData.Grad[2], dataStruct.GPSData.GLAT[2], dataStruct.GPSData.GLONG[2]);
        }
        else if (str.indexOf("SET") > 0)
        {
            FieldCnt = 0;
            sprintf(buff, "#SET#GFENCE3#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "GFENCE3"), ","))
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gstatus[2] = atoi(ptr); // DEBUG_d("Gstatus[2]: %d", dataStruct.GPSData.Gstatus[2]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Gkind[2] = atoi(ptr); // DEBUG_d("Gkind[2]: %d", dataStruct.GPSData.Gkind[2]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.Grad[2] = atoi(ptr); // DEBUG_d("Grad[2]: %d", dataStruct.GPSData.Grad[2]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, ",")) != NULL)
            {
                dataStruct.GPSData.GLAT[2] = atof(ptr); // DEBUG_d("GLAT[2]: %lf", dataStruct.GPSData.GLAT[2]);
                FieldCnt++;
            }
            if ((ptr = strtok(NULL, "#")) != NULL)
            {
                dataStruct.GPSData.GLONG[2] = atof(ptr); // DEBUG_d("GLONG[2]: %lf", dataStruct.GPSData.GLONG[2]);
                FieldCnt++;
            }

            if (FieldCnt == 5)
            {
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putInt("Gstatus[2]", dataStruct.GPSData.Gstatus[2]);
                dataStruct.Mem.putInt("Gkind[2]", dataStruct.GPSData.Gkind[2]);
                dataStruct.Mem.putInt("Grad[2]", dataStruct.GPSData.Grad[2]);
                dataStruct.Mem.putDouble("GLAT[2]", dataStruct.GPSData.GLAT[2]);
                dataStruct.Mem.putDouble("GLONG[2]", dataStruct.GPSData.GLONG[2]);
                dataStruct.Mem.end();
            }
            sprintf(buff, (FieldCnt == 5) ? "#SET#GFENCE3#OK#" : "#SET#GFENCE3#FAILED#");
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.GPSData.Gkind[2] = 0;
            dataStruct.GPSData.Gstatus[2] = 0;
            dataStruct.GPSData.Grad[2] = 0;
            dataStruct.GPSData.GLAT[2] = 0;
            dataStruct.GPSData.GLONG[2] = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putInt("Gstatus[2]", dataStruct.GPSData.Gstatus[2]);
            dataStruct.Mem.putInt("Gkind[2]", dataStruct.GPSData.Gkind[2]);
            dataStruct.Mem.putInt("Grad[2]", dataStruct.GPSData.Grad[2]);
            dataStruct.Mem.putDouble("GLAT[2]", dataStruct.GPSData.GLAT[2]);
            dataStruct.Mem.putDouble("GLONG[2]", dataStruct.GPSData.GLONG[2]);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#GFENCE3#OK#");
        }
    }
    else if ((str.indexOf("VEHECUCNT") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#GET#VEHECUCNT#%u#", dataStruct.DEVData.VEHECUCNT);
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#SET#VEHECUCNT#FAILED#");
            str.toCharArray(Data, str.length());
            _PTR_(ptr = strtok(strstr(Data, "VEHECUCNT"), ","))
            ptr = strtok(NULL, "#");
            if (ptr != NULL)
            {
                dataStruct.DEVData.VEHECUCNT = atoi(ptr);
                dataStruct.Mem.begin("CMD", false);
                dataStruct.Mem.putUChar("VEHECUCNT", dataStruct.DEVData.VEHECUCNT);
                dataStruct.Mem.end();
                sprintf(buff, "#SET#VEHECUCNT#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            dataStruct.DEVData.VEHECUCNT = 0;
            dataStruct.Mem.begin("CMD", false);
            dataStruct.Mem.putUChar("VEHECUCNT", dataStruct.DEVData.VEHECUCNT);
            dataStruct.Mem.end();
            sprintf(buff, "#CLR#VEHECUCNT#OK#");
        }
    }
    else if ((str.indexOf("ECUPARAM") > 0))
    {
        sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_10);
        if (str.indexOf("GET") > 0)
        {
            sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_12);
            if (str.indexOf("ECUPARAM1") > 0)
            {
                sprintf(buff, "#GET#ECUPARAM1#%s#", dataStruct.DEVData.ECUData[0].getParamString().c_str());
            }
            else if (str.indexOf("ECUPARAM2") > 0)
            {
                sprintf(buff, "#GET#ECUPARAM2#%s#", dataStruct.DEVData.ECUData[1].getParamString().c_str());
            }
            else if (str.indexOf("ECUPARAM3") > 0)
            {
                sprintf(buff, "#GET#ECUPARAM3#%s#", dataStruct.DEVData.ECUData[2].getParamString().c_str());
            }
            else if (str.indexOf("ECUPARAM4") > 0)
            {
                sprintf(buff, "#GET#ECUPARAM4#%s#", dataStruct.DEVData.ECUData[3].getParamString().c_str());
            }
            else if (str.indexOf("ECUPARAM5") > 0)
            {
                sprintf(buff, "#GET#ECUPARAM5#%s#", dataStruct.DEVData.ECUData[4].getParamString().c_str());
            }
        }
        else if (str.indexOf("SET") > 0)
        {
            sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_12);
            if (str.indexOf("ECUPARAM1") > 0)
            {
                sprintf(buff, "#SET#ECUPARAM1#FAILED#");
                str.toCharArray(Data, str.length());
                if (dataStruct.DEVData.ECUData[0].setParameters(Data, "ECUPARAM1"))
                    sprintf(buff, "#SET#ECUPARAM1#OK#");
            }
            else if (str.indexOf("ECUPARAM2") > 0)
            {
                sprintf(buff, "#SET#ECUPARAM2#FAILED#");
                str.toCharArray(Data, str.length());
                if (dataStruct.DEVData.ECUData[1].setParameters(Data, "ECUPARAM2"))
                    sprintf(buff, "#SET#ECUPARAM2#OK#");
            }
            else if (str.indexOf("ECUPARAM3") > 0)
            {
                sprintf(buff, "#SET#ECUPARAM3#FAILED#");
                str.toCharArray(Data, str.length());
                if (dataStruct.DEVData.ECUData[2].setParameters(Data, "ECUPARAM3"))
                    sprintf(buff, "#SET#ECUPARAM3#OK#");
            }
            else if (str.indexOf("ECUPARAM4") > 0)
            {
                sprintf(buff, "#SET#ECUPARAM4#FAILED#");
                str.toCharArray(Data, str.length());
                if (dataStruct.DEVData.ECUData[3].setParameters(Data, "ECUPARAM4"))
                    sprintf(buff, "#SET#ECUPARAM4#OK#");
            }
            else if (str.indexOf("ECUPARAM5") > 0)
            {
                sprintf(buff, "#SET#ECUPARAM5#FAILED#");
                str.toCharArray(Data, str.length());
                if (dataStruct.DEVData.ECUData[4].setParameters(Data, "ECUPARAM5"))
                    sprintf(buff, "#SET#ECUPARAM5#OK#");
            }
        }
        else if (str.indexOf("CLR") > 0)
        {
            sprintf(buff, "#ERROR#%02X#", APP_RESP_NACK_12);
            if (str.indexOf("ECUPARAM1") > 0)
            {
                dataStruct.DEVData.ECUData[0].clearAllParameters();
                sprintf(buff, "#CLR#ECUPARAM1#OK#");
            }
            else if (str.indexOf("ECUPARAM2") > 0)
            {
                dataStruct.DEVData.ECUData[1].clearAllParameters();
                sprintf(buff, "#CLR#ECUPARAM2#OK#");
            }
            else if (str.indexOf("ECUPARAM3") > 0)
            {
                dataStruct.DEVData.ECUData[2].clearAllParameters();
                sprintf(buff, "#CLR#ECUPARAM3#OK#");
            }
            else if (str.indexOf("ECUPARAM4") > 0)
            {
                dataStruct.DEVData.ECUData[3].clearAllParameters();
                sprintf(buff, "#CLR#ECUPARAM4#OK#");
            }
            else if (str.indexOf("ECUPARAM5") > 0)
            {
                dataStruct.DEVData.ECUData[4].clearAllParameters();
                sprintf(buff, "#CLR#ECUPARAM5#OK#");
            }
        }
    }

_EXET:
    SendResp(buff, comCh);
}

void AIS140::testFunc()
{
    updateGPS_Data();
}

/*********************** PRIVATE METHODS ************************/
bool AIS140::devConfigInit()
{
    dataStruct.Mem.begin(APP_CNF, true);
    Access = dataStruct.Mem.getInt("ACCESS", APEOL);
    wlan = dataStruct.Mem.getBool("wlan", false);
    dataStruct.Mem.end();
    dataStruct.DataStructInit();
    return true;
}

bool AIS140::userModeinit()
{
    STATUS_CHECK(telemSoc.begin(GSM_UART))

    strcpy(dataStruct.GSMData.IMEI, telemSoc.imei());
    strcpy(dataStruct.GSMData.NETOPNAME, telemSoc.getNetworkOperator());
    DEBUG_i("DEV: IMEI: %s", dataStruct.GSMData.IMEI);
    DEBUG_i("DEV: REG: %s", dataStruct.DEVData.VEHREGNUM);

    while (!srvLogin(dataStruct.SRVData.IP1, dataStruct.SRVData.IP1P))
    {
        DEBUG_w("[RETRY SRV]");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    StartTimer(TrackPackTmr, 2000);
    StartTimer(HealthPackTmr, 3000); // dataStruct.AISData.HLTINT);
    if (dataStruct.DEVData.IVN1INT)
        StartTimer(ivn1Tmr, dataStruct.DEVData.IVN1INT);
    if (dataStruct.DEVData.IVN2INT)
        StartTimer(ivn2Tmr, dataStruct.DEVData.IVN2INT);
    if (dataStruct.DEVData.IVN3INT)
        StartTimer(ivn3Tmr, dataStruct.DEVData.IVN3INT);
    if (dataStruct.DEVData.IVN4INT)
        StartTimer(ivn4Tmr, dataStruct.DEVData.IVN4INT);
    if (dataStruct.DEVData.IVN5INT)
        StartTimer(ivn5Tmr, dataStruct.DEVData.IVN5INT);

    return true;
}

void AIS140::userModeDataProcess(uint8_t index)
{
    while (telemSoc.available(index))
    {
        char c = (char)telemSoc.read(index);
        if (c == '*')
        {
            SrvData = c;
            do
            {
                c = (char)telemSoc.read(index);
                SrvData += c;
            } while (telemSoc.available(index) && c != '#');
            ParseCommands(SrvData, index);
        }
        else if (c == '$')
        {
            SrvData = c;
            do
            {
                c = (char)telemSoc.read(index);
                SrvData += c;
            } while (telemSoc.available(index) && c != '&');
            SrvReply(SrvData);
        }
        else if (c == '{')
        {
            SrvData.clear();
            do
            {
                c = (char)telemSoc.read(index);
                IF(c == '}', break;)
                SrvData += c;
            } while (telemSoc.available(index));
            remoteDiag(SrvData, index);
        }
        else
        {
            DEBUG_w("ERR [%u] [", telemSoc.available(index));
            int x;
            while ((x = telemSoc.available(index)) > 0)
            {
                while (x--)
                    Serial.print(((char)telemSoc.read(index)));
            }
            Serial.println("]");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void AIS140::periodicTransmit()
{
    if (IsTimerElapsed(TrackPackTmr))
    {
        ResetTimer(TrackPackTmr, 2000);
        updateGSM_Data();
        updateGPS_Data();
        updateIO_Data(); // debug_d("[TRK: %s]", trackPack.Packetize());
        if (!telemSoc.print(trackPack.Packetize(), srv_telematics))
        {
            if (trackPack.pushFrame())
            {
                dataStruct.AISData.Mem1Cnt++;
                DEBUG_w("[TRK] [pushFrame]");
            }
        }

        if (trackPack.avaiableFrame())
        {
            if (!telemSoc.print(trackPack.popFrame(), srv_telematics))
                DEBUG_w("[TRK] [Send]");
        }
    }

    if (IsTimerElapsed(HealthPackTmr) /*&& (dataStruct.AISData.HLTINT)*/)
    {
        ResetTimer(HealthPackTmr, 3000); // dataStruct.AISData.HLTINT
        // debug_d("[HLT: %s]", healthPack.Packetize());

        if (!telemSoc.print(healthPack.Packetize(), srv_telematics))
        {
            if (healthPack.pushFrame())
            {
                dataStruct.AISData.Mem1Cnt++;
                DEBUG_w("[HLT] [pushFrame]");
            }
        }

        if (healthPack.avaiableFrame())
        {
            if (!telemSoc.print(healthPack.popFrame(), srv_telematics))
                DEBUG_w("[HLT] [Send]");
        }
    }

    if (dataStruct.DEVData.IVN1INT && IsTimerElapsed(ivn1Tmr))
    {
        ResetTimer(ivn1Tmr, dataStruct.DEVData.IVN1INT);
        String data = "$";
        data += String(dataStruct.DEVData.deviceType) + "-";
        data += String("IVN1") + ",";
        data += String(dataStruct.DEVData.vendorID) + ",";
        data += String(dataStruct.GSMData.IMEI) + ",";
        data += dataStruct.GetIvnFrame(0);
        data += "*50";
        // debug_d("ivn1: %s", data.c_str());
        SendResp((char *)data.c_str(), COM_GSM1);
    }

    if (dataStruct.DEVData.IVN2INT && IsTimerElapsed(ivn2Tmr))
    {
        ResetTimer(ivn2Tmr, dataStruct.DEVData.IVN2INT);
        String data = "$";
        data += String(dataStruct.DEVData.deviceType) + "-";
        data += String("IVN2") + ",";
        data += String(dataStruct.DEVData.vendorID) + ",";
        data += String(dataStruct.GSMData.IMEI) + ",";
        data += dataStruct.GetIvnFrame(1);
        data += "*50";
        // debug_d("ivn2: %s", data.c_str());
        SendResp((char *)data.c_str(), COM_GSM1);
    }

    if (dataStruct.DEVData.IVN3INT && IsTimerElapsed(ivn3Tmr))
    {
        ResetTimer(ivn3Tmr, dataStruct.DEVData.IVN3INT);
        String data = "$";
        data += String(dataStruct.DEVData.deviceType) + "-";
        data += String("IVN3") + ",";
        data += String(dataStruct.DEVData.vendorID) + ",";
        data += String(dataStruct.GSMData.IMEI) + ",";
        data += dataStruct.GetIvnFrame(2);
        data += "*50";
        // debug_d("ivn3: %s", data.c_str());
        SendResp((char *)data.c_str(), COM_GSM1);
    }

    if (dataStruct.DEVData.IVN4INT && IsTimerElapsed(ivn4Tmr))
    {
        ResetTimer(ivn4Tmr, dataStruct.DEVData.IVN4INT);
        String data = "$";
        data += String(dataStruct.DEVData.deviceType) + "-";
        data += String("IVN4") + ",";
        data += String(dataStruct.DEVData.vendorID) + ",";
        data += String(dataStruct.GSMData.IMEI) + ",";
        data += dataStruct.GetIvnFrame(3);
        data += "*50";
        // debug_d("ivn4: %s", data.c_str());
        SendResp((char *)data.c_str(), COM_GSM1);
    }

    if (dataStruct.DEVData.IVN5INT && IsTimerElapsed(ivn5Tmr))
    {
        ResetTimer(ivn5Tmr, dataStruct.DEVData.IVN5INT);
        String data = "$";
        data += String(dataStruct.DEVData.deviceType) + "-";
        data += String("IVN5") + ",";
        data += String(dataStruct.DEVData.vendorID) + ",";
        data += String(dataStruct.GSMData.IMEI) + ",";
        data += dataStruct.GetIvnFrame(4);
        data += "*50";
        // debug_d("ivn5: %s", data.c_str());
        SendResp((char *)data.c_str(), COM_GSM1);
    }
}

// ============================ Remote Diagnostics ============================ //

void AIS140::remoteDiag(String &str, int comCh)
{
    DEBUG_d("[RDG] > [%s]", str.c_str());
    uint32_t idx = 0, i = 0;
    uint8_t Nb1 = 0, Nb2 = 0;
    do
    {
        IF((Nb1 = str[i++]) == 0x2c, continue;)
        Nb2 = str[i++];
        Nb1 = (Nb1 > 64) ? (Nb1 - 87) : (Nb1 - 48);
        Nb2 = (Nb2 > 64) ? (Nb2 - 87) : (Nb2 - 48);
        RDG_RXBuff[idx++] = (Nb1 << 4) | (Nb2);
    } while (i < str.length());
    remoteDiag(RDG_RXBuff, idx);
}

void AIS140::remoteDiag(uint8_t *data, uint16_t len)
{
    uint16_t idx = 0, frameLen = 0;
    RDG_FR_CNT = 0;

    while (idx < (len))
    {
        frameLen = ((uint16_t)(data[idx] & 0x0F) << 8) | (uint16_t)data[idx + 1];
        RDG_FR_CNT += ((data[idx] & 0xF0) == 0x40) ? 2 : 1;
        idx += frameLen + 2;
    }
    idx = frameLen = 0;

    while (idx < (len))
    {
        frameLen = ((uint16_t)(data[idx] & 0x0F) << 8) | (uint16_t)data[idx + 1];
        APP_ProcessData(&data[idx], (frameLen + 2), (APP_CHANNEL_t)0);
        idx += frameLen + 2;
    }
}

void SendDiagnosticPacket(uint8_t *data, uint16_t len)
{
    IF(rdgTmpCnt == 0, sprintf(RdgRsp, "#remote,");)
    rdgTmpCnt++;

    if (RDG_FR_CNT == rdgTmpCnt)
    {
        for (int i = 0; i < len; i++)
        {
            sprintf(RdgRsp, "%s%02x", RdgRsp, data[i]);
        }
        sprintf(RdgRsp, "%s#", RdgRsp);
        DEBUG_d("[RDG] < [%s]", RdgRsp);
        telemSoc.print(RdgRsp, srv_telematics);
        rdgTmpCnt = 0;
    }
    else
    {
        for (int i = 0; i < len; i++)
        {
            sprintf(RdgRsp, "%s%02x", RdgRsp, data[i]);
        }
        sprintf(RdgRsp, "%s,", RdgRsp);
    }
}

//---------------------------------------------------------------//

bool AIS140::srvLogin(const char *host, uint16_t port)
{
    LogInPacket logPack;
    STATUS_CHECK_EXE(telemSoc.connect(host, port, srv_telematics))
    STATUS_CHECK_EXE(telemSoc.print(logPack.Packetize(), srv_telematics))
    DEBUG_d("DEV: %s", logPack.Packetize());

    StartTimer(timOutTmr, 5000);
    do
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        while (telemSoc.available(srv_telematics))
            SrvData += (char)telemSoc.read(srv_telematics);
        if ((SrvData.indexOf("login") > 0) || (SrvData.indexOf("ERROR") > 0))
            break;
    } while (IsTimerRunning(timOutTmr));

    if ((IsTimerRunning(timOutTmr)) && (SrvData.indexOf("login") > 0))
        DEBUG_i("SRV: Login PASS");
    else
    {
        DEBUG_w("LogIn Failed <%s>", SrvData.c_str());
        return false;
    }

    return true;
}

void AIS140::srvConnectionHandler(uint8_t index)
{
    if (index == srv_telematics)
    {
        DEBUG_w("[Telematics Connetion Lost: Retrying...]");
        if (!telemSoc.connect(dataStruct.SRVData.IP1, dataStruct.SRVData.IP1P, srv_telematics))
        {
            LoginFailCnt++;
        }
        else
        {
            DEBUG_i("[Telematics Connetion Established!]");
            LoginFailCnt = 0;
        }
        if (LoginFailCnt == 10)
        {
            DEBUG_e("!!! [FETAL] [Rebooting..]");
            esp_restart();
        }
    }
    else if (index == srv_emergency)
    {
        DEBUG_w("[Emergency Connetion Lost: Retrying...]");
        if (!telemSoc.connect(dataStruct.SRVData.IP2, dataStruct.SRVData.IP2P, srv_emergency))
        {
            LoginFailCnt++;
        }
        else
        {
            DEBUG_i("[Emergency Connetion Established!]");
            LoginFailCnt = 0;
        }
        if (LoginFailCnt == 10)
        {
            DEBUG_e("!!! [FETAL] [Rebooting..]");
            esp_restart();
        }
    }
}

int AIS140::SrvReply(String &str)
{
    DEBUG_d("[%s]", str.c_str());
    return ESP_OK;
}

int AIS140::SendResp(char *buf, int comCh)
{
    switch (comCh)
    {
    case COM_GSM1:
        telemSoc.print(buf, srv_telematics);
        break;

    case COM_GSM2:
        telemSoc.print(buf, srv_emergency);
        break;

    case COM_USB:
        Serial.write(buf);
        break;

    case COM_WIFI:
        wClient.write(buf, strlen(buf));
        break;

    default:
        DEBUG_w("INVALID COM_CH [%s]", buf);
        break;
    }
    return ESP_OK;
}

void AIS140::updateGSM_Data()
{
    char *ptr, *tok, *c_info = NULL;
    char QENG1[512];
    dataStruct.GSMData.GSMSIGNAL = telemSoc.getRSSI();
    PTR_CHECK_RET(c_info = telemSoc.getCellInfo(), )
    PTR_CHECK_RET((ptr = strstr(c_info, "+QENG: 1")), )
    strcpy(QENG1, ptr);                                 // DEBUG_v("[QENG1: %s]", QENG1);
    PTR_CHECK_RET((ptr = strstr(c_info, "+QENG: 0")), ) // DEBUG_v("[QENG0: %s]", ptr);
    PTR_CHECK_RET(tok = strtok(ptr, ","), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.MCC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.MNC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.LAC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.CELLID, tok);
    PTR_CHECK_RET((tok = strtok(QENG1, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    IF((strcmp(tok, "1") != 0), return;)
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR1_SIG, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR1_LAC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR2_CELLID, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    IF((strcmp(tok, "2") != 0), return;)
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR2_SIG, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR2_LAC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR2_CELLID, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    IF((strcmp(tok, "3") != 0), return;)
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR3_SIG, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR3_LAC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR3_CELLID, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    IF((strcmp(tok, "4") != 0), return;)
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR4_SIG, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR4_LAC, tok);
    PTR_CHECK_RET((tok = strtok(NULL, ",")), )
    strcpy(dataStruct.GSMData.NMR4_CELLID, tok);

    // if((tok = strtok(NULL, ",")) == NULL) return;
    // DEBUG_v("[tok: %s]", tok);
    // DEBUG_v("[INFO: %s %s %s %s]", dataStruct.GSMData.MCC, dataStruct.GSMData.MNC, dataStruct.GSMData.LAC, dataStruct.GSMData.CELLID);
    // DEBUG_v("[NMR1: %s %s %s]", dataStruct.GSMData.NMR1_SIG, dataStruct.GSMData.NMR1_LAC, dataStruct.GSMData.NMR2_CELLID);
    // DEBUG_v("[NMR2: %s %s %s]", dataStruct.GSMData.NMR2_SIG, dataStruct.GSMData.NMR2_LAC, dataStruct.GSMData.NMR2_CELLID);
    // DEBUG_v("[NMR3: %s %s %s]", dataStruct.GSMData.NMR3_SIG, dataStruct.GSMData.NMR3_LAC, dataStruct.GSMData.NMR3_CELLID);
    // DEBUG_v("[NMR4: %s %s %s]", dataStruct.GSMData.NMR4_SIG, dataStruct.GSMData.NMR4_LAC, dataStruct.GSMData.NMR4_CELLID);
}

void AIS140::updateGPS_Data()
{
    dataStruct.updateGPSData((char *)telemSoc.getGPSFrame());

    for(int i = 0; i < 3; i++)
    {
        IF(dataStruct.checkGeoFence(i), DEBUG_i("[GEO FENCE %u TRIGERED", (i + 1));)
    }
}

void AIS140::updateIO_Data()
{
    sprintf(dataStruct.AISData.PKTYPE, "NR");

    DEBUG_v("[SOS: %d IGN: %d Main: %2.2lf IntBatt: %2.2lf]", digitalRead(DI_SOS), digitalRead(DI_IGN),
            (double)(analogRead(Adc_MainSupp) * 0.010017), (double)(analogRead(Adc_intBatt) * 0.001603));

    float mainSup = (float)(analogRead(Adc_MainSupp) * 0.010017);
    float intBatt = (float)(analogRead(Adc_intBatt) * 0.001603);

    sprintf(dataStruct.DEVData.MVOLT, "%2.2f", mainSup);
    sprintf(dataStruct.DEVData.INTVOLT, "%2.2f", intBatt);
    dataStruct.DEVData.BATTPER = map((long)mainSup, 4, 12, 0, 100);

    if (digitalRead(DI_SOS))
    {
        Emrgncy = true;
        StartTimer(EmrPackTmr, 5000);
        DEBUG_i("[SOS START]");
    }

    if ((!dataStruct.DEVData.IGNISTAT) && (digitalRead(DI_IGN)))
    {
        dataStruct.DEVData.IGNISTAT = true;
        sprintf(dataStruct.AISData.PKTYPE, "IN");
    }

    if ((dataStruct.DEVData.IGNISTAT) && (!digitalRead(DI_IGN)))
    {
        dataStruct.DEVData.IGNISTAT = false;
        sprintf(dataStruct.AISData.PKTYPE, "IF");
    }

    if (dataStruct.DEVData.MPOW && !(mainSup > (float)5.00))
    {
        sprintf(dataStruct.AISData.PKTYPE, "BD");
    }

    if (!dataStruct.DEVData.MPOW && (mainSup > (float)5.00))
    {
        dataStruct.DEVData.MPOW = true;
        sprintf(dataStruct.AISData.PKTYPE, "BR");
    }

    if (dataStruct.DEVData.BATTPER < dataStruct.DEVData.BATTLOWTH)
    {
        sprintf(dataStruct.AISData.PKTYPE, "BL");
    }
}

//--------------------------------

void AIS140::AdminCmdParse(String &str, int comCh)
{
    char buff[512];
    if (str.indexOf("SET") > 0)
    {
        if (str.indexOf("EMR") > 0)
        {
            Emrgncy = true;
            StartTimer(EmrPackTmr, 5000);
            DEBUG_i("SOS START");
            sprintf(buff, "SET|EMR|OK");
        }
    }
    else if (str.indexOf("CLR") > 0)
    {
        if (str.indexOf("EMR") > 0)
        {
            Emrgncy = false;
            StopTimer(EmrPackTmr);
            DEBUG_i("SOS STOP");
            telemSoc.disconnect(srv_emergency);
            vTaskDelay(pdMS_TO_TICKS(1000));
            sprintf(buff, "CLR|EMR|OK");
        }
    }
    SendResp(buff, comCh);
    return;
}
