#pragma once

#include <WiFi.h>
#include "ESPmDNS.h"
#include "gsm.hpp"
#include "util.hpp"
#include "packets.hpp"
#include "app.hpp"
#include "ivn.hpp"
#include "caniso.hpp"

enum com_ch_e
{
    COM_GSM1,
    COM_GSM2,
    COM_USB,
    COM_WIFI,
    COM_BLE,
};

enum access_e
{
    APEOL = 0,
    VMEOL = 1,
    DLR = 2,
    USR = 3,
};

void dev_core_task(void *args);
void SendDiagnosticPacket(uint8_t *data, uint16_t len);

class AIS140
{

private:
    char RxBuff[1500];
    char TxBuff[1500];

    struct AES_ctx ctx;
    uint8_t seedKey[16];
    char seedKeyStr[33];
    bool Security;

    uint8_t LoginFailCnt;
    uint32_t srvStateTmr;

    const char *APP_CNF = "CNFG";
    int Access;
    bool wlan;
    bool Emrgncy;
    int devEvtType;

    uint32_t timOutTmr;
    uint32_t ivn1Tmr, ivn2Tmr, ivn3Tmr, ivn4Tmr, ivn5Tmr;
    uint16_t TrackPackIntVal;
    uint32_t TrackPackTmr;
    uint32_t HealthPackTmr;
    uint32_t EmrPackTmr;

    bool devConfigInit();
    bool userModeinit();

    bool srvLogin(const char *host, uint16_t port);
    void updateGSM_Data();
    void updateGPS_Data();
    void updateIO_Data();
    void updateIMU_Data();
    void periodicTransmit();
    void userModeDataProcess(uint8_t index);
    
    void srvConnectionHandler(uint8_t index);
    int SendResp(char *buf, int comCh);
    
    int SrvReply(String &str);
    void remoteDiag(String &str, int comCh);
    void remoteDiag(uint8_t *data, uint16_t len);
    void AdminCmdParse(String &str, int comCh);
    

public:
    String SrvData;

    void Init(void);
    void DataProcess(void);
    void ParseCommands(String &str, int comCh);
    void testFunc();
};
