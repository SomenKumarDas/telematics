#pragma once
#include "util.hpp"

#define GSM_UART Serial2
#define GSM_CHECK_ERR(x) if(!x) { DEBUG_e("[GSM] [%s]", #x); return false;}
#define GSM_CHECK_ERR_RET(x,y) if(!x) { DEBUG_e("[GSM] [%s]", #x); return y;}
#define GSM_CHECK_OK(x) if(x) { return true; }


enum NetworkRegistrationState
{
    NotRegistered = 0,
    Registered,
    Searching,
    Denied,
    Unknown,
    Roaming
};

enum State_e
{
    invalid = 0,
    IP_INITIAL,
    IP_START,
    IP_CONFIG,
    IP_IND,
    IP_GPRSACT,
    IP_STATUS,
    CONNECTING,
    IP_CLOSE,
    CONNECT_OK,
    PDP_DEACT
};

class QuectelCellular
{

public:
    bool begin(HardwareSerial &uart);
    const char* imei(){return _DeviceIMEI;}
    char *getGPSFrame();
    char *getCellInfo();
    char* getNetworkOperator() {return _SrvsProName;}
    uint8_t getRSSI();

    bool connect(const char *host, uint16_t port);
    bool disconnect();
    bool connected();
    int available();
    uint8_t read();
    bool read(uint8_t *dest, uint16_t size);
    bool write(uint8_t data);
    bool write(uint8_t *buff, uint16_t len);
    bool print(const char *buff);

    //Multi tcp:
    bool connect(const char *host, uint16_t port, uint8_t index);
    bool connected(uint8_t index);
    int ipState(uint8_t index);
    int connectionStateHandle(int check, uint8_t index);
    int available(uint8_t index);
    bool r_available(uint8_t index);
    bool write(uint8_t *buff, uint16_t len, uint8_t index);
    bool print(const char *buff, uint8_t index);
    uint8_t read(uint8_t index);

private:
    HardwareSerial *_uart;
    char ATReq[255], ATRsp[1024];
    char _DeviceIMEI[20], _SrvsProName[20], _GPSData[1024], _CellInfo[512];
    bool isNetworkConnected;
    uint32_t timOutTmr;

    char rxBuffer[2048], TxBuffer[2048], *rxPtr;
    uint16_t rxLen;
    // multi tcp
    char tcp_rxBuff[2][2048], tcp_txBuff[2][2048], *tcp_rxPtr[2];
    uint16_t tcp_rxLen[2];

    bool getSimPresent();
    const char* getSimServiceProvider();
    NetworkRegistrationState getNetworkRegistration();
    bool getNetworkRegistration(uint32_t timeout);
    bool getIMEI();

    bool connectNetwork(const char* apn);
    bool disconnectNetwork();
    bool isGPRS();
    int ipState();
    int connectionStateHandle(int check);
    bool r_available();

    int modemQuaryVar(const char *cmd, const char *reply);
    bool AT_CheckReply(const char *command, const char *reply = "OK", uint16_t timeout = 1000);
    bool AT_CheckMultiReply(const char *command, const char *reply = "OK", uint16_t timeout = 1000);
    bool AT_WaitFor(const char *command, const char *reply, uint16_t timeout = 1000);
    bool AT_WaitFor(const char *command, uint8_t lines, uint16_t timeout = 1000);
    bool AT_readReply(uint8_t lines, uint16_t timeout);
    bool AT_readReply(const char *reply, uint16_t timeout);
    void AT_flush();

    void gsm_log_req(const char *cmd);
    void gsm_log_rsp(String reply);
};