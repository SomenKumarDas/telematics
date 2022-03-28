#include "gsm.hpp"

#define QC QuectelCellular
#define ATCMD(x) "AT+"
#define MS_SEC(x) ((x)*1000)
#define SEARCH_STR_RET(buff, key, ret) \
    if ((strstr(buff, key) != NULL))   \
    return ret

/***PUBLIC METHODS******/

bool QC::begin(HardwareSerial &uart)
{
    _uart = &uart;
    _uart->setRxBufferSize(2048);
    while (!_uart)
    {
        DEBUG_e("GSM UART Disable");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    DEBUG_i("[Initializing GSM Module...]");
    GSM_CHECK_ERR(AT_CheckMultiReply("AT+CFUN=1,1", "OK", MS_SEC(10)))
    vTaskDelay(pdMS_TO_TICKS(5000));
    DEBUG_i("[GSM Ready!]");
    GSM_CHECK_ERR(AT_CheckMultiReply("ATE0"))
    GSM_CHECK_ERR(AT_CheckReply("AT+CMEE=2"))
    DEBUG_i("[SIM %s]", (getSimPresent()) ? "VALID" : "NOT VALID");
    GSM_CHECK_ERR(getIMEI())
    DEBUG_i("[WAITING NETWORK REGISTRATION]");
    GSM_CHECK_ERR(getNetworkRegistration(60000))
    DEBUG_i("[GSM SIGNAL RSSI: %u]", getRSSI());
    DEBUG_i("[SERVICE PROVIDER %s]", getSimServiceProvider());
    GSM_CHECK_ERR(AT_CheckReply("AT+QENG=1,1"))
    GSM_CHECK_ERR(AT_CheckReply("AT+QINDI=1"))
    // GSM_CHECK_ERR(AT_CheckReply("AT+QGNSSC=1"))
    return true;
}

char *QC::getGPSFrame()
{
    if (AT_WaitFor("AT+QGNSSRD?", "OK"))
    {
        strcpy(_GPSData, ATRsp);
        char *ptr;
        PTR_CHECK_RET((ptr = strstr(ATRsp, "$")), NULL)
        strcpy(_GPSData, ptr); // DEBUG_d("[GPS] [%s]", _GPSData);
        return _GPSData;
    }
    return NULL;
}

char *QC::getCellInfo()
{
    if (AT_WaitFor("AT+QENG?", "OK"))
    {
        strcpy(_CellInfo, ATRsp);
        char *ptr;
        if ((ptr = strstr(ATRsp, "+QENG: 0")) == NULL)
            return NULL;
        strcpy(_CellInfo, ptr);
        return _CellInfo;
    }
    return NULL;
}

uint8_t QC::getRSSI()
{
    char *token, *ptr;
    GSM_CHECK_ERR(AT_WaitFor("AT+CSQ", 3))
    PTR_CHECK_RET((token = strtok(ATRsp, " ")), 0)
    PTR_CHECK_RET((token = strtok(nullptr, ",")), 0)
    return strtol(token, &ptr, 10);
}

bool QC::connect(const char *host, uint16_t port)
{
    StartTimer(timOutTmr, MS_SEC(65));
    while ((connectionStateHandle(IP_GPRSACT) != IP_GPRSACT))
    {
        if (IsTimerElapsed(timOutTmr))
            return false;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    sprintf(ATReq, "AT+QIOPEN=\"TCP\",\"%s\",%u", host, port);
    GSM_CHECK_ERR(AT_CheckReply(ATReq, "OK", MS_SEC(65)));
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (ipState() == CONNECTING)
        vTaskDelay(pdMS_TO_TICKS(500));
    GSM_CHECK_OK((ipState() == CONNECT_OK))

    return false;
}

bool QC::disconnect()
{
    if (ipState() == CONNECT_OK)
    {
        GSM_CHECK_ERR(AT_CheckReply("AT+QICLOSE", "CLOSE OK"));
    }
    return true;
}

bool QC::connected()
{
    return (ipState() == CONNECT_OK);
}

int QC::available()
{
    IF(rxLen, return rxLen;)
    IF(!r_available(), return 0;)
    char *ptr;
    PTR_CHECK_RET((ptr = strstr(rxBuffer, "+QIRD:")), 0);
    PTR_CHECK_RET((ptr = strtok(ptr, "\n")), 0);
    PTR_CHECK_RET((ptr = strtok(NULL, "\n")), 0); // DEBUG_v("Data: [%u] [%s]", strlen(ptr), ptr);
    rxLen = strlen(ptr);
    rxPtr = ptr;
    return rxLen;
}

uint8_t QC::read()
{
    if (rxLen)
    {
        rxLen--;
        return *rxPtr++;
    }
    return 0;
}

bool QC::read(uint8_t *dest, uint16_t size)
{
    DEBUG_e("!!! :NIP");
    return false;
}

bool QC::write(uint8_t data)
{
    return write(&data, 1);
}

bool QC::write(uint8_t *buff, uint16_t len)
{
    GSM_CHECK_ERR(connected())
    sprintf((char *)TxBuffer, "AT+QISEND=%u", len);
    GSM_CHECK_ERR(AT_WaitFor((char *)TxBuffer, "> ", 300))
    GSM_CHECK_ERR(AT_CheckReply((char *)buff, "SEND OK"))
    return true;
}

bool QC::print(const char *buff)
{
    return write((uint8_t *)buff, strlen(buff));
}

/***PRIVATE METHODS*****/
/*------------------------------------------------------------------------------------------*/

bool QC::connectNetwork(const char *apn)
{
    GSM_CHECK_OK(isNetworkConnected)
    GSM_CHECK_ERR(AT_CheckReply("AT+QIFGCNT=0"))
    if (!isGPRS())
    {
        DEBUG_w("[FUCK : QICSGP]");
        return false;
    }
    GSM_CHECK_ERR(AT_CheckReply("AT+QIMUX=0"))
    GSM_CHECK_ERR(AT_CheckReply("AT+QIMODE=0"))
    GSM_CHECK_ERR(AT_CheckReply("AT+QIDNSIP=0"))
    GSM_CHECK_ERR(AT_CheckReply("AT+QIREGAPP"))
    GSM_CHECK_ERR(AT_CheckMultiReply("AT+QIACT", "OK", MS_SEC(65)))
    return (isNetworkConnected = true);
}

bool QC::disconnectNetwork()
{
    // if (modemQuaryVar("AT+QISRVC?", "+QISRVC") == 2)
    // {
    //     GSM_CHECK_ERR(AT_CheckReply("AT+QISRVC=1"))
    // }

    IF((modemQuaryVar("AT+QISRVC?", "+QISRVC") == 2), GSM_CHECK_ERR(AT_CheckReply("AT+QISRVC=1")))
    GSM_CHECK_ERR(AT_CheckReply("AT+QIDEACT", "DEACT OK", MS_SEC(40)))
    isNetworkConnected = false;
    return true;
}

int QC::ipState()
{
    GSM_CHECK_ERR(AT_WaitFor("AT+QISTAT", 3))
    SEARCH_STR_RET(ATRsp, "CONNECT OK", CONNECT_OK);
    SEARCH_STR_RET(ATRsp, "INITIAL", IP_INITIAL);
    SEARCH_STR_RET(ATRsp, "START", IP_START);
    SEARCH_STR_RET(ATRsp, "CONFIG", IP_CONFIG);
    SEARCH_STR_RET(ATRsp, "IND", IP_IND);
    SEARCH_STR_RET(ATRsp, "GPRSACT", IP_GPRSACT);
    SEARCH_STR_RET(ATRsp, "STATUS", IP_STATUS);
    SEARCH_STR_RET(ATRsp, "CONNECTING", CONNECTING);
    SEARCH_STR_RET(ATRsp, "CLOSE", IP_CLOSE);
    SEARCH_STR_RET(ATRsp, "PDP DEACT", PDP_DEACT);
    return 0;
}

int QC::connectionStateHandle(int check)
{
    int state = ipState();
    IF((state == check), return state;)
    switch (state)
    {
    case IP_INITIAL:
        GSM_CHECK_ERR(connectNetwork("www"))
        break;

    
    case CONNECT_OK:
        GSM_CHECK_ERR(disconnect())
        break;

    case IP_START:
    case IP_GPRSACT:
    case IP_CLOSE:
        GSM_CHECK_ERR(disconnectNetwork())
        break;

    case CONNECTING:
        break;

    default:
        DEBUG_w("UNSTATE: %d", state);
        esp_restart();
        break;
    }

    return state;
}

bool QC::isGPRS()
{
    return modemQuaryVar("AT+QICSGP?", "+QICSGP");
}

int QC::modemQuaryVar(const char *cmd, const char *reply)
{
    // workfor:
    // +QISRVC: 1
    //
    // OK
    char *ptr;
    GSM_CHECK_ERR(AT_CheckReply(cmd, reply))
    PTR_CHECK_RET((ptr = strstr(ATRsp, " ")), false)
    return atoi(ptr);
}

bool QuectelCellular::r_available()
{
    uint32_t tmr = 0;
    uint8_t linesFound = 0;
    uint16_t idx = 0;
    ATRsp[idx] = 0;
    AT_flush();
    gsm_log_req("AT+QIRD=0,1,0,1500");
    _uart->println("AT+QIRD=0,1,0,1500");
    StartTimer(tmr, 1000);
    do
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        while (_uart->available())
        {
            char c = _uart->read();
            IF((c == '\r'), continue;)
            IF((c == '\n'), linesFound++;)
            rxBuffer[idx++] = c;
            rxBuffer[idx] = 0;
        }
    } while (IsTimerRunning(tmr) && (strstr(rxBuffer, "OK") == NULL));
    gsm_log_rsp(String(rxBuffer));
    return ((IsTimerRunning(tmr) && linesFound >= 4)) ? true : false;
}

/*------------------------------------------------------------------------------------------*/

bool QC::getSimPresent()
{
    GSM_CHECK_ERR(AT_CheckReply("AT+QSIMSTAT?", "+QSIMSTAT"))
    const char delimiter[] = ",";
    char *token = strtok(ATRsp, delimiter);
    PTR_CHECK_RET((token = strtok(nullptr, delimiter)), false)
    return token[0] == 0x31;
}

const char *QC::getSimServiceProvider()
{
    char *token = NULL;
    const char delimiter[] = "\"";
    GSM_CHECK_ERR_RET(AT_CheckReply("AT+QSPN?", "+QSPN"), "NULL")
    PTR_CHECK_RET((token = strtok(ATRsp, delimiter)), "NULL")
    PTR_CHECK_RET((token = strtok(NULL, delimiter)), "NULL")
    strcpy(_SrvsProName, token);
    return token;
}

NetworkRegistrationState QC::getNetworkRegistration()
{
    if (AT_WaitFor("AT+CREG?", 1))
    {
        const char *delimiter = ",";
        char *token = strtok(ATRsp, delimiter);
        if (token)
        {
            token = strtok(nullptr, delimiter);
            if (token)
            {
                return (NetworkRegistrationState)(token[0] - 0x30);
            }
        }
    }
    return NetworkRegistrationState::Unknown;
}

bool QC::getNetworkRegistration(uint32_t timeout)
{
    uint32_t tmr;
    StartTimer(tmr, timeout);
    // DEBUG_d();
    do
    {
        // log_printf("\b\r");
        NetworkRegistrationState state = getNetworkRegistration();
        switch (state)
        {
        case NetworkRegistrationState::NotRegistered:
            DEBUG_i("[NETWORK REGISTRATION] - [NOT REGISTERED]");
            break;
        case NetworkRegistrationState::Registered:
            DEBUG_i("[NETWORK REGISTRATION] - [REGISTERED");
            break;
        case NetworkRegistrationState::Searching:
            DEBUG_i("[NETWORK REGISTRATION] - [SEARCHING]");
            break;
        case NetworkRegistrationState::Denied:
            DEBUG_i("[NETWORK REGISTRATION] - [DENIED]");
            break;
        case NetworkRegistrationState::Unknown:
            DEBUG_i("[NETWORK REGISTRATION] - [UNKNOWN]");
            break;
        case NetworkRegistrationState::Roaming:
            DEBUG_i("[NETWORK REGISTRATION] - [ROAMING]");
            break;
        }

        if (state == NetworkRegistrationState::Registered ||
            state == NetworkRegistrationState::Roaming)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (IsTimerRunning(tmr));

    if (IsTimerElapsed(tmr))
    {
        DEBUG_e("Failed ! Network Registration");
        return false;
    }
    return true;
}

bool QC::getIMEI()
{
    GSM_CHECK_ERR(AT_WaitFor("AT+GSN", 1))
    strcpy(_DeviceIMEI, ATRsp);
    DEBUG_d("IMEI: %s", _DeviceIMEI);
    return (bool)strlen(_DeviceIMEI);
}

/*------------------------------------------------------------------------------------------*/

bool QC::AT_CheckReply(const char *command, const char *reply, uint16_t timeout)
{
    GSM_CHECK_ERR(AT_WaitFor(command, 1, timeout))
    return ((strstr(ATRsp, reply) != NULL) ? true : false);
}

bool QC::AT_CheckMultiReply(const char *command, const char *reply, uint16_t timeout)
{
    GSM_CHECK_OK(AT_WaitFor(command, reply, timeout))
    return false;
}

bool QC::AT_WaitFor(const char *command, uint8_t lines, uint16_t timeout)
{
    AT_flush();
    gsm_log_req(command);
    _uart->println(command);
    bool ret = AT_readReply(lines, timeout);
    gsm_log_rsp(String(ATRsp));
    return ret;
}

bool QC::AT_WaitFor(const char *command, const char *reply, uint16_t timeout)
{
    AT_flush();
    gsm_log_req(command);
    _uart->println(command);
    bool ret = AT_readReply(reply, timeout);
    gsm_log_rsp(String(ATRsp));
    return ret;
}

bool QC::AT_readReply(uint8_t lines, uint16_t timeout)
{
    uint32_t tmr = 0;
    uint8_t linesFound = 0;
    uint16_t idx = 0;
    ATRsp[idx] = 0;
    StartTimer(tmr, timeout);
    do
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        while (_uart->available())
        {
            char c = _uart->read();
            if ((c == '\r') || (c == '\n' && idx == 0))
                continue;
            if (c == '\n')
                linesFound++;
            if (linesFound == lines)
                return (IsTimerRunning(tmr));
            ATRsp[idx++] = c;
            ATRsp[idx] = 0;
        }
    } while (IsTimerRunning(tmr) && (linesFound < lines));
    return (IsTimerRunning(tmr));
}

bool QC::AT_readReply(const char *reply, uint16_t timeout)
{
    uint32_t tmr = 0;
    uint16_t idx = 0;
    ATRsp[idx] = 0;
    StartTimer(tmr, timeout);
    do
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        while (_uart->available())
        {
            ATRsp[idx++] = (char)_uart->read();
            ATRsp[idx] = 0;
        }
    } while (IsTimerRunning(tmr) && (strstr(ATRsp, reply) == NULL));
    return (IsTimerRunning(tmr));
}

void QC::AT_flush()
{
    int x;
    while ((x = _uart->available()) > 0)
    {
        // log_printf("! ");
        while (x--)
        {
            _uart->read();
            // log_printf("%c", _uart->read());
        }
        // log_printf("\r\n");
    }
}

/*------------------------------------------------------------------------------------------*/

void QC::gsm_log_req(const char *cmd)
{
    // DEBUG_v("< [%s]", cmd);
}

void QC::gsm_log_rsp(String reply)
{
    // reply.replace('\r', '-'); reply.replace('\n', '_'); DEBUG_v("> [%s]", reply.c_str());
}