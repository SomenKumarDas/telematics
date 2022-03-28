
#pragma once
#include "util.hpp"
#include "ivn.hpp"

class Containers
{
private:
    String msg;
    uint64_t addr = 0;
    char buff[4000];
    std::vector<int> FrameAdrs;
    std::vector<int> Framelens;

public:
    bool wirteFrame(String str)
    {
        if ((addr + str.length()) > 4000)
        {
            addr = 0;
            FrameAdrs.clear();
            Framelens.clear();
        }

        flash.eraseSector(addr);
        memcpy((void *)&buff[addr], (void *)str.c_str(), str.length());

        if (flash.writeCharArray(addr, buff, 4000))
        {
            FrameAdrs.push_back(addr);
            Framelens.push_back(str.length());
            addr += str.length();
            return true;
        }
        return false;
    }

    uint16_t availableFrames()
    {
        return FrameAdrs.size();
    }

    String readFrame()
    {
        msg = "";

        flash.readCharArray(0, buff, 4000);

        if (FrameAdrs.size())
        {
            for (int i = 0; i < Framelens[0]; i++)
            {
                msg += (char)buff[FrameAdrs[0] + i];
            }

            Framelens.erase(Framelens.begin());
            FrameAdrs.erase(FrameAdrs.begin());
            return msg;
        }
        return String();
    }
};

struct ECUPARAMS
{
    String StrData;
    char ECU_ID[40] = "00"; // UUID of ECU
    uint8_t PROTOCOL = 0;
    uint32_t TXID = 0x7FF;
    uint32_t RXID = 0x7FF;
    bool PADING = 0;
    uint16_t P1Min = 111;
    uint16_t P2Max = 222;
    uint16_t VIN_Req = 333;
    uint8_t VIN_StartByte = 66;
    uint8_t VIN_NoofBytes = 77;
    uint16_t SWID_Req = 88;
    uint8_t SWID_StartByte = 99;
    uint8_t SWID_NoofBytes = 111;

    bool setParameters(const char *buf, const char *keyword)
    {
        Preferences Mem;
        Mem.begin("CNFG", false);
        uint8_t cnt = 0;
        char *ptr = strtok(strstr(buf, keyword), ",");
        if (ptr == NULL)
            return false;
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            strcpy(ECU_ID, ptr);
            Mem.putString("ECU_ID", ECU_ID); // debug_i("ECU_ID: %s", ECU_ID);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            PROTOCOL = atoi(ptr);
            Mem.putUChar("PROTOCOL", PROTOCOL); // debug_i("PROTOCOL: %u", PROTOCOL);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            TXID = HexAsciToU32(ptr);
            Mem.putUInt("TXID", TXID); // debug_i("TXID: %x", TXID);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            RXID = HexAsciToU32(ptr);
            Mem.putUInt("RXID", RXID); // debug_i("RXID: %x", RXID);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            PADING = atoi(ptr);
            Mem.putBool("PADING", PADING); // debug_i("PADING: %u", PADING);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            P1Min = atoi(ptr);
            Mem.putUShort("P1Min", P1Min); // debug_i("P1Min: %u", P1Min);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            P2Max = atoi(ptr);
            Mem.putUShort("P2Max", P2Max); // debug_i("P2Max: %u", P2Max);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            VIN_Req = atoi(ptr);
            Mem.putUShort("VIN_Req", VIN_Req); // debug_i("VIN_Req: %u", VIN_Req);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            VIN_StartByte = atoi(ptr);
            Mem.putUChar("VIN_StartByte", VIN_StartByte); // debug_i("VIN_StartByte: %u", VIN_StartByte);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            VIN_NoofBytes = atoi(ptr);
            Mem.putUChar("VIN_NoofBytes", VIN_NoofBytes); // debug_i("VIN_NoofBytes: %u", VIN_NoofBytes);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            SWID_Req = atoi(ptr);
            Mem.putUShort("SWID_Req", SWID_Req); // debug_i("SWID_Req: %u", SWID_Req);
        }
        ptr = strtok(NULL, ",");
        if (ptr != NULL)
        {
            cnt++;
            SWID_StartByte = atoi(ptr);
            Mem.putUChar("SWID_StartByte", SWID_StartByte); // debug_i("SWID_StartByte: %u", SWID_StartByte);
        }
        ptr = strtok(NULL, "#");
        if (ptr != NULL)
        {
            cnt++;
            SWID_NoofBytes = atoi(ptr);
            Mem.putUChar("SWID_NoofBytes", SWID_NoofBytes); // debug_i("SWID_NoofBytes: %u", SWID_NoofBytes);
        }

        Mem.end();
        return (cnt == 13);
    }

    String getParamString()
    {
        StrData = String(ECU_ID) + (",");
        StrData += String(PROTOCOL) + (",");
        StrData += String(TXID, HEX) + (",");
        StrData += String(RXID, HEX) + (",");
        StrData += String(PADING) + (",");
        StrData += String(P1Min) + (",");
        StrData += String(P2Max) + (",");
        StrData += String(VIN_Req) + (",");
        StrData += String(VIN_StartByte) + ",";
        StrData += String(VIN_NoofBytes) + ",";
        StrData += String(SWID_Req) + ",";
        StrData += String(SWID_StartByte) + ",";
        StrData += String(SWID_NoofBytes);
        return StrData;
    }

    void clearAllParameters()
    {
        StrData = "";
        memset((void *)ECU_ID, 0, sizeof(ECU_ID));
        PROTOCOL = 0;
        TXID = 0;
        RXID = 0;
        PADING = 0;
        P1Min = 0;
        P2Max = 0;
        VIN_Req = 0;
        VIN_StartByte = 0;
        VIN_NoofBytes = 0;
        SWID_Req = 0;
        SWID_StartByte = 0;
        SWID_NoofBytes = 0;
    }
};

class DataStruct
{
private:
    uint32_t gpsUpdateTmr;
    uint16_t gpsUpdateMs = 200;
    String IvnData;
    char *fields[20];

    int parse_comma_delimited_str(char *string, char **fields, int max_fields);

public:
    const char *dataKey = "DataStruct";
    Preferences Mem;

    struct rdiag
    {
        uint8_t protocol;
        uint32_t RxId;
        uint32_t TxId;
    } Rdiag;

    struct srv_t
    {
        char IP1[20] = "165.232.184.128"; // Backend Control Server
        uint16_t IP1P = 8900;
        char IP2[20] = "165.232.184.128"; // Emergency Response Server
        uint16_t IP2P = 8901;
    } SRVData;

    struct dev_t
    {
        char deviceType[20] = "FXLPGO001";
        char vendorID[15] = "AUTOPEEPAL";                                                                                    // vendor ID
        char FWVER[20] = "0.0.1";                                                                                            // Firmware version
        char PROTVER[20] = "AIS140";  
        char ssid[20];
        char pswd[20];
        char VEHREGNUM[20];                                                                                   
        char VEHVIN[18];
        uint8_t APK[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};  // AEOL Session Secret Key
        uint8_t VMEK[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00}; // VMEOL Session Secret Key
        uint8_t DCSK[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00}; // DC Session Secret Key
        uint8_t NSK[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};  // Normal Session Secret Key
        bool IGNISTAT;                                                                                                       // 1= Ignition On, 0 = Ignition Off
        bool MPOW;                                                                                                           // Vehicle Battery disconnected
        char MVOLT[10] = "0";                                                                                                // Voltage in Volts.
        char INTVOLT[10] = "0";                                                                                              // Indicates the level of battery
        char BATTPER[10] = "0";                                                                                              // Indication for the internal battery percentage
        char BATTLOWTH[10] = "0";                                                                                            // Indication for the low battery alert generated in percentage
        char EMRGNC[5] = "0";                                                                                                // Status 1= ON, 0 = OFF
        char TAMPLT[5] = "C";                                                                                                // Cover Closed
        char DINPUT[10] = "0";                                                                                               // 4 digital input statuses
        char DOUTPUT[5] = "0";                                                                                               // 2 external digital outputs
        char AINPUT1[5] = "0.0";                                                                                             // Analog input 1 voltage in volts
        char AINPUT2[5] = "0.0";                                                                                             // Analog input 2 voltage in volts
        char OTARSP[30] = "00";                                                                                              // (SERVER1,CFG_GPR,1)
        char RDIAG[20] = "00";
        char MEM1_PER[5] = "0";
        char MEM2_PER[5] = "0";
        uint16_t IVN1INT;
        uint16_t IVN2INT;
        uint16_t IVN3INT;
        uint16_t IVN4INT;
        uint16_t IVN5INT;
        uint8_t VEHECUCNT;
        ECUPARAMS ECUData[5];

    } DEVData;

    struct ais_t
    {
        uint16_t HLTINT;                 // Health Packet Interval
        uint16_t HLTPKT;                 // Health Packet
        uint16_t OVSPDLMT;               // Overspeed Limit
        uint16_t HRSHBRK;                // Harsh Brake Limit
        uint16_t HRSHACL;                // Harsh Acceleration Limit
        uint16_t RSHTURN;                // Rash Turn Limit
        char SOSNUM1[20];
        char SOSNUM2[20];
        uint16_t IGNINT;                 // Ignition ON Interval
        uint16_t IGFINT;                 // Ignition OFF Interval
        uint16_t PANINT;                 // Panic Interval
        uint16_t SLPTM;                  // Sleep Time
        uint8_t BATALT;                  // Internal Battery Alert limit
        char PKTYPE[5] = "NR";           // Specify the packet type
        uint32_t MSGID;                  // Refer, Message ID Table
        char PKTSTAT[5] = "L";           // L=Live or H= History
        uint32_t FRMNo = 0;              // Messages (000001 to 999999)
    } AISData;

    struct gsm_t
    {
        char IMEI[20];
        char NETOPNAME[20];
        uint8_t GSMSIGNAL;
        char APN[20];
        char MCC[5];
        char MNC[5];
        char LAC[5];
        char CELLID[5];
        char NMR1_SIG[5];
        char NMR1_LAC[5];
        char NMR1_CELLID[5];
        char NMR2_SIG[5];
        char NMR2_LAC[5];
        char NMR2_CELLID[5];
        char NMR3_SIG[5];
        char NMR3_LAC[5];
        char NMR3_CELLID[5];
        char NMR4_SIG[5];
        char NMR4_LAC[5];
        char NMR4_CELLID[5];
        char REPNUM[15];

    } GSMData;

    struct gps_t
    {
        char VALID[5];
        char PRVDER[5];
        int FIX = 0;
        int NoSATLT;
        int ALTTD;
        char DATE[10];
        char TIME[10];
        float HEADING;
        float SPEED;
        float PDOP;
        float HDOP;
        char LTTD[20] = "0";
        char LNGTD[20] = "0";
        char LTTDIR[5] = "N";
        char LNGTDIR[5] = "E";
        char DELDIST[10];
        double GLAT[3];
        double GLONG[3];
        int Grad[3];
       int Gkind[3];
       int Gstatus[3];
    } GPSData;

    DataStruct();
    void DataStructInit();
    void _updateGPSData();

    void updateGPSData(char *rawbuf);
    void gpsDataEncode(const char *rawbuf);
    bool checkGeoFence(uint8_t idx);

    String GetIvnFrame(uint8_t idx);
    void setIMEI(const char *data);
};

class LogInPacket
{
private:
    String Packet;

public:
    const char *Packetize();
    bool DePackatize(String &data);
};

class TrackingPacket : private Containers
{
private:
    String Packet;

public:
    const char *Packetize();

    bool pushFrame()
    {
        return wirteFrame(Packet);
    }

    uint16_t avaiableFrame() { return availableFrames(); }

    const char *popFrame()
    {
        return readFrame().c_str();
    }
};

class HealthPacket : private Containers
{
private:
    String Packet;

public:
    const char *Packetize();

    bool pushFrame()
    {
        return wirteFrame(Packet);
    }

    uint16_t avaiableFrame() { return availableFrames(); }

    const char *popFrame()
    {
        return readFrame().c_str();
    }
};

class EmrgncyPacket
{
private:
    String Packet;

public:
    const char *Packetize();
};

extern DataStruct dataStruct;