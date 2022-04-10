#include "packets.hpp"
#include "caniso.hpp"

DataStruct dataStruct;

DataStruct::DataStruct()
{
    // log_v("OPEN - DataStruct");
    // init();
}

void DataStruct::DataStructInit()
{

    Mem.begin("CMD", true);
    strcpy(dataStruct.SRVData.IP1, Mem.getString("IP1", "0").c_str());
    dataStruct.SRVData.IP1P = Mem.getUShort("IP1P");                               // DEBUG_i("[IP1: %s,%u]", dataStruct.SRVData.IP1, dataStruct.SRVData.IP1P);
    strcpy(dataStruct.SRVData.IP2, Mem.getString("IP2", "0").c_str());             //
    dataStruct.SRVData.IP2P = Mem.getUShort("IP2P");                               // DEBUG_i("[IP2: %s,%u]", dataStruct.SRVData.IP2, dataStruct.SRVData.IP2P);
    strcpy(dataStruct.DEVData.VEHREGNUM, Mem.getString("VEHREGNUM", " ").c_str()); // DEBUG_i("[VEHREGNUM: %s]", dataStruct.DEVData.VEHREGNUM);
    dataStruct.AISData.OVSPDLMT = Mem.getUShort("OVSPDLMT");                       // DEBUG_i("[OVSPDLMT: %u]", dataStruct.AISData.OVSPDLMT);
    dataStruct.AISData.HRSHBRK = Mem.getUShort("HRSHBRK");                         // DEBUG_i("[HRSHBRK: %u]", dataStruct.AISData.HRSHBRK);
    dataStruct.AISData.HRSHACL = Mem.getUShort("HRSHACL");                         // DEBUG_i("[HRSHACL: %u]", dataStruct.AISData.HRSHACL);
    dataStruct.AISData.RSHTURN = Mem.getUShort("RSHTURN");                         // DEBUG_i("[RSHTURN: %u]", dataStruct.AISData.RSHTURN);
    strcpy(dataStruct.AISData.SOSNUM1, Mem.getString("SOSNUM1", "0").c_str());     // DEBUG_i("[SOSNUM1: %s]", dataStruct.AISData.SOSNUM1);
    strcpy(dataStruct.AISData.SOSNUM2, Mem.getString("SOSNUM2", "0").c_str());     // DEBUG_i("[SOSNUM2: %s]", dataStruct.AISData.SOSNUM2);
    dataStruct.AISData.IGNINT = Mem.getUShort("IGNINT");                           // DEBUG_i("[IGNINT: %u]", dataStruct.AISData.IGNINT);
    dataStruct.AISData.IGFINT = Mem.getUShort("IGFINT");                           // DEBUG_i("[IGFINT: %u]", dataStruct.AISData.IGFINT);
    dataStruct.AISData.PANINT = Mem.getUShort("PANINT");                           // DEBUG_i("[PANINT: %u]", dataStruct.AISData.PANINT);
    dataStruct.AISData.HLTINT = Mem.getUShort("HLTINT");                           // DEBUG_i("[HLTINT: %u]", dataStruct.AISData.HLTINT);
    strcpy(dataStruct.GSMData.APN, Mem.getString("APN", "0").c_str());             // DEBUG_i("[APN: %s]", dataStruct.GSMData.APN);
    dataStruct.AISData.SLPTM = Mem.getUShort("SLPTM");                             // DEBUG_i("[SLPTM: %u]", dataStruct.AISData.SLPTM);
    dataStruct.AISData.BATALT = Mem.getUShort("BATALT");                           // DEBUG_i("[BATALT: %u]", dataStruct.AISData.BATALT);
    //
    dataStruct.DEVData.baud = Mem.getUChar("baud");         // DEBUG_i("baud: %u", dataStruct.DEVData.baud);
    dataStruct.DEVData.address = Mem.getUChar("address");   // DEBUG_i("address: %u", dataStruct.DEVData.address);
    dataStruct.DEVData.IVN1INT = Mem.getUShort("IVN1INT");  // DEBUG_i("IVN1INT: %u", dataStruct.DEVData.IVN1INT);
    dataStruct.DEVData.IVN2INT = Mem.getUShort("IVN2INT");  // DEBUG_i("IVN2INT: %u", dataStruct.DEVData.IVN2INT);
    dataStruct.DEVData.IVN3INT = Mem.getUShort("IVN3INT");  // DEBUG_i("IVN3INT: %u", dataStruct.DEVData.IVN3INT);
    dataStruct.DEVData.IVN4INT = Mem.getUShort("IVN4INT");  // DEBUG_i("IVN4INT: %u", dataStruct.DEVData.IVN4INT);
    dataStruct.DEVData.IVN5INT = Mem.getUShort("IVN5INT");  // DEBUG_i("IVN5INT: %u", dataStruct.DEVData.IVN5INT);
    IVNs[0].ivnframes[0].FrameID = Mem.getULong("IVN1[0]"); // DEBUG_i("IVN1[0]: %u", IVNs[0].ivnframes[0].FrameID);
    IVNs[0].ivnframes[1].FrameID = Mem.getULong("IVN1[1]"); // DEBUG_i("IVN1[1]: %u", IVNs[0].ivnframes[1].FrameID);
    IVNs[0].ivnframes[2].FrameID = Mem.getULong("IVN1[2]"); // DEBUG_i("IVN1[2]: %u", IVNs[0].ivnframes[2].FrameID);
    IVNs[0].ivnframes[3].FrameID = Mem.getULong("IVN1[3]"); // DEBUG_i("IVN1[3]: %u", IVNs[0].ivnframes[3].FrameID);
    IVNs[0].ivnframes[4].FrameID = Mem.getULong("IVN1[4]"); // DEBUG_i("IVN1[4]: %u", IVNs[0].ivnframes[4].FrameID);
    IVNs[0].ivnframes[1].extd = 0;                          //
    IVNs[0].ivnframes[2].extd = 0;                          //
    IVNs[0].ivnframes[3].extd = 0;                          //
    IVNs[0].ivnframes[4].extd = 0;                          //
    IVNs[1].ivnframes[0].FrameID = Mem.getULong("IVN2[0]"); // DEBUG_i("IVN2[0]: %u", IVNs[1].ivnframes[0].FrameID);
    IVNs[1].ivnframes[1].FrameID = Mem.getULong("IVN2[1]"); // DEBUG_i("IVN2[1]: %u", IVNs[1].ivnframes[1].FrameID);
    IVNs[1].ivnframes[2].FrameID = Mem.getULong("IVN2[2]"); // DEBUG_i("IVN2[2]: %u", IVNs[1].ivnframes[2].FrameID);
    IVNs[1].ivnframes[3].FrameID = Mem.getULong("IVN2[3]"); // DEBUG_i("IVN2[3]: %u", IVNs[1].ivnframes[3].FrameID);
    IVNs[1].ivnframes[4].FrameID = Mem.getULong("IVN2[4]"); // DEBUG_i("IVN2[4]: %u", IVNs[1].ivnframes[4].FrameID);
    IVNs[1].ivnframes[1].extd = 0;                          //
    IVNs[1].ivnframes[2].extd = 0;                          //
    IVNs[1].ivnframes[3].extd = 0;                          //
    IVNs[1].ivnframes[4].extd = 0;                          //
    IVNs[2].ivnframes[0].FrameID = Mem.getULong("IVN3[0]"); // DEBUG_i("IVN3[0]: %u", IVNs[2].ivnframes[0].FrameID);
    IVNs[2].ivnframes[1].FrameID = Mem.getULong("IVN3[1]"); // DEBUG_i("IVN3[1]: %u", IVNs[2].ivnframes[1].FrameID);
    IVNs[2].ivnframes[2].FrameID = Mem.getULong("IVN3[2]"); // DEBUG_i("IVN3[2]: %u", IVNs[2].ivnframes[2].FrameID);
    IVNs[2].ivnframes[3].FrameID = Mem.getULong("IVN3[3]"); // DEBUG_i("IVN3[3]: %u", IVNs[2].ivnframes[3].FrameID);
    IVNs[2].ivnframes[4].FrameID = Mem.getULong("IVN3[4]"); // DEBUG_i("IVN3[4]: %u", IVNs[2].ivnframes[4].FrameID);
    IVNs[2].ivnframes[1].extd = 0;                          //
    IVNs[2].ivnframes[2].extd = 0;                          //
    IVNs[2].ivnframes[3].extd = 0;                          //
    IVNs[2].ivnframes[4].extd = 0;                          //
    IVNs[3].ivnframes[0].FrameID = Mem.getULong("IVN4[0]"); // DEBUG_i("IVN4[0]: %u", IVNs[3].ivnframes[0].FrameID);
    IVNs[3].ivnframes[1].FrameID = Mem.getULong("IVN4[1]"); // DEBUG_i("IVN4[1]: %u", IVNs[3].ivnframes[1].FrameID);
    IVNs[3].ivnframes[2].FrameID = Mem.getULong("IVN4[2]"); // DEBUG_i("IVN4[3]: %u", IVNs[3].ivnframes[2].FrameID);
    IVNs[3].ivnframes[3].FrameID = Mem.getULong("IVN4[3]"); // DEBUG_i("IVN4[3]: %u", IVNs[3].ivnframes[3].FrameID);
    IVNs[3].ivnframes[4].FrameID = Mem.getULong("IVN4[4]"); // DEBUG_i("IVN4[4]: %u", IVNs[3].ivnframes[4].FrameID);
    IVNs[3].ivnframes[1].extd = 0;                          //
    IVNs[3].ivnframes[2].extd = 0;                          //
    IVNs[3].ivnframes[3].extd = 0;                          //
    IVNs[3].ivnframes[4].extd = 0;                          //
    IVNs[4].ivnframes[0].FrameID = Mem.getULong("IVN5[0]"); // DEBUG_i("IVN5[0]: %u", IVNs[4].ivnframes[0].FrameID);
    IVNs[4].ivnframes[1].FrameID = Mem.getULong("IVN5[1]"); // DEBUG_i("IVN5[1]: %u", IVNs[4].ivnframes[1].FrameID);
    IVNs[4].ivnframes[2].FrameID = Mem.getULong("IVN5[2]"); // DEBUG_i("IVN5[2]: %u", IVNs[4].ivnframes[2].FrameID);
    IVNs[4].ivnframes[3].FrameID = Mem.getULong("IVN5[3]"); // DEBUG_i("IVN5[3]: %u", IVNs[4].ivnframes[3].FrameID);
    IVNs[4].ivnframes[4].FrameID = Mem.getULong("IVN5[4]"); // DEBUG_i("IVN5[4]: %u", IVNs[4].ivnframes[4].FrameID);
    IVNs[4].ivnframes[1].extd = 0;
    IVNs[4].ivnframes[2].extd = 0;
    IVNs[4].ivnframes[3].extd = 0;
    IVNs[4].ivnframes[4].extd = 0;

    switch (dataStruct.DEVData.baud)
    {
    case 0:
        CAN_SetBaud(CAN_SPEED_250KBPS);
        break;
    case 1:
        CAN_SetBaud(CAN_SPEED_500KBPS);
        break;
    case 2:
        CAN_SetBaud(CAN_SPEED_1000KBPS);
        break;
    }

    Mem.end();
}

void DataStruct::setIMEI(const char *data)
{
    strcpy(GSMData.IMEI, data);
}

String DataStruct::GetIvnFrame(uint8_t idx)
{
    IvnData = "";

    if (IVNs[idx].ivnframes[0].FrameID > 0)
    {
        IvnData += String(IVNs[idx].ivnframes[0].FrameID, HEX);
        IvnData += ":";
        IvnData += String(IVNs[idx].ivnframes[0].DataBuff);
        IvnData += ",";
    }
    else
    {
        IvnData += ",";
    }

    if (IVNs[idx].ivnframes[1].FrameID > 0)
    {
        IvnData += String(IVNs[idx].ivnframes[1].FrameID, HEX);
        IvnData += ":";
        IvnData += String(IVNs[idx].ivnframes[1].DataBuff);
        IvnData += ",";
    }
    else
    {
        IvnData += ",";
    }

    if (IVNs[idx].ivnframes[2].FrameID > 0)
    {
        IvnData += String(IVNs[idx].ivnframes[2].FrameID, HEX);
        IvnData += ":";
        IvnData += String(IVNs[idx].ivnframes[2].DataBuff);
        IvnData += ",";
    }
    else
    {
        IvnData += ",";
    }

    if (IVNs[idx].ivnframes[3].FrameID > 0)
    {
        IvnData += String(IVNs[idx].ivnframes[3].FrameID, HEX);
        IvnData += ":";
        IvnData += String(IVNs[idx].ivnframes[3].DataBuff);
        IvnData += ",";
    }
    else
    {
        IvnData += ",";
    }

    if (IVNs[idx].ivnframes[4].FrameID > 0)
    {
        IvnData += String(IVNs[idx].ivnframes[4].FrameID, HEX);
        IvnData += ":";
        IvnData += String(IVNs[idx].ivnframes[4].DataBuff);
    }
    else
    {
        IvnData += ",";
    }
    return IvnData;
}

// [GPS] ============================================================== >>

int DataStruct::parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
    int i = 0;
    fields[i++] = string;
    while ((i < max_fields) && NULL != (string = strchr(string, ',')))
    {
        *string = '\0';
        fields[i++] = ++string;
    }
    return --i;
}

double DataStruct::GpsEncodingToDegrees(char *gpsencoding)
{
    double a = strtod(gpsencoding, 0);
    double d = (int)a / 100;
    a -= d * 100;
    return d + (a / 60);
}

double DataStruct::deg2rad(double deg)
{
  return (deg * pi / 180);
}

double DataStruct::rad2deg(double rad)
{
  return (rad * 180 / pi);
}

double DataStruct::distance(double lat1, double lon1, double lat2, double lon2)
{
  double theta, dist;
  if ((lat1 == lat2) && (lon1 == lon2))
  {
    return 0;
  }
  else
  {
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;
    return (dist);
  }
}

void DataStruct::updateGPSData(char *rawbuf)
{
    char *tok = (strtok(rawbuf, "\r\n"));
    if (tok == NULL)
        return;
    do
    {
        if (strstr(tok, "RMC"))
        {
            parse_comma_delimited_str(tok, fields, 12);
            strcpy(GPSData.TIME, fields[1]);                        // DEBUG_v("[Time:       %s]", GPSData.TIME);
            strcpy(GPSData.VALID, fields[2]);                       // DEBUG_v("[Valid:      %s]", GPSData.VALID);
            GPSData.FIX = (GPSData.VALID[0] == 'A') ? true : false; // DEBUG_v("[Fix:        %d]", GPSData.FIX);
            GPSData.Latitude = GpsEncodingToDegrees(fields[3]);     // DEBUG_v("[Latitude:   %2.6lf]", GPSData.Latitude);
            strcpy(GPSData.LTTDIR, fields[4]);                      // DEBUG_v("[N/S:        %s]", GPSData.LTTDIR);
            GPSData.longitude = GpsEncodingToDegrees(fields[5]);    // DEBUG_v("[longitude:   %2.6lf]", GPSData.longitude);
            strcpy(GPSData.LNGTDIR, fields[6]);                     // DEBUG_v("[E/W:        %s]", GPSData.LNGTDIR);
            GPSData.SPEED = (float)atof(fields[7]);                 // DEBUG_v("[Speed:      %f]", GPSData.SPEED);
            strcpy(GPSData.DATE, fields[9]);                        // DEBUG_v("[Date:       %s]", GPSData.DATE);
        }
        else if (strstr(tok, "GSA"))
        {
            parse_comma_delimited_str(tok, fields, 18);
            GPSData.PDOP = (float)atof(fields[15]); // DEBUG_v("[PDOP:       %f]", GPSData.PDOP);
            GPSData.HDOP = (float)atof(fields[16]); // DEBUG_v("[HDOP:       %f]", GPSData.HDOP);
        }
        else if (strstr(tok, "GGA"))
        {
            parse_comma_delimited_str(tok, fields, 14);
            GPSData.FIX = atoi(fields[6]);     // DEBUG_v("[Fix:        %d]", GPSData.FIX);
            GPSData.NoSATLT = atoi(fields[7]); // DEBUG_v("[Satellites: %d]", GPSData.NoSATLT);
            GPSData.ALTTD = atoi(fields[9]);   // DEBUG_v("[Altitude:   %d]", GPSData.ALTTD);
        }
        else
        {
            // DEBUG_w("[UNH: %s]", tok);
        }
    } while ((tok = strtok(NULL, "\r\n")) != NULL);
}

bool DataStruct::checkGeoFence(uint8_t idx)
{
    IF(!GPSData.Gstatus[idx], return false;)
    int dist = (int)distance(GPSData.Latitude, GPSData.longitude, GPSData.GLAT[idx], GPSData.GLONG[idx]);
    return ( (GPSData.Gkind[idx]) ? (dist > GPSData.Grad[idx]) : (dist < GPSData.Grad[idx]));
}

//===================================================================== >>

const char *LogInPacket::Packetize()
{
    Packet = "$";
    Packet += String(dataStruct.DEVData.deviceType) + "-";
    Packet += String("LIN") + ",";
    Packet += String(dataStruct.DEVData.vendorID) + ",";
    Packet += String(dataStruct.DEVData.VEHREGNUM) + ",";
    Packet += String(dataStruct.GSMData.IMEI) + ",";
    Packet += String(dataStruct.DEVData.FWVER) + ",";
    Packet += String(dataStruct.DEVData.PROTVER) + ",";
    Packet += String(dataStruct.GPSData.Latitude) + ",";
    Packet += String(dataStruct.GPSData.LTTDIR) + ",";
    Packet += String(dataStruct.GPSData.longitude) + ",";
    Packet += String(dataStruct.GPSData.LTTDIR);
    Packet += "*50";
    // DEBUG_d("LogIn Packet: %s", Packet.c_str());

    return Packet.c_str();
}

//====================================================================================================

const char *TrackingPacket::Packetize()
{
    Packet = "$";
    Packet += String(dataStruct.DEVData.deviceType) + "-";
    Packet += String("TP") + ",";
    Packet += String(dataStruct.DEVData.vendorID) + ",";    // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.FWVER) + ",";       // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.AISData.PKTYPE) + ",";      // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.AISData.MSGID++) + ",";     // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.AISData.PKTSTAT) + ",";     // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.IMEI) + ",";        // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.VEHREGNUM) + ",";   // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GPSData.FIX) + ",";         // log_v("FIX -> %d", dataStruct.GPSData.FIX);
    Packet += String(dataStruct.GPSData.DATE) + ",";        // log_v("DATE -> %s", dataStruct.GPSData.DATE);
    Packet += String(dataStruct.GPSData.TIME) + ",";        // log_v("TIME -> %s", GPSData.TIME);
    Packet += String(dataStruct.GPSData.Latitude) + ",";    // log_v("LTTD -> %s", GPSData.LTTD);
    Packet += String(dataStruct.GPSData.LTTDIR) + ",";      // log_v("LTTDIR -> %s", GPSData.LTTDIR);
    Packet += String(dataStruct.GPSData.longitude) + ",";   // log_v("LNGTD -> %s", GPSData.LNGTD);
    Packet += String(dataStruct.GPSData.LNGTDIR) + ",";     // log_v("LNGTDIR -> %s", GPSData.LNGTDIR);
    Packet += String(dataStruct.GPSData.SPEED) + ",";       // log_v("SPEED -> %s", GPSData.SPEED);
    Packet += String(dataStruct.GPSData.HEADING) + ",";     // log_v("HEADING -> %s", GPSData.HEADING);
    Packet += String(dataStruct.GPSData.NoSATLT) + ",";     // log_v("NoSATLT -> %d", GPSData.NoSATLT);
    Packet += String(dataStruct.GPSData.ALTTD) + ",";       // log_v("ALTTD -> %d", GPSData.ALTTD);
    Packet += String(dataStruct.GPSData.PDOP) + ",";        // log_v("PDOP -> %s", GPSData.PDOP);
    Packet += String(dataStruct.GPSData.HDOP) + ",";        // log_v("HDOP -> %s", GPSData.HDOP);
    Packet += String(dataStruct.GSMData.NETOPNAME) + ",";   // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.IGNISTAT) + ",";    // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.MPOW) + ",";        // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.MVOLT) + ",";       // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.INTVOLT) + ",";     // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.EMRGNC) + ",";      // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.DEVData.TAMPLT) + ",";      // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.GSMSIGNAL) + ",";   // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.MCC) + ",";         // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.MNC) + ",";         // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.LAC) + ",";         // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.CELLID) + ",";      // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.NMR1_SIG) + ",";    // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.NMR1_LAC) + ",";    // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.NMR1_CELLID) + ","; // log_v("vendorID -> %s", DEVData.vendorID);
    Packet += String(dataStruct.GSMData.NMR2_SIG) + ",";
    Packet += String(dataStruct.GSMData.NMR2_LAC) + ",";
    Packet += String(dataStruct.GSMData.NMR2_CELLID) + ",";
    Packet += String(dataStruct.GSMData.NMR3_SIG) + ",";
    Packet += String(dataStruct.GSMData.NMR3_LAC) + ",";
    Packet += String(dataStruct.GSMData.NMR3_CELLID) + ",";
    Packet += String(dataStruct.GSMData.NMR4_SIG) + ",";
    Packet += String(dataStruct.GSMData.NMR4_LAC) + ",";
    Packet += String(dataStruct.GSMData.NMR4_CELLID) + ",";
    Packet += String(dataStruct.DEVData.DINPUT) + ",";
    Packet += String(dataStruct.DEVData.DOUTPUT) + ",";
    Packet += String((dataStruct.AISData.FRMNo == 999999 ? dataStruct.AISData.FRMNo = 0 : dataStruct.AISData.FRMNo++)) + " ,";
    Packet += String(dataStruct.DEVData.AINPUT1) + ",";
    Packet += String(dataStruct.DEVData.AINPUT2) + ",";
    Packet += String(dataStruct.GPSData.DELDIST) + ",";
    Packet += String(dataStruct.DEVData.OTARSP);
    Packet += "*1E";
    // DEBUG_d("[TP] [%s]", Packet.c_str());
    return Packet.c_str();
}

//====================================================================================================

const char *HealthPacket::Packetize()
{
    Packet = "$";
    Packet += String(dataStruct.DEVData.deviceType) + "-";
    Packet += String("HMP") + ",";
    Packet += String(dataStruct.DEVData.vendorID) + ",";
    Packet += String(dataStruct.DEVData.FWVER) + ",";
    Packet += String(dataStruct.GSMData.IMEI) + ",";
    Packet += String(dataStruct.DEVData.BATTPER) + ",";
    Packet += String(dataStruct.DEVData.BATTLOWTH) + ",";
    Packet += String(dataStruct.memoryUsed(1)) + "%,";
    Packet += String(dataStruct.memoryUsed(2)) + "%,";
    Packet += String(dataStruct.AISData.IGNINT) + ",";
    Packet += String(dataStruct.AISData.IGFINT) + ",";
    Packet += String(dataStruct.DEVData.DINPUT) + ",";
    Packet += String(dataStruct.DEVData.AINPUT1) + ",";
    Packet += String(dataStruct.DEVData.AINPUT2);
    Packet += "*5A";
    // DEBUG_d("[HMP] [%s]", Packet.c_str());
    return Packet.c_str();
}

//====================================================================================================

const char *EmrgncyPacket::Packetize()
{
    Packet = "$";
    Packet += String(dataStruct.DEVData.deviceType) + "-";
    Packet += String("EPB") + ",";
    Packet += String(dataStruct.DEVData.vendorID) + ",";
    Packet += "EMR,";
    Packet += String(dataStruct.GSMData.IMEI) + ",";
    Packet += String(dataStruct.AISData.PKTYPE) + ",";
    Packet += String(dataStruct.GPSData.DATE) + ",";
    Packet += String(dataStruct.GPSData.VALID) + ",";
    Packet += String(dataStruct.GPSData.Latitude) + ",";
    Packet += String(dataStruct.GPSData.LTTDIR) + ",";
    Packet += String(dataStruct.GPSData.longitude) + ",";
    Packet += String(dataStruct.GPSData.LNGTDIR) + ",";
    Packet += String(dataStruct.GPSData.ALTTD) + ",";
    Packet += String(dataStruct.GPSData.SPEED) + ",";
    Packet += String(dataStruct.GPSData.DELDIST) + ",";
    Packet += String(dataStruct.GSMData.NETOPNAME) + ",";
    Packet += String(dataStruct.DEVData.VEHREGNUM) + ",";
    Packet += String(dataStruct.AISData.SOSNUM1) + ",";
    Packet += String(dataStruct.AISData.SOSNUM2) + ",";
    Packet += "*4B";
    // DEBUG_d("[EMR PACK] [%s]", Packet.c_str());
    return Packet.c_str();
}

//====================================================================================================