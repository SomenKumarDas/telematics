#include "ivn.hpp"
#include "caniso.hpp"

ivn_t IVNs[IVN_ARR_SIZE];

void handle_ivn_rx(CAN_frame_t *canFrame)
{
    // Serial.printf("RX FrameID: %x\r\n", canFrame->MsgID);
//===============================================================================================
 
    if(IVNs[0].ivnframes[0].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[0].ivnframes[0].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[0].ivnframes[1].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[0].ivnframes[1].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[0].ivnframes[2].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[0].ivnframes[2].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[0].ivnframes[3].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[0].ivnframes[3].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[0].ivnframes[4].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[0].ivnframes[4].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
 
//===============================================================================================

    else if(IVNs[1].ivnframes[0].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[1].ivnframes[0].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[1].ivnframes[1].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[1].ivnframes[1].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[1].ivnframes[2].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[1].ivnframes[2].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[1].ivnframes[3].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[1].ivnframes[3].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[1].ivnframes[4].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[1].ivnframes[4].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }

//===============================================================================================

    else if(IVNs[2].ivnframes[0].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[2].ivnframes[0].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[2].ivnframes[1].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[2].ivnframes[1].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[2].ivnframes[2].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[2].ivnframes[2].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[2].ivnframes[3].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[2].ivnframes[3].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[2].ivnframes[4].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[2].ivnframes[4].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    //===============================================================================================

    else if(IVNs[3].ivnframes[0].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[3].ivnframes[0].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[3].ivnframes[1].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[3].ivnframes[1].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[3].ivnframes[2].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[3].ivnframes[2].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[3].ivnframes[3].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[3].ivnframes[3].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[3].ivnframes[4].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[3].ivnframes[4].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    //===============================================================================================

    else if(IVNs[4].ivnframes[0].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[4].ivnframes[0].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[4].ivnframes[1].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[4].ivnframes[1].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[4].ivnframes[2].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[4].ivnframes[2].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[4].ivnframes[3].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[4].ivnframes[3].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
    else if(IVNs[4].ivnframes[4].FrameID == canFrame->MsgID)
    {
        // log_v("IVN FrameID: %x", canFrame->MsgID);
        sprintf(IVNs[4].ivnframes[4].DataBuff, "%02x%02x%02x%02x%02x%02x%02x%02x",
        canFrame->data.u8[0],canFrame->data.u8[1],canFrame->data.u8[2],canFrame->data.u8[3],
        canFrame->data.u8[4],canFrame->data.u8[5],canFrame->data.u8[6],canFrame->data.u8[7]);
    }
}

