#pragma once
#include <Arduino.h>
#include "CAN.h"


#define IVN_ARR_SIZE 5
#define IVN_FRAME_ARR_SIZE 5
#define WaitTIMEOUTms 500

struct ivnFrame_t
{
    uint32_t FrameID;
    bool extd;
    char DataBuff[20];
};

struct ivn_t
{
    ivnFrame_t ivnframes[IVN_FRAME_ARR_SIZE];
};


extern ivn_t IVNs[IVN_ARR_SIZE];

void test_setup();

void update_All_IVN();
void update_ivn_by_index(uint8_t idx);
void handle_ivn_rx(CAN_frame_t *canFrame);