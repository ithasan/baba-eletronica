/*********************************************************************
 *
 *                  PHY for the UBEC 2400
 *
 *********************************************************************
 * FileName:        zPHY_UZ2400.c
 * Dependencies:
 * Processor:       PIC18F
 * Complier:        MCC18 v3.00 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright © 2004-2007 Microchip Technology Inc.  All rights reserved.
 *
 * Microchip licenses to you the right to use, copy and distribute Software 
 * only when embedded on a Microchip microcontroller or digital signal 
 * controller and used with a Microchip radio frequency transceiver, which 
 * are integrated into your product or third party product (pursuant to the 
 * sublicense terms in the accompanying license agreement).  You may NOT 
 * modify or create derivative works of the Software.  
 *
 * If you intend to use this Software in the development of a product for 
 * sale, you must be a member of the ZigBee Alliance.  For more information, 
 * go to www.zigbee.org.
 *
 * You should refer to the license agreement accompanying this Software for 
 * additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY 
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR 
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED 
 * UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF 
 * WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR 
 * EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, 
 * PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF 
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY 
 * THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER 
 * SIMILAR COSTS.
 *
 *
 * Author               Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * DF/KO                07/20/06 Microchip ZigBee Stack v1.0-3.5.1
 *                               Moved interrupt
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/

#include "ZigbeeTasks.h"
#include "Zigbee.def"
#include "zPHY.h"
#include "zMAC.h"
#include "MSPI.h"
#include "generic.h"
#include "sralloc.h"
#include "console.h"

#if(RF_CHIP == UZ2400)

extern MAC_TASKS_PENDING macTasksPending;
extern CURRENT_PACKET currentPacket;

PHY_PIB phyPIB;

void UserInterruptHandler(void);

typedef union _PHYPendingTasks
{
    struct
    {
        unsigned int PLME_SET_TRX_STATE_request_task :1;
        unsigned int PHY_RX :1;
    } bits;
    BYTE Val;
} PHY_PENDING_TASKS;

typedef struct _RX_DATA
{
    unsigned int size :7;
    unsigned int inUse :1;
} RX_DATA;

volatile RX_DATA RxData;
volatile TX_STAT TxStat;
volatile INT_SAVE IntStatus;
BYTE TRXCurrentState;
volatile PHY_PENDING_TASKS PHYTasksPending;
SAVED_BITS savedBits;
volatile MAC_FRAME_CONTROL pendingAckFrameControl;

#if (RX_BUFFER_SIZE > 256)
    extern volatile WORD RxWrite;
    extern WORD RxRead;
#else
    extern volatile BYTE RxWrite;
    extern BYTE RxRead;
#endif

extern BYTE RxBuffer[RX_BUFFER_SIZE];

static unsigned char GIEH_backup;




BYTE PHYGet(void)
{
    BYTE toReturn;

    toReturn = RxBuffer[RxRead++];

    if(RxRead == RX_BUFFER_SIZE)
    {
        RxRead = 0;
    }

    return toReturn;
}

BYTE PHYHasBackgroundTasks(void)
{
    //PHY might want to check to verify that we are not in the middle of
    // a transmission before allowing the user to turn off the TRX
    return PHYTasksPending.Val;
}

void PHYInit(void)
{
        BYTE i;
        PHY_RESETn_TRIS = 0;

        RxData.inUse = 0;
        PHYTasksPending.Val = 0;

        CLRWDT();

        PHY_RESETn = 0;
        for(i=0;i<255;i++){}
        PHY_RESETn = 1;
        for(i=0;i<255;i++){}

        /* flush the RX fifo */
        PHYSetShortRAMAddr(RXFLUSH,0x01);

        /* Program the short MAC Address, 0xffff */
        //<TODO> this step may not be required
        PHYSetShortRAMAddr(SADRL,0xFF);
        PHYSetShortRAMAddr(SADRH,0xFF);
        PHYSetShortRAMAddr(PANIDL,0xFF);
        PHYSetShortRAMAddr(PANIDH,0xFF);

        /* Program Long MAC Address, 0xFFFFFFFFFFFFFFFF*/
        PHYSetShortRAMAddr(EADR0, MAC_LONG_ADDR_BYTE0);
        PHYSetShortRAMAddr(EADR1, MAC_LONG_ADDR_BYTE1);
        PHYSetShortRAMAddr(EADR2, MAC_LONG_ADDR_BYTE2);
        PHYSetShortRAMAddr(EADR3, MAC_LONG_ADDR_BYTE3);
        PHYSetShortRAMAddr(EADR4, MAC_LONG_ADDR_BYTE4);
        PHYSetShortRAMAddr(EADR5, MAC_LONG_ADDR_BYTE5);
        PHYSetShortRAMAddr(EADR6, MAC_LONG_ADDR_BYTE6);
        PHYSetShortRAMAddr(EADR7, MAC_LONG_ADDR_BYTE7);

        /* program the RF and Baseband Register */
        PHYSetLongRAMAddr(RFCTRL4,0x02);

        /* Enable the RX */
        PHYSetLongRAMAddr(RFRXCTRL,0x01);

        /* setup */
        //PHYSetLongRAMAddr(RFCTRL1,0x00);
        PHYSetLongRAMAddr(RFCTRL2,0x80);


/*   PA_LEVEL determiens output power of transciever
        Default output power is 0 dBm. Summation of “large” and “small” tuning decreases
        output power
    PA_LEVEL:
        [7:6] -> large scale tuning
              00: 0 dB
              01: -10 dB
              10: -20 dB
              11: -30 dB
        [5:3] -> small scale tuning
              000: 0 dB
              001: -1.25 dB
              010: -2.5 dB
              011: -3.75 dB
              100: -5 dB
              101: -6.25 dB
              110: -7.5 dB
              111: -8.75 dB
        [2:0] -> 000
*/
        PHYSetLongRAMAddr(RFCTRL3,PA_LEVEL);


        /* program RSSI ADC with 2.5 MHz clock */
        PHYSetLongRAMAddr(RFCTRL6,0x04);
        PHYSetLongRAMAddr(RFCTRL7,0b00000000);

        /* Program CCA mode using RSSI */
        PHYSetShortRAMAddr(BBREG2,0x80);
        /* Enable the packet RSSI */
        PHYSetShortRAMAddr(BBREG6,0x40);
        /* Program CCA, RSSI threshold values */
        PHYSetShortRAMAddr(RSSITHCCA,0x60);

        #ifdef I_AM_FFD
            PHYSetShortRAMAddr(ACKTMOUT,0xB9);
        #endif

        // Set interrupt mask
        PHYSetShortRAMAddr(0x32, 0xF6);

        do
        {
            i = PHYGetLongRAMAddr(RFSTATE);
        }
        while((i&0xA0) != 0xA0);

        TMR1IE = 0;

        TMR1ON = 1;

}

ZIGBEE_PRIMITIVE PHYTasks(ZIGBEE_PRIMITIVE inputPrimitive)
{
    if(inputPrimitive == NO_PRIMITIVE)
    {
        /* manage background tasks here */
        if(ZigBeeStatus.flags.bits.bRxBufferOverflow == 1)
        {
            ConsolePutROMString((rom char*)"RxBufferOverflow!\r\n");
        }
        if(ZigBeeTxUnblocked)   //(TxFIFOControl.bFIFOInUse==0)
        {

            if(PHYTasksPending.bits.PHY_RX)
            {
                BYTE packetSize;

                if(CurrentRxPacket != NULL)
                {
                    return NO_PRIMITIVE;
                }

                packetSize = RxBuffer[RxRead];

                params.PD_DATA_indication.psdu = SRAMalloc(packetSize);

                if(params.PD_DATA_indication.psdu == NULL)
                {
                    return NO_PRIMITIVE;
                }

                /* save the packet head somewhere so that it can be freed later */
                if(CurrentRxPacket == NULL)
                {
                    CurrentRxPacket = params.PD_DATA_indication.psdu;

                    params.PD_DATA_indication.psduLength = packetSize;
                    RxRead++;
                    if(RxRead == RX_BUFFER_SIZE)
                    {
                        RxRead = 0;
                    }

                    while(packetSize--)
                    {
                        *params.PD_DATA_indication.psdu++ = PHYGet();
                    }

                    /* reset the psdu to the head of the alloc-ed RAM, just happens to me CurrentRxPacket */
                    params.PD_DATA_indication.psdu = CurrentRxPacket;

                    /* disable interrupts before checking to see if this was the
                        last packet in the FIFO so that if we get a packet(interrupt) after the check,
                        but before the clearing of the bit then the new indication will not
                        get cleared */

                    savedBits.bGIEH = GIEH;
                    GIEH = 0;

                    if(RxRead == RxWrite)
                    {
                        PHYTasksPending.bits.PHY_RX = 0;
                    }

                    GIEH = savedBits.bGIEH;

                    return PD_DATA_indication;
                }
            }
        }
        else
        {
            return NO_PRIMITIVE;
        }

    }
    else
    {
        /* handle primitive here */
        switch(inputPrimitive)
        {
// Not necessary for operation
//            case PD_DATA_request:
//                break;
//CSMA-CA will be automatic
//           case PLME_CCA_request:
//                break;
//            case PLME_ED_request:
  //              break;
/*          User will modify these directly
            case PLME_SET_request:
                break;
            case PLME_GET_request:
                break;  */
/*          The UBEC part switches states automatically
            case PLME_SET_TRX_STATE_request:
                    */
        }
    }
}

void PHYSetLongRAMAddr(WORD address, BYTE value)
{
    IntStatus.CCP2IntF = INT0IE;
    INT0IE = 0;
    RC0 = 0;
    NOP();
    SPIPut((((BYTE)(address>>3))&0b01111111)|0x80);
    SPIPut((((BYTE)(address<<5))&0b11100000)|0x10);
    SPIPut(value);
    NOP();
    RC0 = 1;
    INT0IE = IntStatus.CCP2IntF;
}

void PHYSetShortRAMAddr(BYTE address, BYTE value)
{
    IntStatus.CCP2IntF = INT0IE;
    INT0IE = 0;
    RC0 = 0;
    NOP();
    SPIPut(((address<<1)&0b01111111)|0x01);
    SPIPut(value);
    NOP();
    RC0 = 1;
    INT0IE = IntStatus.CCP2IntF;
}

BYTE PHYGetShortRAMAddr(BYTE address)
{
    BYTE toReturn;
    IntStatus.CCP2IntF = INT0IE;
    INT0IE = 0;
    RC0 = 0;
    NOP();
    SPIPut((address<<1)&0b01111110);
    toReturn = SPIGet();
    NOP();
    RC0 = 1;
    INT0IE = IntStatus.CCP2IntF;
    return toReturn;
}

BYTE PHYGetLongRAMAddr(WORD address)
{
    BYTE toReturn;
    IntStatus.CCP2IntF = INT0IE;
    INT0IE = 0;
    RC0 = 0;
    NOP();
    SPIPut(((address>>3)&0b01111111)|0x80);
    SPIPut(((address<<5)&0b11100000));
    toReturn = SPIGet();
    NOP();
    RC0 = 1;
    INT0IE = IntStatus.CCP2IntF;
    return toReturn;
}

//******************************************************************************
// ISR
//******************************************************************************

#pragma interruptlow HighISR
void HighISR (void)
{
    BYTE    CheckInterrupt;
    BYTE    TxStatus;

    // Check if our interrupt came from the INT pin on the UBEC part
    if(INT0IF)
    {
//  if (CCP2IF)
        if(INT0IE)
        {
            INT0IF = 0;
//      if (CCP2IE)
            // Clear interrupt
//          CCP2IF = 0;

            // Reading this interrupt register will clear the interrupts
            CheckInterrupt = PHYGetShortRAMAddr(0x31);

            if (CheckInterrupt & 0x01)
            {
                TxStat.finished = 1;
                //Transmit FIFO release interrupt
                // The release status will be set to ok if an ACK was required and
                // was received successfully

                TxStatus = PHYGetShortRAMAddr(0x24);
                if (TxStatus & 0x01)
                {
                    //Failure- No ack back
                    TxStat.success = 0;
                }
                else
                {
                    //success
                    TxStat.success = 1;
                }
            }

            if (CheckInterrupt & 0x08)
            {
                BYTE_VAL ack_status;
                BYTE count;
                BYTE counter;
                BYTE_VAL w;

                #if (RX_BUFFER_SIZE > 256)
                        #error "Rx buffer must be <= 256"
                #else
                        #define BUFFER_CAST BYTE
                        BYTE RxBytesRemaining;
                        BYTE OldRxWrite;
                #endif

                ack_status.Val = 0;
                count = 0;
                counter = 0x00;

                // Receive ok interrupt
                if(RxData.inUse == 0)
                {
                    RxData.size = PHYGetLongRAMAddr ((WORD)(0x300 + counter++));
                    RxData.inUse=1;
                }

                OldRxWrite = RxWrite;
                if(RxWrite < RxRead)
                {
                    RxBytesRemaining = (BUFFER_CAST)(RxRead - RxWrite - 1);
                }
                else
                {
                    RxBytesRemaining = (BUFFER_CAST)(RX_BUFFER_SIZE - 1 - RxWrite + RxRead);
                }

                w.Val = RxData.size;


                /* this is less then because we need one extra byte for the length (which worst case would make it equivent to less then or equal to )*/
                if(w.Val < RxBytesRemaining)
                {
                    MAC_FRAME_CONTROL mfc;

                    /* there is room in the buffer */
                    RxData.inUse = 0;

                    /* add the packet */
                    RxBuffer[RxWrite++]=w.Val;

                    if(RxWrite==RX_BUFFER_SIZE)
                    {
                        RxWrite = 0;
                    }

                    while(w.Val--)
                    {
                        //Note: I am counting on the fact that RxWrite doesn't get incremented here anymore such that the ACK packet doesn't get written into the Buffer and the RxWrite doesn't get modified.
                        RxBuffer[RxWrite] = PHYGetLongRAMAddr ((WORD)(0x300 + counter++));

                        if(count==0)
                        {
                            //if the frame control indicates that this packet is an ack

                            mfc.word.byte.LSB=RxBuffer[RxWrite];

                            if(mfc.bits.FrameType == 0b010)
                            {
                                //it was an ack then set the ack_status.bits.b0 to 1 showing it was an ack
                                ack_status.bits.b0 = 1;
                                //ConsolePut('@');
                            }
                        }
                        else if(count==2)
                        {
                            //if we are reading the sequence number and the packet was an ack
                            if(ack_status.bits.b0)
                            {

                                if ((macTasksPending.bits.packetPendingAck) &&
                                    (RxBuffer[RxWrite] == currentPacket.sequenceNumber))
                                {
                                    // If this is the ACK we've been waiting for, set the flag to
                                    // send up the confirm and save the Frame Control.
                                    macTasksPending.bits.bSendUpMACConfirm = 1;
                                    pendingAckFrameControl = mfc;
                                }
                                RxWrite = OldRxWrite;
                                goto DoneReceivingPacket;
                            }
                        }

                        count++;
                        RxWrite++;
                        //roll buffer if required
                        if(RxWrite==RX_BUFFER_SIZE)
                        {
                            RxWrite = 0;
                        }
                    }

                    if(RxWrite==0)
                    {
                        w.Val = RxBuffer[RX_BUFFER_SIZE-1];
                    }
                    else
                    {
                        w.Val = RxBuffer[RxWrite - 1];
                    }

                    if(PHYGetShortRAMAddr(RXSR) & 0x08)
                    {
                        /* crc failed.  Erase packet from the array */
                        RxWrite = OldRxWrite;
                        // Flush the RX FIFO
                        PHYSetShortRAMAddr (RXFLUSH, 0x01);
                    }
                    else
                    {
                        PHYTasksPending.bits.PHY_RX = 1;
                    }
                }
                else
                {
                    RxWrite = OldRxWrite;
                    ZigBeeStatus.flags.bits.bRxBufferOverflow = 1;
                }

DoneReceivingPacket:
                Nop();
            }

        }

    }
    if(TMR0IF)
    {
        if(TMR0IE)
        {
            /* there was a timer overflow */
            TMR0IF = 0;
            timerExtension++;
        }
    }

    UserInterruptHandler();

}

/************************************/
/*        Interrupt Vectors         */
/************************************/

#if defined(MCHP_C18)
#pragma code highVector=0x08
void HighVector (void)
{
    _asm goto HighISR _endasm
}
#pragma code /* return to default code section */
#endif

#if defined(MCHP_C18)
#pragma code lowhVector=0x18
void LowVector (void)
{
    _asm goto HighISR _endasm
}
#pragma code /* return to default code section */
#endif


#else
    #error This file is for the UZ2400 transceiver. Please link the appropriate PHY file for the selected transceiver.
#endif      // RF_CHIP == US2400
