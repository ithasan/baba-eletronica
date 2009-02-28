/*********************************************************************
 *
 *                  MAC for the MRF24J40
 *
 *********************************************************************
 * FileName:        zMAC_MRF24J40.c
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
 * Author               Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * DF/KO                08/31/06 Microchip ZigBee Stack v1.0-3.6
 * DF/KO                09/06/06 Microchip ZigBee Stack v1.0-3.6.1
 *                               Corrected transaction persistence calculation
 *                               (Need corresponding .def file)
 * DF/KO/YY             11/27/06 Microchip ZigBee Stack v1.0-3.7
 * DF/KO/YY             01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/

#include "ZigbeeTasks.h"
#include "zigbee.def"
#include "generic.h"
#include "sralloc.h"
#include "zigbee.h"
#include "SymbolTime.h"
#include "MSPI.h"
#include "zMAC.h"
#include "zPHY.h"
#include "zNWK.h"
#include "zNVM.h"

#ifdef ZCP_DEBUG
    #include "Console.h"
#else
    #define ConsolePutROMString(x)
    #undef printf
    #define printf(x)
    #define PrintChar(x)
#endif

#ifdef I_SUPPORT_SECURITY
    #include "zSecurity.h"
#endif


#if (RF_CHIP == MRF24J40) || (RF_CHIP == UZ2400)


// If we are using separate SPI's for the transceiver and a serial EE, redefine
// the SPI routines.
#if defined(USE_EXTERNAL_NVM) && !defined(EE_AND_RF_SHARE_SPI)
    #define SPIPut( x )     RF_SPIPut( x )
    #define SPIGet()        RF_SPIGet()
#endif


// ******************************************************************************
// Data Structures

typedef struct _association_request_info
{
    BOOL SecurityEnabled;
    SHORT_ADDR CoordAddress;                      //aligns with MCPS_DATA_request
} ASSOCIATION_REQ_INFO;

typedef struct _MAC_INDIRECT_BUFFER_INFO
{
    TICK timeStamp;
    BYTE* packet;
    BYTE size;
//#ifdef I_SUPPORT_SECURITY
    BYTE HeaderSize;
//#endif
} MAC_INDIRECT_BUFFER_INFO;

typedef struct _MAC_INDIRECT_BUFFER
{
    MAC_INDIRECT_BUFFER_INFO* buffer;
} MAC_INDIRECT_BUFFER;

typedef struct _scanParams
{
    DWORD_VAL Channels;
    DWORD_VAL UnscannedChannels;
    BYTE ScanType;
    TICK ScanEndTime;
    TICK ScanStartTime;
    BYTE ScanDuration;
    DWORD numScannedChannels;
    BYTE ResultListSize;
    BYTE *results;  //for energy results
    BYTE *currentResults;
    BYTE maxRSSI;
} SCAN_PARAMS;

// ******************************************************************************
// Variable Definitions

extern volatile PHY_PENDING_TASKS  PHYTasksPending;
ASSOCIATION_REQ_INFO                associationReqParams;
CURRENT_PACKET                      currentPacket;
extern volatile INT_SAVE            IntStatus;

#ifndef NIB_STATIC_IMPLEMENTATION
    /* if the nwkMaxChildren is dynamic then I can only assume that all of the neighbor table might become children */
    #if (MAX_NEIGHBORS > 255)
        #error "macIndirectBuffers has to be less then 256.  NEIGHBOR_TABLE_SIZE is too large."
    #endif
    MAC_INDIRECT_BUFFER             macIndirectBuffers[MAX_NEIGHBORS];
#else
    #if (NIB_nwkMaxChildren > 255)
        #error "macIndirectBuffers has to be less then 256.  nwkMaxChildren is too large."
    #endif
    MAC_INDIRECT_BUFFER             macIndirectBuffers[NIB_nwkMaxChildren];
#endif

MAC_PIB                             macPIB;
MAC_STATUS                          macStatus;
MAC_TASKS_PENDING                   macTasksPending;
extern volatile MAC_FRAME_CONTROL   pendingAckFrameControl;
SCAN_PARAMS                         scanParams;
const ROM DWORD                     timeToAdd[] = {SCAN_DURATION_0,SCAN_DURATION_1,SCAN_DURATION_2,SCAN_DURATION_3,SCAN_DURATION_4,SCAN_DURATION_5,SCAN_DURATION_6,SCAN_DURATION_7,SCAN_DURATION_8,SCAN_DURATION_9,SCAN_DURATION_10,SCAN_DURATION_11,SCAN_DURATION_12,SCAN_DURATION_13,SCAN_DURATION_14};
extern volatile TX_STAT             TxStat;

#ifdef I_SUPPORT_SECURITY
    extern BYTE nwkSecurityLevel;
    extern BYTE SecurityLevel_ZIGBEE_2_IEEE(INPUT BYTE SL_ZigBee);
    extern DWORD_VAL    OutgoingFrameCount[2];
#endif

#ifdef ZCP_DEBUG
    BOOL bDRLong = FALSE;
    BOOL bDisableShortAddress = FALSE;
    BOOL bDisAssocShort = FALSE;
    BYTE MACDataTransmission = 0;
#endif
/*********************************************************************
 * Function:        BOOL MACHasBackgroundTasks( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - MAC layer has background tasks to run
 *                  FALSE - MAC layer does not have background tasks
 *
 * Side Effects:    None
 *
 * Overview:        Determines if the MAC layer has background tasks
 *                  that need to be run.
 *
 * Note:            None
 ********************************************************************/

BYTE MACHasBackgroundTasks(void)
{
    return macTasksPending.Val>0?TRUE:FALSE;
}

/*********************************************************************
 * Function:        BYTE MACGet(void)
 *
 * PreCondition:    A message has been received by the PHY layer
 *
 * Input:           None
 *
 * Output:          The next byte of the current received message.
 *
 * Side Effects:    None
 *
 * Overview:        This function retrieves the next byte of the
 *                  received message that is currently being processed.
 *
 * Note:            None
 ********************************************************************/

BYTE MACGet(void)
{
    params.PD_DATA_indication.psduLength--;
    return *params.PD_DATA_indication.psdu++;
}

/*********************************************************************
 * Function:        void MACDiscardRx(void)
 *
 * PreCondition:    CurrentRxPacket has been allocated.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine frees the current message being processed.
 *
 * Note:            A new message cannot be processed until after this
 *                  function is called.
 ********************************************************************/

void MACDiscardRx(void)
{
    free(CurrentRxPacket);
}

/*********************************************************************
 * Function:        BOOL MACDisable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function disableds the transceiver.
 *
 * Note:            None
 ********************************************************************/
BOOL MACDisable(void)
{
    if(PHYTasksPending.Val)
    {
        return FALSE;
    }
    else
    {
        PHY_RESETn = 0;
        return TRUE;
    }
}

/*********************************************************************
 * Function:        void MACEnable (void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function enables the transceiver.
 *
 * Note:            This function completely reinitializes the
 *                  transceiver, including programming the MAC and NWK
 *                  addresses.
 ********************************************************************/

void MACEnable (void)
{
    LONG_ADDR   tempMACAddr;
    BYTE i;

    GetMACAddress( &tempMACAddr );

    PHYInit();

    MLME_SET_macPANId_hw();

    MLME_SET_macShortAddress_hw();

    /* Program Long MAC Address  */
    for(i=0;i<8;i++)
    {
        PHYSetShortRAMAddr(EADR0+i,tempMACAddr.v[i]);
    }

    /* choose appropriate channel */
    PHYSetLongRAMAddr(0x200, (0x02 | (BYTE)((phyPIB.phyCurrentChannel - 11) << 4)));
    PHYSetShortRAMAddr(PWRCTL,0x04);
    PHYSetShortRAMAddr(PWRCTL,0x00);
}

/*********************************************************************
 * Function:        void MACInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function initializes the MAC layer.
 *
 * Note:            None
 ********************************************************************/

void MACInit(void)
{
    BYTE i;

    TxHeader = TX_HEADER_START;
    TxData = TX_DATA_START;

    /* clear the indirect buffers */
    for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
    {
        macIndirectBuffers[i].buffer = NULL;
    }

    params.MLME_RESET_request.SetDefaultPIB = TRUE;
    MACTasks(MLME_RESET_request);

    macTasksPending.Val = 0;
    currentPacket.info.Val = 0;

    #if defined(I_AM_FFD)
        // if this device is an FFD then lets listen for packets for us
    //  PHYSetLongRAMAddr(0x202,0x80);  // Better PLL performance
    //  PHYSetLongRAMAddr(0x204,0x06);  // Don't know what this does- programming guide says to set it for RX

        macStatus.Val = 0;
    #endif
}

/*********************************************************************
 * Function:        ZIGBEE_PRIMITIVE MACTasks(ZIGBEE_PRIMITIVE inputPrimitive)
 *
 * PreCondition:    None
 *
 * Input:           inputPrimitive - the next primitive to run
 *
 * Output:          The next primitive to run.
 *
 * Side Effects:    Numerous
 *
 * Overview:        This routine executes the indicated ZigBee primitive.
 *                  If no primitive is specified, then background
 *                  tasks are executed.
 *
 * Note:            None
 ********************************************************************/

ZIGBEE_PRIMITIVE MACTasks(ZIGBEE_PRIMITIVE inputPrimitive)
{
    BYTE_VAL d;
    MAC_FRAME_CONTROL   frameControl;

    if(inputPrimitive == NO_PRIMITIVE)
    {
        TICK t;
        TICK d;

        if (macTasksPending.bits.bSendUpMACConfirm)
        {
            macTasksPending.bits.bSendUpMACConfirm = 0;
            frameControl = pendingAckFrameControl;
            goto ProcessACKPacket;
        }

        /* manage background tasks here */
        t = TickGet();

        if( PHYTasksPending.bits.PHY_DATA_REQUEST &&
            TickGetDiff(t, currentPacket.startTime) > SYMBOLS_TO_TICKS((DWORD)aMaxFrameResponseTime))
        {
            PHYTasksPending.bits.PHY_DATA_REQUEST = 0;
        }


        /* if there is a packet already in the TX pending an ACK */
        if(macTasksPending.bits.packetPendingAck == 1)
        {
            if (TxStat.finished)
            {
                IntStatus.CCP2IntF = INTCONbits.INT0IE;
                INTCONbits.INT0IE = 0;
                TxStat.finished = 0;
                if (TxStat.success)
                {
                    TxStat.success = 0;
                    INTCONbits.INT0IE = IntStatus.CCP2IntF;
                    macTasksPending.bits.bSendUpMACConfirm = 1;
                    return NO_PRIMITIVE;
                }
                else
                {
                    INTCONbits.INT0IE = IntStatus.CCP2IntF;
                    // We did not receive an ACK
                    macTasksPending.bits.packetPendingAck = 0;
                    ZigBeeUnblockTx();
#if !(defined (I_AM_COORDINATOR))
                    if (currentPacket.info.bits.association == 1)
                    {
                        params.MLME_ASSOCIATE_confirm.AssocShortAddress.Val = 0xffff;
                        params.MLME_ASSOCIATE_confirm.status = NO_ACK;
                        currentPacket.info.bits.association = 0;
                        macTasksPending.bits.associationPending = 0;
                        return MLME_ASSOCIATE_confirm;
                    }
                    else if (currentPacket.info.bits.data_request == 1)
                    {
                        currentPacket.info.bits.data_request = 0;
                        params.MLME_POLL_confirm.status = NO_ACK;
                        return MLME_POLL_confirm;
                    }
                    else if (currentPacket.info.bits.disassociation == 1)
                    {
                        currentPacket.info.bits.disassociation = 0;
                        params.MLME_DISASSOCIATE_confirm.status = NO_ACK;
                        return MLME_DISASSOCIATE_confirm;
                    }
                    else
#endif

#if !(defined (I_AM_END_DEVICE))
                    if (currentPacket.info.bits.RX_association == 1)
                    {
                        currentPacket.info.bits.RX_association = 0;
                        params.MLME_COMM_STATUS_indication.status = NO_ACK;
                        params.MLME_COMM_STATUS_indication.DstAddrMode = 0x03;
                        params.MLME_COMM_STATUS_indication.DstAddr.v[0] = currentPacket.DstAddr.v[0];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[1] = currentPacket.DstAddr.v[1];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[2] = currentPacket.DstAddr.v[2];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[3] = currentPacket.DstAddr.v[3];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[4] = currentPacket.DstAddr.v[4];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[5] = currentPacket.DstAddr.v[5];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[6] = currentPacket.DstAddr.v[6];
                        params.MLME_COMM_STATUS_indication.DstAddr.v[7] = currentPacket.DstAddr.v[7];
                        params.MLME_COMM_STATUS_indication.SrcAddrMode = 0x03;
                        GetMACAddress( &params.MLME_COMM_STATUS_indication.SrcAddr);
                        return MLME_COMM_STATUS_indication;
                    }
                    else
#endif
                    {
                        params.MCPS_DATA_confirm.status = NO_ACK;
                        params.MCPS_DATA_confirm.msduHandle = currentPacket.sequenceNumber;
                        return MCPS_DATA_confirm;
                    }
                }
            }
        } /* End packet in TX pending an Ack loop */

        if(macTasksPending.bits.dataInBuffer == 1)
        {
            params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = macTasksPending.bits.packetPendingAck;
            goto SendTxBuffer;
        }

        // if the tx is blocked then we can't do any of these because they generate primitives
        // that may exit ZigbeeTasks and this could create a resource control issue
        if(ZigBeeTxBlocked) //(TxFIFOControl.bFIFOInUse)
        {
            return NO_PRIMITIVE;
        }

        if(currentPacket.info.bits.expecting_data == 1)
        {
            if(TickGetDiff(t,currentPacket.startTime) > SYMBOLS_TO_TICKS((DWORD)aMaxFrameResponseTime))
            {
                PHYTasksPending.bits.PHY_DATA_REQUEST = 0;
                currentPacket.info.bits.expecting_data = 0;
                params.MLME_POLL_confirm.status = NO_DATA;
                return MLME_POLL_confirm;
            }
        }

#if !defined(I_AM_COORDINATOR)
        if(macTasksPending.bits.associationPending==1)
        {
            if(currentPacket.info.bits.data_request == 0)
            {
                if(TickGetDiff(t,currentPacket.startTime) > SYMBOLS_TO_TICKS((DWORD)aResponseWaitTime))
                {
                    /* send data request */
                    if(macTasksPending.bits.packetPendingAck==0)
                    {
                        {
                            params.MLME_POLL_request.SecurityEnabled = FALSE;
                        }
                        currentPacket.info.bits.data_request = 1;
                    #ifdef ZCP_DEBUG
                        if( associationReqParams.CoordAddress.Val == 0xfffe )
                            params.MLME_POLL_request.CoordAddrMode = 0x00;
                        else
                    #endif
                            params.MLME_POLL_request.CoordAddrMode = 0x02;
                        params.MLME_POLL_request.CoordPANId = macPIB.macPANId;
                        params.MLME_POLL_request.CoordAddress.ShortAddr.Val = associationReqParams.CoordAddress.Val;
                        return MLME_POLL_request;
                    }
                }
            }
            if(TickGetDiff(t,currentPacket.startTime) > SYMBOLS_TO_TICKS((DWORD)aResponseWaitTime))
            {
                macTasksPending.bits.associationPending = 0;
                params.MLME_ASSOCIATE_confirm.AssocShortAddress.Val = 0xffff;
                params.MLME_ASSOCIATE_confirm.status = NO_DATA;
                macTasksPending.bits.packetPendingAck = 0;
                currentPacket.info.Val = 0;
                return MLME_ASSOCIATE_confirm;
            }
        }
#endif

        if(macTasksPending.bits.scanInProgress == 1)
        {
            BYTE TxStatus;

            TxStatus = PHYGetLongRAMAddr(0x20f);
            t = TickGet();
            d.Val = TickGetDiff(t,scanParams.ScanStartTime);

            if(macTasksPending.bits.channelScanning == 0)
            {
                // if there are no more channels to scan
                if(scanParams.numScannedChannels & 0x80000000)
                {
                    BYTE    test;

                    /* done scanning */
                    macTasksPending.bits.scanInProgress = 0;

                    params.MLME_SCAN_confirm.UnscannedChannels.Val = scanParams.UnscannedChannels.Val;
                    params.MLME_SCAN_confirm.status = SUCCESS;
                    params.MLME_SCAN_confirm.EnergyDetectList = scanParams.results;
                    params.MLME_SCAN_confirm.PANDescriptorList = (PAN_DESCRIPTOR*)scanParams.results;
                    params.MLME_SCAN_confirm.ScanType = scanParams.ScanType;
                    params.MLME_SCAN_confirm.ResultListSize = scanParams.ResultListSize;

                    if((scanParams.ScanType == MAC_SCAN_ACTIVE_SCAN) || (scanParams.ScanType == MAC_SCAN_PASSIVE_SCAN))
                    {
                        if(params.MLME_SCAN_confirm.ResultListSize == 0)
                        {
                            params.MLME_SCAN_confirm.status = MAC_NO_BEACON;
                        }
                    }
                    else if(scanParams.ScanType == MAC_SCAN_ORPHAN_SCAN)
                    {
                        params.MLME_SCAN_confirm.status = MAC_NO_BEACON;
                    }

                    test = PHYGetShortRAMAddr(0x00);
                    test = test & 0xFE;
                          //enable PAN address filtering for beacon frames
                    PHYSetShortRAMAddr(0x00, test);

                    return MLME_SCAN_confirm;
                }

                // if the next channel needs to be scanned
                if(scanParams.Channels.bits.b0)
                {
                    phyPIB.phyCurrentChannel++;

                    /* scan this channel */

                    //save the start time
                    scanParams.ScanStartTime.Val = t.Val;

                    //set the channel to check
                    scanParams.Channels.Val >>= 1;

                    if(scanParams.numScannedChannels == 0)
                    {
                        scanParams.numScannedChannels=1;
                    }
                    else
                    {
                        scanParams.numScannedChannels<<=1;
                    }

                    /* mark channels as having been scanned */
                    scanParams.UnscannedChannels.Val ^= scanParams.numScannedChannels;

                    //indicate that we have started the scan
                    macTasksPending.bits.channelScanning = 1;

                    /* choose appropriate channel */
                    PHYSetLongRAMAddr(0x200, (0x02 | (BYTE)((phyPIB.phyCurrentChannel - 11) << 4)));
                    PHYSetShortRAMAddr(PWRCTL,0x04);
                    PHYSetShortRAMAddr(PWRCTL,0x00);
                    switch(scanParams.ScanType)
                    {
                        #if defined(I_AM_FFD) || defined(INCLUDE_ED_SCAN)
                        case MAC_SCAN_ENERGY_DETECT:
                            scanParams.maxRSSI = 0;
                            break;
                        #endif

                        #if defined(I_AM_FFD) || defined(INCLUDE_ACTIVE_SCAN)
                        case MAC_SCAN_ACTIVE_SCAN:
                            if(TxStatus != 0x80)
                            {
                                /* send out the beacon request on the new channel */
                                TxBuffer[TxData++] = BEACON_REQUEST;
                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = macPIB.macDSN++;
                                TxBuffer[TxHeader--] = 0x08;
                                TxBuffer[TxHeader--] = 0x03;
                                currentPacket.info.Val = 0;
                                params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 0;
                                // beacon request cannot secured
                                params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = 0;

                                goto SendTxBuffer;
                            }
                            //was not able to send the packet.  going to scan complete here will allow this to retry next time the
                            // background tasks are called.
                            goto SCAN_COMPLETE;
                        #endif

                        case MAC_SCAN_PASSIVE_SCAN:
                            break;

                        case MAC_SCAN_ORPHAN_SCAN:

                            /* send out orphan notification on the new channel */
                            if(TxStatus != 0x80)
                            {
                                TxBuffer[TxData++] = ORPHAN_NOTIFICATION;
                                TxHeader -= 7;
                                GetMACAddress( &(TxBuffer[TxHeader]) );
                                TxHeader--;

                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;

                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;

                                TxBuffer[TxHeader--] = 0xff;
                                TxBuffer[TxHeader--] = 0xff;

                                TxBuffer[TxHeader--] = macPIB.macDSN++;
                                TxBuffer[TxHeader--] = 0xC8;
                                TxBuffer[TxHeader--] = 0x03;
                                currentPacket.info.Val = 0;
                                params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 0;
                                params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = 0;

                                goto SendTxBuffer;
                            }
                            //was not able to send the packet.  going to scan complete here will allow this to retry next time the
                            // background tasks are called.
                            goto SCAN_COMPLETE;

                    }


                }
                else
                {
                    phyPIB.phyCurrentChannel++;

                    //set the channel to check
                    scanParams.Channels.Val >>= 1;

                    if(scanParams.numScannedChannels == 0)
                    {
                        scanParams.numScannedChannels=1;
                    }
                    else
                    {
                        scanParams.numScannedChannels<<=1;
                    }

                    /* mark channels as having been scanned */
                    scanParams.UnscannedChannels.Val ^= scanParams.numScannedChannels;

                }
            }
            else
            {
                if(scanParams.ScanType == MAC_SCAN_ENERGY_DETECT)
                {
                    BYTE RSSIcheck;
                    PHYSetShortRAMAddr(0x3E, 0x80);

                    // Firmware Request the RSSI
                    RSSIcheck =     PHYGetShortRAMAddr (0x3E);
                    while ((RSSIcheck & 0x01) != 0x01)
                    {
                        RSSIcheck = PHYGetShortRAMAddr (0x3E);
                    }

                    RSSIcheck = PHYGetLongRAMAddr(0x210);

                    if(RSSIcheck > scanParams.maxRSSI)
                    {
                        scanParams.maxRSSI = RSSIcheck;
                    }

                    //RSSI when packets received again
                    PHYSetShortRAMAddr(0x3E, 0x40);

                }

                // if the scan has run the correct amount of time
                if(d.Val >= scanParams.ScanEndTime.Val)
                {
                    //save any results that are required
                    if(scanParams.ScanType == MAC_SCAN_ENERGY_DETECT)
                    {
                        *scanParams.currentResults++ = scanParams.maxRSSI;
                        scanParams.ResultListSize++;
                        scanParams.maxRSSI = 0;
                    }
                    //Orphan scan, Active scan, and Passive scan are all taken care of when you receive a Coordinator Realignment or Beacon

                    //indicate that this channel is complete
                    macTasksPending.bits.channelScanning = 0;
                }
            }
        }
SCAN_COMPLETE:
#if !defined(I_AM_END_DEVICE)
        if(macTasksPending.bits.indirectPackets == 1)
        {
            BYTE i,j;

            for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
            {
                if(macIndirectBuffers[i].buffer == NULL)
                {
                    if(i==0)
                    {
                        macTasksPending.bits.indirectPackets = 0;
                    }
                    /* first NULL entry we come across is the end of the queue */
                    break;
                }
                else
                {
                    /* this is a valid entry */
                    if(TickGetDiff(t,macIndirectBuffers[i].buffer->timeStamp) > (DWORD)MAC_PIB_macTransactionPersistenceTime)
                    {
                        /* packet timed out */
                        BYTE* packet;
                        BYTE k;

                        packet = macIndirectBuffers[i].buffer->packet;
                        /* let's check the addressing mode */
                        /* if this is a association responce it will be extended addressing */

                        k=*(packet+1);

                        if((k & 0b00001100) == 0b00001100)
                        {

//  This is no longer required.  ZigBee changed the way that disassociation works and thus no longer indirectly sends disassociation requests.
//                            if(currentPacket.info.bits.disassociation == 1)
//                            {
//                                currentPacket.info.bits.disassociation = 0;
//                                params.MLME_DISASSOCIATE_confirm.status = TRANSACTION_EXPIRED;
//                                k = MLME_DISASSOCIATE_confirm;
//                            }
//                            else
                            {
                                packet+=5;
                                for(j=0;j<8;j++)
                                {
                                    params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[j]=*packet++;
                                }
                                params.MLME_COMM_STATUS_indication.status = TRANSACTION_EXPIRED;
                                k = MLME_COMM_STATUS_indication;
                            }
                        }
                        else
                        {
                            params.MCPS_DATA_confirm.status = TRANSACTION_EXPIRED;
                            params.MCPS_DATA_confirm.msduHandle = *(packet+2);
                            k = MCPS_DATA_confirm;
                        }

                        SRAMfree((void *)macIndirectBuffers[i].buffer->packet);
                        SRAMfree((void *)macIndirectBuffers[i].buffer);

                        while(i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER)-1))
                        {
                            macIndirectBuffers[i].buffer = macIndirectBuffers[i+1].buffer;
                            i++;
                        }

                        /* the last entry is now null because we removed one */
                        macIndirectBuffers[i].buffer = NULL;

                        return k;
                    }
                }
            }
        }
#else
    {}
#endif
    }
    else
    {
        /* handle primitive here */
        switch(inputPrimitive)
        {
            case PD_DATA_indication:
            {
                {
                    BYTE sequenceNumber;
                    BYTE decryptResults;

                    #ifdef I_AM_RFD
                        // If we are an RFD, we've received a requested message.  We don't know if
                        // we've received all available messages, so clear this flag so we'll be
                        // able to send another data request after we've processed this one.
                        ZigBeeStatus.flags.bits.bRequestingData = 0;
                    #endif

                    frameControl.word.byte.LSB = MACGet();
                    frameControl.word.byte.MSB = MACGet();

                    sequenceNumber = MACGet();
                    params.MCPS_DATA_indication.DstAddrMode = frameControl.bits.DstAddrMode;
                    params.MCPS_DATA_indication.SrcAddrMode = frameControl.bits.SrcAddrMode;

                    /* i am using params.MCPS_DATA_indication.DstAddrMode here because it equals
                    /* and is faster to access then frameControl.bits.DstAddrMode */
                    if(params.MCPS_DATA_indication.DstAddrMode == 0b10)
                    {
                        params.MCPS_DATA_indication.DstPANId.byte.LSB = MACGet();
                        params.MCPS_DATA_indication.DstPANId.byte.MSB = MACGet();

                        params.MCPS_DATA_indication.DstAddr.ShortAddr.byte.LSB = MACGet();
                        params.MCPS_DATA_indication.DstAddr.ShortAddr.byte.MSB = MACGet();
                        #ifdef ZCP_DEBUG
                            if( bDisableShortAddress && (params.MCPS_DATA_indication.DstAddr.ShortAddr.Val != 0xFFFF))
                            {
                                MACDiscardRx();
                                return NO_PRIMITIVE;
                            }
                        #endif
                    }
                    else if(params.MCPS_DATA_indication.DstAddrMode == 0b11)
                    {
                        params.MCPS_DATA_indication.DstPANId.byte.LSB = MACGet();
                        params.MCPS_DATA_indication.DstPANId.byte.MSB = MACGet();

                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[0] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[1] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[2] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[3] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[4] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[5] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[6] = MACGet();
                        params.MCPS_DATA_indication.DstAddr.LongAddr.v[7] = MACGet();
                    }

                    /* i am using params.MCPS_DATA_indication.SrcAddrMode here because it equals
                    /* and is faster to access then frameControl.bits.DstAddrMode */
                    if(params.MCPS_DATA_indication.SrcAddrMode == 0b10)
                    {
                        #ifdef ZCP_DEBUG
                            if( bDisableShortAddress )
                            {
                                MACDiscardRx();
                                return NO_PRIMITIVE;
                            }
                        #endif
                        if(frameControl.bits.IntraPAN)
                        {
                            params.MCPS_DATA_indication.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;
                            params.MCPS_DATA_indication.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                        }
                        else
                        {
                            params.MCPS_DATA_indication.SrcPANId.byte.LSB = MACGet();
                            params.MCPS_DATA_indication.SrcPANId.byte.MSB = MACGet();
                        }

                        params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.LSB = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.MSB = MACGet();
                    }
                    else if(params.MCPS_DATA_indication.SrcAddrMode == 0b11)
                    {
                        if(frameControl.bits.IntraPAN)
                        {
                            params.MCPS_DATA_indication.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;
                            params.MCPS_DATA_indication.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                        }
                        else
                        {
                            params.MCPS_DATA_indication.SrcPANId.byte.LSB = MACGet();
                            params.MCPS_DATA_indication.SrcPANId.byte.MSB = MACGet();
                        }

                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[0] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[1] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[2] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[3] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[4] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[5] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[6] = MACGet();
                        params.MCPS_DATA_indication.SrcAddr.LongAddr.v[7] = MACGet();
                    }

                    /* TODO(DF) - decrypt the data payload if security is enabled */
                    #ifdef I_SUPPORT_SECURITY_SPEC
                        if(frameControl.bits.SecurityEnabled)
                        {
                            #ifdef CHECK_MAC_FRAME_COUNTER
                                DWORD_VAL frameCount;
                                BYTE KeySeq, i;

                                for(i = 0; i < 4; i++)
                                {
                                    frameCount.v[i] = MACGet();
                                }
                                KeySeq = MACGet();

                                if( securityStatus.flags.bits.nwkAllFresh && !checkFrameCount(&frameCount, KeySeq))
                                {
                                    MACDiscardRx();
                                    return NO_PRIMITIVE;
                                }
                            #else
                                params.PD_DATA_indication.psduLength -= 5;
                                params.PD_DATA_indication.psdu += 5;
                            #endif
                        }
                    #endif

                    /* SecurityUse and ACLEntry are used by many different commands packets as well
                    /* as the data packet so we will populate that data here */
                    /* TODO(DF) - SecurityUse */
                    /* TODO(DF) - ACLEntry */

                    //this is LQI in the MRF24J40
                    params.MCPS_DATA_indication.mpduLinkQuality = *(params.PD_DATA_indication.psdu + params.PD_DATA_indication.psduLength - 2);

                    //this is RSSI converted to unsigned
                    //params.MCPS_DATA_indication.mpduLinkQuality = *(params.PD_DATA_indication.psdu + params.PD_DATA_indication.psduLength - 1) + 128;
                    params.PD_DATA_indication.psduLength-=2;

                    switch(frameControl.bits.FrameType)
                    {
                        case MAC_FRAME_TYPE_DATA:
                            //These two parameters overlap in the params structure so don't need to copy them
                            //params.MCPS_DATA_indication.msduLength = params.PD_DATA_indication.psduLength;
                            //params.MCPS_DATA_indication.msdu = params.PD_DATA_indication.psdu;

							#if defined(I_AM_RFD)
                            	// this is to filter out a broadcast packet that an RFD might here right after sending out
	                            // it's data request
    	                        if(params.MCPS_DATA_indication.DstAddr.ShortAddr.Val == 0xFFFF)
        	                    {
            	                    MACDiscardRx();
                	                return NO_PRIMITIVE;
                    	        }
							#endif

                            //if this is a zero length packet then send back no_data and kill the packet here
                            if(currentPacket.info.bits.expecting_data == 1)
                            {
                                currentPacket.info.bits.expecting_data = 0;
                                if(params.MCPS_DATA_indication.msduLength == 0)
                                {
                                    MACDiscardRx();
                                    params.MLME_POLL_confirm.status = NO_DATA;
                                    return MLME_POLL_confirm;
                                }
                            }

                            #ifdef ZCP_DEBUG
                            if( MACDataTransmission )
                            {
                                BYTE i;
                                BYTE lastByte;
                                i = params.PD_DATA_indication.psduLength;

                                #ifdef ZCP_PRINTOUT
                                    printf("\r\nMCPS_DATA_indication Result: ");
                                    printf("\r\nSrcAddrMode: ");
                                    PrintChar(frameControl.bits.SrcAddrMode);
                                    printf("\r\nSrcAddr: ");
                                    if( frameControl.bits.SrcAddrMode == 0x02 )
                                    {
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.ShortAddr.v[1]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.ShortAddr.v[0]);
                                    }
                                    else
                                    {
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[7]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[6]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[5]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[4]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[3]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[2]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[1]);
                                        PrintChar(params.MCPS_DATA_request.SrcAddr.LongAddr.v[0]);
                                    }
                                    printf("\r\nDstAddrMode: ");
                                    PrintChar(frameControl.bits.DstAddrMode);
                                    printf("\r\nMAC Data Transmission Received: ");
                                    while(params.PD_DATA_indication.psduLength)
                                    {
                                        lastByte= MACGet();
                                        PrintChar(lastByte);
                                    }
                                    printf("\r\nLinkQuality: ");
                                    PrintChar(params.MCPS_DATA_indication.mpduLinkQuality);
                                    printf("\r\n");
                                #endif

                                MACDiscardRx();
                                if( i == 6 && lastByte == 0x16 )
                                {
                                    ADDR tmpAddr;

                                    TxBuffer[TxData++] = 0xff;
                                    TxBuffer[TxData++] = 0xe4;
                                    TxBuffer[TxData++] = 0xe3;
                                    TxBuffer[TxData++] = 0xe2;
                                    TxBuffer[TxData++] = 0xe1;
                                    TxBuffer[TxData++] = 0xe0;

                                    params.MCPS_DATA_request.SrcPANId.Val = 0x1AAA;
                                    params.MCPS_DATA_request.DstPANId.Val = 0x1AAA;

                                    params.MCPS_DATA_request.SrcAddrMode = frameControl.bits.DstAddrMode;
                                    tmpAddr = params.MCPS_DATA_request.SrcAddr;
                                    if( params.MCPS_DATA_request.SrcAddrMode == 0x02 )
                                    {
                                        params.MCPS_DATA_request.SrcAddr.ShortAddr.Val = macPIB.macShortAddress.Val;
                                    }
                                    else
                                    {
                                        GetMACAddress(&params.MCPS_DATA_request.SrcAddr.LongAddr);
                                    }
                                    params.MCPS_DATA_request.DstAddrMode = frameControl.bits.SrcAddrMode;
                                    params.MCPS_DATA_request.DstAddr = tmpAddr;

                                    params.MCPS_DATA_request.frameType = FRAME_DATA;
                                    params.MCPS_DATA_request.msduHandle = MLME_GET_macDSN();
                                    params.MCPS_DATA_request.TxOptions.Val = 0x01;
                                    return MCPS_DATA_request;
                                }
                                return NO_PRIMITIVE;
                            }
                            #endif
                            return MCPS_DATA_indication;

                        case MAC_FRAME_TYPE_BEACON:
                            params.MLME_BEACON_NOTIFY_indication.CoordAddrMode = params.MCPS_DATA_indication.SrcAddrMode;
                            params.MLME_BEACON_NOTIFY_indication.BSN = sequenceNumber;
                            params.MLME_BEACON_NOTIFY_indication.LogicalChannel = phyPIB.phyCurrentChannel;
                            params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.byte.LSB = MACGet();
                            params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.byte.MSB = MACGet();

                            {
                                GTS_HEADER GTSHeader;

                                GTSHeader.Val = MACGet();

                                params.MLME_BEACON_NOTIFY_indication.GTSPermit = FALSE;
                                if(GTSHeader.bits.GTSPermit)
                                {
                                    params.MLME_BEACON_NOTIFY_indication.GTSPermit = TRUE;
                                }

                                // TODO: We don't support GTS or Pending Addresses, so we'll reuse GTSHeader
                                // to get the Pending Address Fields byte
                                GTSHeader.Val = MACGet();
                            }

                            params.MLME_BEACON_NOTIFY_indication.TimeStamp = TickGet();

                            /* TODO(DF) - add params.MLME_BEACON_NOTIFY_indication.SecurityFailure when security suite added */
                            //params.MLME_BEACON_NOTIFY_indication.SecurityFailure = decryptResults;

                            //We don't use PendAddrSpec or AddrList because we have a non-beacon network only
                            //params.MLME_BEACON_NOTIFY_indication.PendAddrSpec
                            //params.MLME_BEACON_NOTIFY_indication.AddrList

                            //The following parameters overlap in the params structure so don't need to copy them
                            //params.MLME_BEACON_NOTIFY_indication.CoordAddrMode = params.MCPS_DATA_indication.SrcAddrMode;
                            //params.MLME_BEACON_NOTIFY_indication.CoordPANId = params.MCPS_DATA_indication.SrcPANId;
                            //params.MLME_BEACON_NOTIFY_indication.CoordAddress = params.MCPS_DATA_indication.SrcAddr;
                            //params.MLME_BEACON_NOTIFY_indication.LinkQuality = params.MCPS_DATA_indication.msduLinkQuality;
                            //params.MLME_BEACON_NOTIFY_indication.SecurityUse = params.MCPS_DATA_indication.SecurityUse;
                            //params.MLME_BEACON_NOTIFY_indication.ACLEntry = params.MCPS_DATA_indication.ACLEntry;
                            //params.MLME_BEACON_NOTIFY_indication.sduLength = params.MCPS_DATA_indication.msduLength;
                            //params.MLME_BEACON_NOTIFY_indication.sdu = params.MCPS_DATA_indication.msdu;

                            if(macTasksPending.bits.scanInProgress == 1)
                            {
                                if((scanParams.ScanType == MAC_SCAN_ACTIVE_SCAN) || (scanParams.ScanType == MAC_SCAN_PASSIVE_SCAN))
                                {
                                    PAN_DESCRIPTOR *p;

                                    if(scanParams.results == NULL)
                                    {
                                        p = (PAN_DESCRIPTOR*)SRAMalloc(sizeof(PAN_DESCRIPTOR));
                                        scanParams.results = (void*)p;
                                    }
                                    else
                                    {
                                        p = (PAN_DESCRIPTOR*)scanParams.results;

                                        while(p->next != NULL)
                                        {
                                            p = p->next;
                                        }

                                        p->next = (PAN_DESCRIPTOR*)SRAMalloc(sizeof(PAN_DESCRIPTOR));
                                        if(p->next == NULL)
                                        {
                                            /* no room for more */
                                            params.MLME_BEACON_NOTIFY_indication.ACLEntry = 0x08;
                                            return MLME_BEACON_NOTIFY_indication;
                                        }
                                        p = p->next;
                                    }

                                    if(p != NULL)
                                    {
                                        scanParams.ResultListSize++;
                                        p->CoordAddrMode = params.MLME_BEACON_NOTIFY_indication.CoordAddrMode & 0xFD;
                                        p->GTSPermit = params.MLME_BEACON_NOTIFY_indication.GTSPermit;
                                        /* TODO(DF) - SecurityUse */
                                        /* TODO(DF) - SecurityFailure */
                                        /* TODO(DF) - ACLEntry */
                                        p->CoordPANId = params.MLME_BEACON_NOTIFY_indication.CoordPANId;
                                        p->CoordAddress = params.MCPS_DATA_indication.SrcAddr;
                                        p->LogicalChannel = params.MLME_BEACON_NOTIFY_indication.LogicalChannel;
                                        p->SuperframeSpec = params.MLME_BEACON_NOTIFY_indication.SuperframeSpec;
                                        p->LinkQuality = params.MCPS_DATA_indication.mpduLinkQuality;
                                        p->TimeStamp = params.MLME_BEACON_NOTIFY_indication.TimeStamp;
                                        p->next = NULL;
                                    }
                                }
                            }
                            params.MLME_BEACON_NOTIFY_indication.ACLEntry = 0x08;
                            return MLME_BEACON_NOTIFY_indication;

                        case MAC_FRAME_TYPE_ACK:
                            /* if this ACK was for us then mark the packet as being ACKed */
                            MACDiscardRx();

                            if(macTasksPending.bits.packetPendingAck == 1)
                            {
                                /* if we are waiting for an ACK */
                                if(sequenceNumber == currentPacket.sequenceNumber)  //OldACKSeqNum)
                                {
                                    /* and this ACK matches the packet that was just sent out */
ProcessACKPacket:
                                    ZigBeeUnblockTx();
                                    macTasksPending.bits.packetPendingAck = 0;
									#if !defined(I_AM_COORDINATOR)
                                    	if(currentPacket.info.bits.association == 1)
	                                    {
    	                                    macTasksPending.bits.associationPending = 1;
        	                                /* have aResponseWaitTime to get a response */
            	                            currentPacket.startTime = TickGet();
                	                    }
                    	                else if(currentPacket.info.bits.data_request == 1)
									#else
                                    	if(currentPacket.info.bits.data_request == 1)
									#endif
                                    {
                                        if(frameControl.bits.FramePending == 1)
                                        {
                                            currentPacket.info.bits.expecting_data = 1;
                                            currentPacket.startTime = TickGet();
                                        }
                                        else
                                        {
                                            currentPacket.info.bits.data_request = 0;
                                            params.MLME_POLL_confirm.status = NO_DATA;
                                            return MLME_POLL_confirm;
                                        }
                                    }

                                    if(currentPacket.info.bits.disassociation == 1)
                                    {
                                        MLME_SET_macShortAddress_hw();
                                        currentPacket.info.bits.disassociation = 0;
                                        params.MLME_DISASSOCIATE_confirm.status = SUCCESS;
                                        macPIB.macPANId.Val = 0xFFFF;
                                        macPIB.macShortAddress.Val = 0xFFFF;
                                    #ifdef ZCP_PRINTOUT
                                        printf("\r\nMAC PIB: ");
                                        printf("\r\nMAC PANID: ");
                                        PrintChar(macPIB.macPANId.v[1]);
                                        PrintChar(macPIB.macPANId.v[0]);
                                        printf("\r\nMAC Short Address: ");
                                        PrintChar(macPIB.macShortAddress.v[1]);
                                        PrintChar(macPIB.macShortAddress.v[0]);
                                        printf("\r\n");
                                    #endif
                                        return MLME_DISASSOCIATE_confirm;
                                    }
                                    else if(currentPacket.info.bits.RX_association == 1)
                                    {
                                        currentPacket.info.bits.RX_association = 0;
                                        params.MLME_COMM_STATUS_indication.status = SUCCESS;
                                        params.MLME_COMM_STATUS_indication.DstAddrMode = 0x03;
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[0] = currentPacket.DstAddr.v[0];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[1] = currentPacket.DstAddr.v[1];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[2] = currentPacket.DstAddr.v[2];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[3] = currentPacket.DstAddr.v[3];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[4] = currentPacket.DstAddr.v[4];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[5] = currentPacket.DstAddr.v[5];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[6] = currentPacket.DstAddr.v[6];
                                        params.MLME_COMM_STATUS_indication.DstAddr.v[7] = currentPacket.DstAddr.v[7];
                                        params.MLME_COMM_STATUS_indication.SrcAddrMode = 0x03;
                                        GetMACAddress( &params.MLME_COMM_STATUS_indication.SrcAddr );
                                        return MLME_COMM_STATUS_indication;
                                    }
                                    else
                                    {
                                        params.MCPS_DATA_confirm.status = SUCCESS;
                                        params.MCPS_DATA_confirm.msduHandle = currentPacket.sequenceNumber; //OldACKSeqNum;
                                        return MCPS_DATA_confirm;
                                    }
                                }
                            }
                            /* else we are done cause we already discarded it */
                            break;

                        case MAC_FRAME_TYPE_CMD:
                            {
                                /* this is a command frame */
                                BYTE command;

                                /* read the command indication byte */
                                command = MACGet();

                                /* determine the course of action */
                                switch(command)
                                {
                                    case ASSOCIATION_REQUEST:
                                        params.MLME_ASSOCIATE_indication.CapabilityInformation.Val = MACGet();
                                        params.MLME_ASSOCIATE_indication.SecurityUse = frameControl.bits.SecurityEnabled;
                                        MACDiscardRx();
                                        return MLME_ASSOCIATE_indication;

                                    case ORPHAN_NOTIFICATION:
                                        //The following parameters overlap MCPS_DATA_indication and thus are already loaded
                                        //params.MLME_ORPHAN_indication.OrphanAddress = params.MCPS_DATA_indication.SrcAddr.LongAddr
                                        //params.MLME_ORPHAN_indication.SecurityUse = params.MCPH_DATA_indication.SecurityUse
                                        //params.MLME_ORPHAN_indication.ACLEntry = params.MCPH_DATA_indication.ACLEntry

                                        //done with the packet so lets discard it before we move on
                                        MACDiscardRx();
                                        return MLME_ORPHAN_indication;

                                    case DISASSOCIATION_NOTIFICATION:
                                        //The following parameters overlap MCPS_DATA_indication and thus are already loaded
                                        //params.MLME_DISASSOCIATE_indication.DeviceAddress = params.MCPS_DATA_indication.SrcAddr.LongAddr
                                        //params.MLME_DISASSOCIATE_indication.SecurityUse = params.MCPH_DATA_indication.SecurityUse
                                        //params.MLME_DISASSOCIATE_indication.ACLEntry = params.MCPH_DATA_indication.ACLEntry

                                        params.MLME_DISASSOCIATE_indication.DisassociateReason = MACGet();
                                        params.MLME_DISASSOCIATE_indication.ACLEntry = 0x08;

                                        MACDiscardRx();
                                        return MLME_DISASSOCIATE_indication;
                                    //case GTS_REQUEST: //do nothing.  This is not valid for non-beacon networks
#if !defined(I_AM_END_DEVICE)
                                    case BEACON_REQUEST:
                                        /* need to send out a beacon frame */

                                        /* if this is a beacon request then we are done with the old packet */
                                        MACDiscardRx();
                                        if(macStatus.bits.allowBeacon)
                                        {
                                            /* stuff the beacon into the buffer and send it */
                                            /* Superframe specification */
                                            TxBuffer[TxData++] = MAC_PIB_macBeaconOrder | (MAC_PIB_macSuperframeOrder << 4);
    #if defined(I_AM_COORDINATOR)
                                            TxBuffer[TxData++] = 0x0F | (MAC_PIB_macBattLifeExt << 4) | (1<<6) | (macPIB.macAssociationPermit << 7);
    #else
                                            TxBuffer[TxData++] = 0x0F | (MAC_PIB_macBattLifeExt << 4) | (macPIB.macAssociationPermit << 7);
    #endif
                                            /* GTS fields */
                                            /* non-beacon only - thus no GTS */
                                            TxBuffer[TxData++] = 0x00;

                                            /* Pending Address Feilds */
                                            /* not adding the pending fields because it is pointless in a non-beacon network */
                                            TxBuffer[TxData++] = 0x00;

                                            /* Beacon payload */
                                            {
                                                BYTE i;

                                                for(i=0;i<MAC_PIB_macBeaconPayloadLength;i++)
                                                {
                                                    TxBuffer[TxData++] = macPIB.macBeaconPayload[i];
                                                }
                                            }
                                            /* TX offset not sent because we are non-beacon network */

                                            params.MCPS_DATA_request.DstAddrMode = 0;
                                        #ifdef ZCP_DEBUG
                                            if( bDisableShortAddress )
                                            {
                                                params.MCPS_DATA_request.SrcAddrMode = 0x03;
                                                GetMACAddress(&params.MCPS_DATA_request.SrcAddr.LongAddr);
                                            }
                                            else
                                        #endif
                                            {
                                                params.MCPS_DATA_request.SrcAddrMode = 0x02;
                                                params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.MSB = macPIB.macShortAddress.byte.MSB;
                                                params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.LSB = macPIB.macShortAddress.byte.LSB;
                                            }

                                            params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                                            params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;
                                            params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;
                                            /* no security, not ACKed, not indirect, no GTS */
                                            params.MCPS_DATA_request.TxOptions.Val = 0;
                                            params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_BEACON;

                                            goto MCPS_DATA_request_label;
                                        }
                                        else
                                        {
                                            return NO_PRIMITIVE;
                                        }
#endif

                                    case COORDINATOR_REALIGNMENT:
                                        /* otherwise if I get a broadcast realignment then what am I suppose to do? */

                                        if(macTasksPending.bits.scanInProgress == 1)
                                        {
                                            if(scanParams.ScanType == MAC_SCAN_ORPHAN_SCAN)
                                            {
                                                BYTE    test;

                                                /* this was an expected coordinator realignment */
                                                /* done scanning */
                                                macTasksPending.bits.scanInProgress = 0;

                                                //save the long address of the parent
                                                macPIB.macCoordExtendedAddress.v[0] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[0];
                                                macPIB.macCoordExtendedAddress.v[1] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[1];
                                                macPIB.macCoordExtendedAddress.v[2] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[2];
                                                macPIB.macCoordExtendedAddress.v[3] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[3];
                                                macPIB.macCoordExtendedAddress.v[4] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[4];
                                                macPIB.macCoordExtendedAddress.v[5] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[5];
                                                macPIB.macCoordExtendedAddress.v[6] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[6];
                                                macPIB.macCoordExtendedAddress.v[7] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[7];

                                                macPIB.macPANId.byte.LSB = MACGet();
                                                macPIB.macPANId.byte.MSB = MACGet();
                                                macPIB.macCoordShortAddress.byte.LSB = MACGet();
                                                macPIB.macCoordShortAddress.byte.MSB = MACGet();
                                                MACGet();  //logical channel, already there so just ignore
                                                macPIB.macShortAddress.byte.LSB = MACGet();
                                                macPIB.macShortAddress.byte.MSB = MACGet();

                                                MACDiscardRx();

                                                test = PHYGetShortRAMAddr(0x00);
                                                test = test & 0xFE;
                                                //enable PAN address filtering for beacon frames
                                                PHYSetShortRAMAddr(0x00, test);

                                                params.MLME_SCAN_confirm.status = SUCCESS;
                                                params.MLME_SCAN_confirm.UnscannedChannels = scanParams.UnscannedChannels;
                                                params.MLME_SCAN_confirm.EnergyDetectList = scanParams.results;
                                                params.MLME_SCAN_confirm.PANDescriptorList = (PAN_DESCRIPTOR*)scanParams.results;
                                                params.MLME_SCAN_confirm.ScanType = scanParams.ScanType;
                                                params.MLME_SCAN_confirm.ResultListSize = scanParams.ResultListSize;

                                                macTasksPending.bits.channelScanning = 0;

                                                return MLME_SCAN_confirm;
                                            }
                                        }

                                        // TODO(DF): IEEE 802.15.4 only specifies that you return REALIGNMENT and not try to recover the network by switching channels or PAN IDs thus we do not try to recover
                                        params.MLME_SYNC_LOSS_indication.LossReason = REALIGNMENT;
                                        MACDiscardRx();
                                        return MLME_SYNC_LOSS_indication;

									#if defined(I_AM_FFD)
                                    	case PAN_ID_CONFLICT_NOTIFICATION:
                                        	params.MLME_SYNC_LOSS_indication.LossReason = PAN_ID_CONFLICT;
	                                        MACDiscardRx();
    	                                    return MLME_SYNC_LOSS_indication;
									#endif

                                    case DATA_REQUEST:
                                    {
                                        BYTE* packet;
                                        BYTE i,k;
                                        struct _addressMatch
                                        {
                                            unsigned int Val :1;
                                        } addressMatch;

                                        /* there is no other useful information in the packet so discard it */
                                        MACDiscardRx();

                                        /* start saying that we found a packet for the device */
                                        addressMatch.Val = FALSE;

                                        /* search to see if there is a packet for the device */
                                        for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
                                        {
                                            if(macIndirectBuffers[i].buffer == NULL)
                                            {
                                                /* first NULL entry we come across is the end of the queue */
                                                break;
                                            }
                                            else
                                            {
                                                /* this is a valid entry so lets see if it is for this device */
                                                packet = macIndirectBuffers[i].buffer->packet;
                                                /* before we go off and check the address, let's check the addressing mode */
                                                /* if this is a association responce it will be extended addressing on both the data request and the response */
                                                /* otherwise it is someone in our network so it should be short addressing mode on both the request and response */
                                                k=*(packet+1);

                                                if(params.MCPS_DATA_indication.SrcAddrMode == 0b11 ) /* long addressing in the data request so we are looking for a packet with long addressing */
                                                {
                                                    if((k & 0b00001100) == 0b00001100)
                                                    {
                                                        BYTE i;
                                                        BYTE* p;

                                                        p = packet+5;

                                                        /* this packet also has long addressing mode */
                                                        addressMatch.Val = TRUE;
                                                        for(i=0;i<8;i++)
                                                        {
                                                            currentPacket.DstAddr.v[i] = *p;

                                                            if(*p++ != params.MCPS_DATA_indication.SrcAddr.LongAddr.v[i])
                                                            {
                                                                addressMatch.Val = FALSE;
                                                            }
                                                        }
                                                        // must be association response or coord realignment
                                                        currentPacket.info.bits.RX_association = 1;
                                                    }
                                                }
                                                else
                                                {
                                                    if((k & 0b00001100) == 0b00001000)
                                                    {
                                                        BYTE* p;

                                                        /* this packet also has short addressing mode */
                                                        p = packet+5;

                                                        addressMatch.Val = TRUE;
                                                        if(*p++ != params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.LSB)
                                                        {
                                                            addressMatch.Val = FALSE;
                                                        }
                                                        if(*p != params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.MSB)
                                                        {
                                                            addressMatch.Val = FALSE;
                                                        }
                                                        //this packet is not a association response or coord realignment
                                                        currentPacket.info.bits.association = 0;
                                                        currentPacket.info.bits.RX_association = 0;
                                                    }
                                                }
                                                if(addressMatch.Val == TRUE)
                                                {
                                                    break;
                                                }
                                            }
                                        }
                                        if(addressMatch.Val == TRUE)
                                        {
                                            if(ZigBeeReady())
                                            {
                                                macTasksPending.bits.packetPendingAck = 0;
                                                currentPacket.sequenceNumber = *(packet+2);

                                                if((*packet & 0b00100000) != 0)
                                                {
                                                    macTasksPending.bits.packetPendingAck = 1;
                                                    params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 1;
                                                }
                                                else
                                                {
                                                    params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 0;
                                                }
                                                params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = 0;
                                                #ifdef I_SUPPORT_SECURITY_SPEC
                                                    if (*packet & 0x08)
                                                    {
                                                        params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = 1;
                                                    }
                                                #endif

                                                /* we found a packet for this device */
                                                k = macIndirectBuffers[i].buffer->HeaderSize;
                                                while(k--)
                                                {
                                                    TxBuffer[TX_HEADER_START-k] = *packet++;
                                                }
                                                TxHeader = TX_HEADER_START-macIndirectBuffers[i].buffer->HeaderSize;

                                                k = macIndirectBuffers[i].buffer->size - macIndirectBuffers[i].buffer->HeaderSize;
                                                while(k--)
                                                {
                                                    TxBuffer[TxData++] = *packet++;
                                                }

                                                SRAMfree((void *)macIndirectBuffers[i].buffer->packet);
                                                SRAMfree((void *)macIndirectBuffers[i].buffer);

                                                while(i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER)-1))
                                                {
                                                    macIndirectBuffers[i].buffer = macIndirectBuffers[i+1].buffer;
                                                    i++;
                                                }

                                                /* the last entry is now null because we removed one */
                                                macIndirectBuffers[i].buffer = NULL;

                                                //if (TxBuffer[0] & 0x20)   //(TxBuffer[TxHeader]&0x20)
                                                //{
                                                //    /* ACK transmission requested */
                                                //    params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 1;
                                                //}
                                                //else
                                                //{
                                                //    params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 0;
                                                //}

                                                goto SendTxBuffer;
                                            }
                                        }
                                        else
                                        {
                                            //send zero length packet to associated devices
                                            if(!ZigBeeReady())
                                            {
                                                return NO_PRIMITIVE;
                                            }

                                            if(params.MCPS_DATA_indication.SrcAddrMode == 0b10 ||
                                                params.MCPS_DATA_indication.SrcAddrMode == 0b11 )
                                            {
                                            #ifdef ZCP_DEBUG
                                                BYTE SrcMode = params.MCPS_DATA_indication.DstAddrMode;
                                            #endif
                                                /*if(params.MCPS_DATA_indication.SecurityUse == TRUE)
                                                {
                                                    params.MCPS_DATA_request.TxOptions.Val = 0b00001000;
                                                }
                                                else
                                                {
                                                    params.MCPS_DATA_request.TxOptions.Val = 0b00000000;
                                                }*/
                                                params.MCPS_DATA_request.TxOptions.Val = 0;

                                                params.MCPS_DATA_request.DstPANId.byte.LSB = params.MCPS_DATA_indication.SrcPANId.byte.LSB;
                                                params.MCPS_DATA_request.DstPANId.byte.MSB = params.MCPS_DATA_indication.SrcPANId.byte.MSB;

                                                if( params.MCPS_DATA_indication.SrcAddrMode == 0b10)
                                                {
                                                    params.MCPS_DATA_request.DstAddrMode = 0x02;
                                                    params.MCPS_DATA_request.DstAddr.ShortAddr.byte.LSB = params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.LSB;
                                                    params.MCPS_DATA_request.DstAddr.ShortAddr.byte.MSB = params.MCPS_DATA_indication.SrcAddr.ShortAddr.byte.MSB;
                                                }
                                                else
                                                {
                                                    params.MCPS_DATA_request.DstAddrMode = 0x03;
                                                    params.MCPS_DATA_request.DstAddr.LongAddr = params.MCPS_DATA_indication.SrcAddr.LongAddr;
                                                }

                                            #ifdef ZCP_DEBUG
                                                if( SrcMode == 0b11)
                                                {
                                                    params.MCPS_DATA_request.SrcAddrMode = 0x03;
                                                    GetMACAddress(&(params.MCPS_DATA_request.SrcAddr.LongAddr));
                                                }
                                                else
                                            #endif
                                                {
                                                    params.MCPS_DATA_request.SrcAddrMode = 0x02;
                                                    params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.LSB = macPIB.macShortAddress.byte.LSB;
                                                    params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.MSB = macPIB.macShortAddress.byte.MSB;
                                                }

                                                params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                                                params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                                                currentPacket.sequenceNumber = macPIB.macDSN;

                                                params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;

                                                params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_DATA;

                                                goto MCPS_DATA_request_label;
                                            }
                                        }
                                        return NO_PRIMITIVE;
                                    }

                                    case ASSOCIATION_RESPONSE:

                                        macTasksPending.bits.associationPending = 0;
                                        currentPacket.info.bits.association = 0;
                                        params.MLME_ASSOCIATE_confirm.AssocShortAddress.byte.LSB = MACGet();
                                        params.MLME_ASSOCIATE_confirm.AssocShortAddress.byte.MSB = MACGet();
                                        params.MLME_ASSOCIATE_confirm.status = MACGet();
                                        if(params.MLME_ASSOCIATE_confirm.status == 0x00)
                                        {
                                            //save the long address of the parent
                                            macPIB.macCoordExtendedAddress.v[0] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[0];
                                            macPIB.macCoordExtendedAddress.v[1] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[1];
                                            macPIB.macCoordExtendedAddress.v[2] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[2];
                                            macPIB.macCoordExtendedAddress.v[3] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[3];
                                            macPIB.macCoordExtendedAddress.v[4] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[4];
                                            macPIB.macCoordExtendedAddress.v[5] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[5];
                                            macPIB.macCoordExtendedAddress.v[6] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[6];
                                            macPIB.macCoordExtendedAddress.v[7] = params.MCPS_DATA_indication.SrcAddr.LongAddr.v[7];
                                        }
                                        MACDiscardRx();
                                        return MLME_ASSOCIATE_confirm;
                                    default:
                                        MACDiscardRx();
                                        return NO_PRIMITIVE;
                                        break;
                                }
                            }
                        default:
                            /* packet was of a reserved frame type */
                            /* discard the packet */
                            MACDiscardRx();
                            return NO_PRIMITIVE;
                            break;
                    }
                }
                break;
            }   // End PD_DATA_indication

            /*
            case PD_DATA_confirm:
                break;
            case PLME_ED_confirm:
                break;      */

            /* The user will access the variable directly
            case PLME_GET_confirm:
                break;      */

            /* Will be handled automatically by CSMA-CA module
            case PLME_CCA_confirm:
                break;
            case PLME_SET_TRX_STATE_confirm:
                break;      */

            /* The user will access the variable directly
            case PLME_SET_confirm:
                break;      */

            case MCPS_DATA_request:
            {
                // if coming here, the MCPS_DATA_request is coming from the NWK layer and it is not a routing
            #ifdef I_SUPPORT_SECURITY
                if( TxBuffer[TxHeader+2] & 0x02 )   // security on
                {
                    if( !DataEncrypt(TxBuffer, &TxData, &(TxBuffer[TxHeader+1]), (TX_HEADER_START-TxHeader), ID_NetworkKey, TRUE))
                    {
                        ZigBeeUnblockTx();
                        params.MCPS_DATA_confirm.status = FAILED_SECURITY_CHECK;
                        return MCPS_DATA_confirm;
                    }
                }
            #endif


                currentPacket.sequenceNumber = macPIB.macDSN++;
MCPS_DATA_request_label:
                currentPacket.info.Val = 0;
MCPS_DATA_request_label_skip_init:
                {
                    BOOL intraPAN = FALSE;

                    if(params.MCPS_DATA_request.SrcAddrMode == 0x02)
                    {
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.MSB;
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.LSB;

                        if(params.MCPS_DATA_request.SrcPANId.Val == params.MCPS_DATA_request.DstPANId.Val)
                        {
                            intraPAN = TRUE;
                        }
                        else
                        {
                            TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcPANId.byte.MSB;
                            TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcPANId.byte.LSB;
                        }
                    }
                    else if(params.MCPS_DATA_request.SrcAddrMode == 0x03)
                    {
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[7];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[6];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[5];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[4];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[3];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[2];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[1];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcAddr.v[0];

                        if(params.MCPS_DATA_request.SrcPANId.Val == params.MCPS_DATA_request.DstPANId.Val)
                        {
                            intraPAN = TRUE;
                        }
                        else
                        {
                            TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcPANId.byte.MSB;
                            TxBuffer[TxHeader--]=params.MCPS_DATA_request.SrcPANId.byte.LSB;
                        }
                    }

                    if(params.MCPS_DATA_request.DstAddrMode == 0x02)
                    {
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.ShortAddr.byte.MSB;
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.ShortAddr.byte.LSB;

                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstPANId.byte.MSB;
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstPANId.byte.LSB;
                    }
                    else if(params.MCPS_DATA_request.DstAddrMode == 0x03)
                    {
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[7];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[6];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[5];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[4];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[3];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[2];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[1];
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstAddr.v[0];

                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstPANId.byte.MSB;
                        TxBuffer[TxHeader--]=params.MCPS_DATA_request.DstPANId.byte.LSB;
                    }

                    TxBuffer[TxHeader--]=params.MCPS_DATA_request.msduHandle;

                    /* frame control bytes */
                    TxBuffer[TxHeader]= params.MCPS_DATA_request.DstAddrMode << 2;
                    TxBuffer[TxHeader]|= (params.MCPS_DATA_request.SrcAddrMode << 6);
                    TxHeader--;

                    TxBuffer[TxHeader]= params.MCPS_DATA_request.frameType;

                    macTasksPending.bits.packetPendingAck = 0;

                    if(intraPAN == TRUE)
                    {
                        /* set intra-pan bit */
                        TxBuffer[TxHeader]|=0x40;
                    }

                    if(params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission)
                    {
                        /* ACK transmission requested */
                        TxBuffer[TxHeader]|=0x20;
                        // macTasksPending.bits.packetPendingAck - this bit is set only if it is actually going out
                    }

                    /* our stack doesn't do beaconed networks so GTS is ignored */
                #ifdef I_SUPPORT_SECURITY_SPEC
                    if(params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission)
                    {
                        /* security enabled */
                        TxBuffer[TxHeader]|=0x08;
                    }
                #endif
                    TxHeader--;

                    /* check to see if the packet is too large */
                    if(TxHeader < TxData)
                    {
                        /* packet is too large to send, delete it */
                        //TxHeader = TX_HEADER_START;
                        //TxData = TX_DATA_START;
                        ZigBeeUnblockTx();
                    }
                    else
                    {
                        /* if was requested for indirect transmission */
                        #if !defined(I_AM_END_DEVICE)
                            if(params.MCPS_DATA_request.TxOptions.bits.indirect_transmission)
                            {
                                BYTE i,k;
                                MAC_INDIRECT_BUFFER macBufferInfo;
                                BYTE* packet;

                                /* search to find an open indirect buffer */
                                for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
                                {
                                    macBufferInfo = macIndirectBuffers[i];
                                    if(macBufferInfo.buffer == NULL)
                                    {
                                        goto Found_Open_Buffer;
                                    }
                                }
                                /* didn't find an open buffer */
    Return_TRANSACTION_OVERFLOW:
                                if(currentPacket.info.bits.RX_association == 0)
                                {
                                    ZigBeeUnblockTx();
                                    params.MCPS_DATA_confirm.status = TRANSACTION_OVERFLOW;
                                    //currentPacket.info.bits.RX_association = 0;
                                    //The following parameters overlap MCPS_DATA_request and thus are already loaded
                                    //params.MCPS_DATA_confirm.msduHandle = MCPS_DATA_request.msduHandle
                                    return MCPS_DATA_confirm;
                                }
                                else
                                {
                                    ZigBeeUnblockTx();
                                    params.MLME_COMM_STATUS_indication.status = TRANSACTION_OVERFLOW;
                                    //The following parameters overlap MCPS_DATA_request and thus are already loaded
                                    //params.MLME_COMM_STATUS_indication.DstAddr = MCPS_DATA_request.DstAddr
                                    //all others don't really matter
                                    return MLME_COMM_STATUS_indication;
                                }
    Found_Open_Buffer:
                                /* found an open buffer */

                                /* if there is enough room for the TICK and packet pointer */
                                macBufferInfo.buffer = (MAC_INDIRECT_BUFFER_INFO*)SRAMalloc(sizeof(MAC_INDIRECT_BUFFER_INFO));
                                if(macBufferInfo.buffer == NULL)
                                {
                                    /* not enough room for buffer, let alone the packet */
                                    goto Return_TRANSACTION_OVERFLOW;
                                }

                                /* if there is then alloc enough room for the packet */
                                k=(TX_HEADER_START-TxHeader)+TxData;
                                macBufferInfo.buffer->HeaderSize = TX_HEADER_START-TxHeader;

                                macBufferInfo.buffer->packet = SRAMalloc(k);
                                packet = macBufferInfo.buffer->packet;
                                if(packet == NULL)
                                {
                                    /* not enough room for the packet */
                                    goto Return_TRANSACTION_OVERFLOW;
                                }

                                macBufferInfo.buffer->size = k;

                                /* copy the TXBuffer into the allocated buffer.packet */
                                while(++TxHeader <= TX_HEADER_START)
                                {
                                    *packet++ = TxBuffer[TxHeader];
                                }
                                TxHeader = TX_HEADER_START;
                                {
                                    BYTE j;

                                    for(j=0;j<TxData;j++)
                                    {
                                        /* TxData is always pointing to next empty byte */
                                        *packet++ = TxBuffer[j];
                                    }
                                    TxData = 0;
                                }
                                /* done buffering the packet */
                                /* record when the packet was buffered */
                                macBufferInfo.buffer->timeStamp = TickGet();

                                /* copy the buffer back to the array */
                                macIndirectBuffers[i].buffer = macBufferInfo.buffer;

                                /* mark bit indicating that we are waiting on an indirect tx */
                                macTasksPending.bits.indirectPackets = 1;

                                ZigBeeUnblockTx();

                                /* return NO_PRIMITIVE because this packet is not done but we have nothing left to do */
                                return NO_PRIMITIVE;
                            }
                            else
                        #else
                            /* if we are an RFD then always transmit the packet */
                        #endif
SendTxBuffer:
                        {   //need this bracket for the #ifdef with the hanging else

                            BYTE TxStatus, test;
                            #ifdef I_SUPPORT_SECURITY_SPEC
                                    BOOL bSecurity = FALSE;
                                    BYTE ActiveKeyIndex;

                                    GetNwkActiveKeyNumber(&ActiveKeyIndex);
                            #endif

                            TxStatus = PHYGetLongRAMAddr(0x20f);

                            ZigBeeBlockTx();    //    TxFIFOControl.bFIFOInUse = 1; TODO should already be blocked...

                            if(TxStatus == 0x80)
                            {
                                //TX is busy, can't send it
                                macTasksPending.bits.dataInBuffer = 1;
                                return NO_PRIMITIVE;
                            }
                            macTasksPending.bits.dataInBuffer = 0;

                            /* now send the packet to the TRX and try to send it */

                            TxStat.ack = 0;
                            /* Is it an ack? */
                            if (TxBuffer[TxHeader + 1] & 0x20)
                            {
                                // We are requesting an ack
                                TxStat.ack = 1;
                                macTasksPending.bits.packetPendingAck = 1;
                            }

                            #ifdef I_SUPPORT_SECURITY_SPEC
                                if (params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission)
                                {
                                    if(ActiveKeyIndex == 0x01 || ActiveKeyIndex == 0x02)
                                    {
                                        // This packet need secured
                                        bSecurity = TRUE;
                                    } else {
                                        TxBuffer[TxHeader + 1] &= 0xf7;
                                    }
                                }
                            #endif

                            PHYSetLongRAMAddr(0x000, TX_HEADER_START-TxHeader);
                            // Packet Length
                            #ifdef I_SUPPORT_SECURITY_SPEC
                                if( bSecurity )
                                {
                                    PHYSetLongRAMAddr(0x001, (TX_HEADER_START-TxHeader)+TxData+5);
                                } else
                            #endif
                            {
                                PHYSetLongRAMAddr(0x001, (TX_HEADER_START-TxHeader)+TxData);
                            }

                            {
                                WORD fifoloc = 0x002;

                                while(++TxHeader <= TX_HEADER_START)
                                {
                                    PHYSetLongRAMAddr (fifoloc++, TxBuffer[TxHeader]);
                                }

                                TxHeader = TX_HEADER_START;
                                {
                                    BYTE j;
                                    #ifdef I_SUPPORT_SECURITY_SPEC
                                        if( bSecurity )
                                        {
                                            for(j = 0; j < 4; j++)
                                            {
                                                PHYSetLongRAMAddr(fifoloc++, OutgoingFrameCount[ActiveKeyIndex-1].v[j]);
                                            }
                                            OutgoingFrameCount[ActiveKeyIndex-1].Val++;
                                            #ifdef USE_EXTERNAL_NVM
                                                currentNetworkKeyInfo = plainSecurityKey[ActiveKeyIndex-1];
                                            #else
                                                GetNwkKeyInfo( &currentNetworkKeyInfo, &(networkKeyInfo[ActiveKeyIndex-1]) );
                                            #endif
                                            PHYSetLongRAMAddr(fifoloc++, currentNetworkKeyInfo.SeqNumber.v[0]);
                                        }
                                    #endif
                                    for(j=0;j<TxData;j++)
                                    {
                                        /* TxData is always pointing to next empty byte */
                                        PHYSetLongRAMAddr (fifoloc++, TxBuffer[j]);
                                    }
                                    TxData = 0;
                                }
                            }

                            ZigBeeBlockTx();
                            PHYTasksPending.bits.PHY_TX = 1;

                            #ifdef I_SUPPORT_SECURITY_SPEC
                                if( bSecurity )
                                {
                                    WORD loc;
                                    BYTE i;

                                    // set the security key
                                    loc = 0x280;
                                    for(i = 0; i < 16; i++)
                                    {
                                        PHYSetLongRAMAddr(loc++, currentNetworkKeyInfo.NetKey.v[i]);
                                    }

                                    // set the cipher mode
                                    PHYSetShortRAMAddr(0x2c, SecurityLevel_ZIGBEE_2_IEEE(nwkSecurityLevel));

                                    // trigger the transition
                                    if( TxStat.ack )
                                    {
                                        PHYSetShortRAMAddr(0x1b, 0x07);
                                    } else {
                                        PHYSetShortRAMAddr(0x1b, 0x03);
                                    }
                                } else
                            #endif
                            {
                                /* Transmit the packet in the FIFO  */
                                if (TxStat.ack)
                                    PHYSetShortRAMAddr(0x1b, 0x05);
                                else
                                    PHYSetShortRAMAddr (0x1b, 0x01);

                            }

                            currentPacket.startTime = TickGet();
                            currentPacket.info.bits.retries = 0;

                            /* wait for ACK */
                            if(params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission != 1)
                            {
                                macTasksPending.bits.packetPendingAck = 0;
                                ZigBeeUnblockTx();  //TxFIFOControl.bFIFOInUse = 0;
                                params.MCPS_DATA_confirm.status = SUCCESS;
                                params.MCPS_DATA_confirm.msduHandle = currentPacket.sequenceNumber; //OldACKSeqNum;
                                return MCPS_DATA_confirm;
                            }
                        }

                    }
                }
                break;

            } // End MCPS_DATA_request

#ifndef REMOVE_MCPS_PURGE_request
            case MCPS_PURGE_request:
                {
                    BYTE i,*p;

                    params.MCPS_PURGE_confirm.status = MAC_INVALID_HANDLE;
                    for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
                    {
                        p = macIndirectBuffers[i].buffer->packet;
                        if( *(p+2) == params.MCPS_PURGE_request.msduHandle)
                        {
                            SRAMfree((void *)macIndirectBuffers[i].buffer->packet);
                            SRAMfree((void *)macIndirectBuffers[i].buffer);

                            while(i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER)-1))
                            {
                                macIndirectBuffers[i].buffer = macIndirectBuffers[i+1].buffer;
                                i++;
                            }

                            /* the last entry is now null because we removed one */
                            macIndirectBuffers[i].buffer = NULL;

                            params.MCPS_PURGE_confirm.status = SUCCESS;
                            break;
                        }
                    }
                    //the following primitive is unneeded because the memory overlaps
                    //MCPS_PURGE_confirm.msduHandle = MCPS_PURGE_request.msduHandle
                    return MCPS_PURGE_confirm;
                }
                break;
#endif

            case MLME_ASSOCIATE_request:
            {

                /* choose appropriate channel */
                phyPIB.phyCurrentChannel = params.MLME_ASSOCIATE_request.LogicalChannel;
                PHYSetLongRAMAddr(0x200, (0x02 | (BYTE)((phyPIB.phyCurrentChannel - 11) << 4)));
                PHYSetShortRAMAddr(PWRCTL,0x04);
                PHYSetShortRAMAddr(PWRCTL,0x00);

                // set the macPANid field
                macPIB.macPANId = params.MLME_ASSOCIATE_request.CoordPANId;
                MLME_SET_macPANId_hw();

                // send association request
                TxBuffer[TxData++] = ASSOCIATION_REQUEST;
                TxBuffer[TxData++] = params.MLME_ASSOCIATE_request.CapabilityInformation.Val;

                //source addr
                TxHeader -= 7;
                GetMACAddress( &(TxBuffer[TxHeader]) );
                TxHeader --;

                // source pan
                TxBuffer[TxHeader--] = 0xff;
                TxBuffer[TxHeader--] = 0xff;

                //dest addr
            #ifdef ZCP_DEBUG
                if( params.MLME_ASSOCIATE_request.CoordAddrMode == 0x03 )
                {
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[7];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[6];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[5];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[4];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[3];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[2];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[1];
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.LongAddr.v[0];
                    associationReqParams.CoordAddress.Val = 0xfffe;
                } else
            #endif
                {
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.ShortAddr.byte.MSB;
                    TxBuffer[TxHeader--] = params.MLME_ASSOCIATE_request.CoordAddress.ShortAddr.byte.LSB;
                    associationReqParams.CoordAddress.Val = params.MLME_ASSOCIATE_request.CoordAddress.ShortAddr.Val;
                }


                //dest pan
                TxBuffer[TxHeader--] = macPIB.macPANId.byte.MSB;
                TxBuffer[TxHeader--] = macPIB.macPANId.byte.LSB;

                currentPacket.sequenceNumber = macPIB.macDSN;

                TxBuffer[TxHeader--] = macPIB.macDSN++;
            #ifdef ZCP_DEBUG
                if( params.MLME_ASSOCIATE_request.CoordAddrMode == 0x03 )
                    TxBuffer[TxHeader--] = 0xCC;
                else
            #endif
                TxBuffer[TxHeader--] = 0xC8;
                TxBuffer[TxHeader--] = 0x23;
                #ifdef I_SUPPORT_SECURITY_SPEC
                    if( params.MLME_ASSOCIATE_request.SecurityEnable )
                    {
                        TxBuffer[TxHeader+1] |= 0x08;
                        params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = 1;
                    }
                #endif

                currentPacket.info.Val = 0;
                params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 1;
                currentPacket.info.bits.association = 1;
                goto SendTxBuffer;
            }   // End MLME_associate_request

            case MLME_ASSOCIATE_response:
            {
                /* result of an association request */
                /* generate the association response command packet */

                TxBuffer[TxData++]=ASSOCIATION_RESPONSE;
                TxBuffer[TxData++]=params.MLME_ASSOCIATE_response.AssocShortAddress.byte.LSB;
                TxBuffer[TxData++]=params.MLME_ASSOCIATE_response.AssocShortAddress.byte.MSB;
                TxBuffer[TxData++]=params.MLME_ASSOCIATE_response.status;

                #ifdef I_SUPPORT_SECURITY_SPEC
                    if(params.MLME_ASSOCIATE_response.SecurityEnable == TRUE)
                    {
                        params.MCPS_DATA_request.TxOptions.Val = 0b00001101;
                    }
                    else
                #endif
                {
                    params.MCPS_DATA_request.TxOptions.Val = 0b00000101;
                }

                params.MCPS_DATA_request.DstAddrMode = 3;
                /* params.MCPS_DATA_request.DstAddr already aligns with params.MLME_ASSOCIATE_response.DeviceAddr so we don't need to copy it */
                //still need to copy the long address into the current packet information section
                currentPacket.DstAddr.v[0] = params.MLME_ASSOCIATE_response.DeviceAddress.v[0];
                currentPacket.DstAddr.v[1] = params.MLME_ASSOCIATE_response.DeviceAddress.v[1];
                currentPacket.DstAddr.v[2] = params.MLME_ASSOCIATE_response.DeviceAddress.v[2];
                currentPacket.DstAddr.v[3] = params.MLME_ASSOCIATE_response.DeviceAddress.v[3];
                currentPacket.DstAddr.v[4] = params.MLME_ASSOCIATE_response.DeviceAddress.v[4];
                currentPacket.DstAddr.v[5] = params.MLME_ASSOCIATE_response.DeviceAddress.v[5];
                currentPacket.DstAddr.v[6] = params.MLME_ASSOCIATE_response.DeviceAddress.v[6];
                currentPacket.DstAddr.v[7] = params.MLME_ASSOCIATE_response.DeviceAddress.v[7];

                params.MCPS_DATA_request.DstPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                params.MCPS_DATA_request.DstPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                params.MCPS_DATA_request.SrcAddrMode = 0x03;
                GetMACAddress( &params.MCPS_DATA_request.SrcAddr.LongAddr );

                params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                currentPacket.sequenceNumber = macPIB.macDSN;

                params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;

                params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_CMD;

                currentPacket.info.Val = 0;
                currentPacket.info.bits.RX_association = 1;
                goto MCPS_DATA_request_label_skip_init;
            }   // End MLME_Associate_response

            case MLME_DISASSOCIATE_request:
            {
            #ifndef ZCP_DEBUG
                if ((params.MLME_DISASSOCIATE_request.DeviceAddress.v[0] == macPIB.macCoordExtendedAddress.v[0]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[1] == macPIB.macCoordExtendedAddress.v[1]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[2] == macPIB.macCoordExtendedAddress.v[2]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[3] == macPIB.macCoordExtendedAddress.v[3]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[4] == macPIB.macCoordExtendedAddress.v[4]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[5] == macPIB.macCoordExtendedAddress.v[5]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[6] == macPIB.macCoordExtendedAddress.v[6]) &&
                    (params.MLME_DISASSOCIATE_request.DeviceAddress.v[7] == macPIB.macCoordExtendedAddress.v[7]))
            #endif
                {
                    if( params.MLME_DISASSOCIATE_request.DisassociateReason == SELF_INITIATED_LEAVE )
                        currentPacket.info.bits.disassociation = 1;

                    TxBuffer[TxData++]=DISASSOCIATION_NOTIFICATION;
                    TxBuffer[TxData++]=params.MLME_DISASSOCIATE_request.DisassociateReason;

                #ifdef I_SUPPORT_SECURITY_SPEC
                    if(params.MLME_DISASSOCIATE_request.SecurityUse == TRUE)
                    {
                        params.MCPS_DATA_request.TxOptions.Val = 0b00001001;
                    }
                    else
                #endif
                    {
                        params.MCPS_DATA_request.TxOptions.Val = 0b00000001;
                    }

                #ifdef ZCP_DEBUG
                    params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 1;
                    if (NWKLookupNodeByLongAddr( &(params.MCPS_DATA_request.DstAddr.LongAddr) ) != INVALID_NEIGHBOR_KEY)
                    {
                        if (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)
                        {
                            params.MCPS_DATA_request.TxOptions.bits.indirect_transmission = 1;
                        }
                    }
                #endif

// this code is not needed because the DeviceAddress is already aligned with the DstAddr.
//                    //copy out the destination address
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[0] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[0];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[1] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[1];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[2] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[2];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[3] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[3];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[4] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[4];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[5] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[5];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[6] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[6];
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[7] = params.MLME_DISASSOCIATE_request.DeviceAddress.v[7];
//
                    params.MCPS_DATA_request.DstAddrMode = 3;

                    params.MCPS_DATA_request.DstPANId.Val = macPIB.macPANId.Val;

                    //params.MCPS_DATA_request.DstPANId.Val = 0xFFFF;
/*#ifdef ZCP_DEBUG
                    if( bDisAssocShort )
                    {
                        params.MCPS_DATA_request.SrcAddrMode = 0x02;
                        params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.LSB = macPIB.macShortAddress.byte.LSB;
                        params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.MSB = macPIB.macShortAddress.byte.MSB;
                    } else
#endif*/
                    {
                        params.MCPS_DATA_request.SrcAddrMode = 0x03;
                        GetMACAddress( &params.MCPS_DATA_request.SrcAddr.LongAddr );
                    }

                    params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                    params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                    currentPacket.sequenceNumber = macPIB.macDSN;
                    params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;

                    params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_CMD;

                    goto MCPS_DATA_request_label_skip_init;
                }
            #ifndef ZCP_DEBUG
                else
                {
                    //Zigbee changed the way that the IEEE disassociate works.  Now only sending your parents address is valid.
                    params.MLME_DISASSOCIATE_confirm.status = INVALID_PARAMETER;
                    return MLME_DISASSOCIATE_confirm;
//                    #if defined(I_AM_COORDINATOR) || defined(I_AM_ROUTER)
//                        currentPacket.info.bits.disassociation = 1;
//                        // TODO: send using indirect transmission???  This seems like it wouldn't work if the device is an FFD //
//                    #else
//                        params.MLME_DISASSOCIATION_confirm.Status = INVALID_PARAMETER;
//                    #endif
                }
            #endif
                break;
            }   // End MLME-Disassociate-request

            /* User will manipulate the variables themselves
            case MLME_GET_request:
                break;          */

            /*  Not used in non-beacon networks
            case MLME_GTS_request:
                break;      */

            case MLME_ORPHAN_response:
            {
                BYTE i;
                //This code has been removed because it is not needed if MLME_ORPHAN_response is not called when
                //associatedMember = FALSE.  I have removed both of these sections
//                if(params.MLME_ORPHAN_response.AssociatedMember == FALSE)
//                {
//                    //no primitive
//                    break;
//                }

                //if it is a memeber of this ieee coordinator then send a coordinator realignment
                TxBuffer[TxData++]=COORDINATOR_REALIGNMENT;
                TxBuffer[TxData++]=macPIB.macPANId.byte.LSB;
                TxBuffer[TxData++]=macPIB.macPANId.byte.MSB;
                TxBuffer[TxData++]=macPIB.macShortAddress.byte.LSB;
                TxBuffer[TxData++]=macPIB.macShortAddress.byte.MSB;
                TxBuffer[TxData++]=phyPIB.phyCurrentChannel;
                TxBuffer[TxData++]=params.MLME_ORPHAN_response.ShortAddress.byte.LSB;
                TxBuffer[TxData++]=params.MLME_ORPHAN_response.ShortAddress.byte.MSB;

                if(params.MLME_ORPHAN_response.SecurityEnable == TRUE)
                {
                    params.MCPS_DATA_request.TxOptions.Val = 0b00001001;
                }
                else
                {
                    params.MCPS_DATA_request.TxOptions.Val = 0b00000001;
                }

                //copy out the destination address
                for(i=0; i<8; i++)
                {
                   params.MCPS_DATA_request.DstAddr.LongAddr.v[i] = params.MLME_ORPHAN_response.OrphanAddress.v[i];
                }
                params.MCPS_DATA_request.DstAddrMode = 3;

                params.MCPS_DATA_request.DstPANId.Val = 0xFFFF;

                params.MCPS_DATA_request.SrcAddrMode = 0x03;
                GetMACAddress( &params.MCPS_DATA_request.SrcAddr.LongAddr );

                params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                currentPacket.sequenceNumber = macPIB.macDSN;
                params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;

                params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_CMD;

                currentPacket.info.Val = 0;
                currentPacket.info.bits.RX_association = 1;
                for(i = 0; i < 8; i++)
                {
                    currentPacket.DstAddr.v[i] = params.MCPS_DATA_request.DstAddr.LongAddr.v[i];
                }

                goto MCPS_DATA_request_label_skip_init;
            }   // End MLME_ORPHAN_response

            case MLME_RESET_request:
            {
                if(params.MLME_RESET_request.SetDefaultPIB==TRUE)
                {
                    macPIB.macPANId.Val = 0xFFFF;
                    macPIB.macShortAddress.Val = 0xFFFF;
                    phyPIB.phyCurrentChannel = 11;
                    macPIB.macAssociationPermit = FALSE;
                    macPIB.macMaxCSMABackoffs = 4;
                    macPIB.macMinBE = 3;
                }

                macStatus.bits.allowBeacon = 0;

                TxHeader = TX_HEADER_START;
                TxData = TX_DATA_START;

                macTasksPending.Val = 0;
                currentPacket.info.Val = 0;

                macStatus.Val = 0;

                MACEnable();

                {
                    BYTE i;

                    /* clear the indirect buffers */
                    for(i=0;i<(sizeof(macIndirectBuffers)/sizeof(MAC_INDIRECT_BUFFER));i++)
                    {
                        if(macIndirectBuffers[i].buffer != NULL)
                        {
                            free(macIndirectBuffers[i].buffer);
                        }
                    }
                }

                params.MLME_RESET_confirm.status = SUCCESS;
                return MLME_RESET_confirm;
            }

            /*  Rx should be enabled by default- not useful in non-beacon networks
            case MLME_RX_ENABLE_request:
                break;  */

            case MLME_SCAN_request:
            {
                if(macTasksPending.bits.scanInProgress)
                {
MLME_SCAN_request_INVALID_PARAMETER:
                    params.MLME_SCAN_confirm.status = INVALID_PARAMETER;
                    return MLME_SCAN_confirm;
                }
                if(params.MLME_SCAN_request.ScanDuration>14)
                {
                    goto MLME_SCAN_request_INVALID_PARAMETER;
                }
                //we can remove this code if we assume that no one will call incorrectly, uncomment if you are really worried about it
//                if(params.MLME_SCAN_request.ScanType>3)
//                {
//                    goto MLME_SCAN_request_INVALID_PARAMETER;
//                }

                scanParams.results = NULL;

                scanParams.ResultListSize = 0;
                scanParams.ScanType = params.MLME_SCAN_request.ScanType;
                scanParams.Channels.Val = params.MLME_SCAN_request.ScanChannels.Val;
                scanParams.UnscannedChannels.Val = scanParams.Channels.Val;
                scanParams.Channels.Val &= POSSIBLE_CHANNEL_MASK;

                //This is required so that the first iteration of the scan checks channel 0
                phyPIB.phyCurrentChannel = 0xFF;

                /* for all scans other than orphan scan it will take aBaseSuperframeDuration * (2^n + 1) */

                scanParams.ScanDuration = params.MLME_SCAN_request.ScanDuration;
                scanParams.numScannedChannels = 0;

                switch(params.MLME_SCAN_request.ScanType)
                {
                    case MAC_SCAN_ENERGY_DETECT:
                        #if defined(I_AM_FFD) || defined(INCLUDE_ED_SCAN)
                            scanParams.ScanEndTime.Val = (DWORD)timeToAdd[scanParams.ScanDuration];
                            scanParams.results = SRAMalloc(27); //alloc a byte for every possible channel scanned
                            scanParams.currentResults = scanParams.results;
                            if(scanParams.results == NULL)
                            {
                                params.MLME_SCAN_confirm.status = SUCCESS;
                                params.MLME_SCAN_confirm.UnscannedChannels = params.MLME_SCAN_request.ScanChannels;
                                params.MLME_SCAN_confirm.ResultListSize = 0;
                                return MLME_SCAN_confirm;
                            }
                        #else
                            params.MLME_SCAN_confirm.status = INVALID_PARAMETER;
                            return MLME_SCAN_confirm;
                        #endif
                        break;
                    case MAC_SCAN_ACTIVE_SCAN:
                    case MAC_SCAN_PASSIVE_SCAN:
                    {
                        BYTE    test;
                        test = PHYGetShortRAMAddr(0x00);
                        test = test | 0x01;
                        //disable PAN address filtering for beacon frames
                        PHYSetShortRAMAddr(0x00, test);

                        scanParams.ScanEndTime.Val = (DWORD)timeToAdd[scanParams.ScanDuration];

                        break;
                    }
                    case MAC_SCAN_ORPHAN_SCAN:
                        scanParams.ScanEndTime.Val = SYMBOLS_TO_TICKS(aResponseWaitTime) + 30; /* +30 to make up for the time to Tx the packet */
                        break;

                    default:
                        break;

                }

                macTasksPending.bits.channelScanning = 0;
                macTasksPending.bits.scanInProgress = 1;

                break;
            }   // End MLME_SCAN_request

            /* User will manipulate the variables themselves
            case MLME_SET_request:
                break;      */

            case MLME_START_request:
            {
                if(macPIB.macShortAddress.Val == 0xffff)
                {
                    params.MLME_START_confirm.status = NO_SHORT_ADDRESS;
                    return MLME_START_confirm;
                }

                if(params.MLME_START_request.fields.bits.CoordRealignment)
                {
                    /* TODO: first send coord realignment */
                }

                if(params.MLME_START_request.fields.bits.PANCoordinator)
                {
                    macPIB.macPANId = params.MLME_START_request.PANId;
                    MLME_SET_macPANId_hw();
                    phyPIB.phyCurrentChannel = params.MLME_START_request.LogicalChannel;

                    /* choose appropriate channel */
                    PHYSetLongRAMAddr(0x200, (0x02 | (BYTE)((phyPIB.phyCurrentChannel - 11) << 4)));
                    PHYSetShortRAMAddr(PWRCTL,0x04);
                    PHYSetShortRAMAddr(PWRCTL,0x00);
                }

                macStatus.bits.allowBeacon = 1;

                params.MLME_START_confirm.status = SUCCESS;
                return MLME_START_confirm;
            }   // End MLME-START.request

            /* Primative not used in non-beacon networks
            case MLME_SYNC_request:
                break;                              */

            case MLME_POLL_request:
            {
                if (!ZigBeeReady())
                {
                    // There is not a good return code available for this issue...
                    params.MLME_POLL_confirm.status = INVALID_PARAMETER;
                    return MLME_POLL_confirm;
                }

                ZigBeeBlockTx();
                TxBuffer[TxData++]=DATA_REQUEST;

                if(params.MLME_POLL_request.SecurityEnabled == TRUE)
                {
                    params.MCPS_DATA_request.TxOptions.Val = 0b00001001;
                }
                else
                {
                    params.MCPS_DATA_request.TxOptions.Val = 0b00000001;
                }


            #ifdef ZCP_DEBUG
                if( params.MCPS_DATA_request.DstAddrMode == 0 )
                {
                    params.MCPS_DATA_request.DstPANId.Val = 0x1234;
                }
            #endif
                params.MCPS_DATA_request.DstAddrMode = params.MLME_POLL_request.CoordAddrMode;

                if(params.MLME_POLL_request.CoordAddrMode == 0x02)
                {
                    // This is per spec, but kind of silly...
                    //if(params.MLME_POLL_request.CoordAddress.ShortAddr.Val == 0x0000)
                    //{
                    //    params.MCPS_DATA_request.DstAddrMode = 0x00;
                    //}
                    // the following code is not needed because all variables align with the data request
//                    params.MCPS_DATA_request.DstAddr.ShortAddr.byte.MSB = params.MLME_POLL_request.CoordAddress.ShortAddr.byte.MSB
//                    params.MCPS_DATA_request.DstAddr.ShortAddr.byte.LSB = params.MLME_POLL_request.CoordAddress.ShortAddr.byte.LSB

                }
//                else
//                {
                        // the following code is not needed because all variables align with the data request
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[0] = params.MLME_POLL_request.CoordAddress.LongAddr.v[0]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[1] = params.MLME_POLL_request.CoordAddress.LongAddr.v[1]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[2] = params.MLME_POLL_request.CoordAddress.LongAddr.v[2]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[3] = params.MLME_POLL_request.CoordAddress.LongAddr.v[3]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[4] = params.MLME_POLL_request.CoordAddress.LongAddr.v[4]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[5] = params.MLME_POLL_request.CoordAddress.LongAddr.v[5]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[6] = params.MLME_POLL_request.CoordAddress.LongAddr.v[6]
//                    params.MCPS_DATA_request.DstAddr.LongAddr.v[7] = params.MLME_POLL_request.CoordAddress.LongAddr.v[7]
//                }

                /* params.MCPS_DATA_request.DstAddr already aligns with params.MLME_ASSOCIATE_response.DeviceAddr so we don't need to copy it */

                params.MCPS_DATA_request.SrcPANId.byte.MSB = macPIB.macPANId.byte.MSB;
                params.MCPS_DATA_request.SrcPANId.byte.LSB = macPIB.macPANId.byte.LSB;

                #ifdef I_AM_COORDINATOR
                if (!ZigBeeStatus.flags.bits.bNetworkFormed)
                #else
            #ifdef ZCP_DEBUG
                if (!ZigBeeStatus.flags.bits.bNetworkJoined || bDRLong)
            #else
                if(!ZigBeeStatus.flags.bits.bNetworkJoined)
            #endif
                #endif
//                if((macPIB.macPANId.Val == 0xFFFF) || (macTasksPending.bits.associationPending==1))
                {
                    #ifdef ZCP_DEBUG
                        bDRLong = FALSE;
                    #endif
                    params.MCPS_DATA_request.SrcAddrMode = 0x03;
                    GetMACAddress( &params.MCPS_DATA_request.SrcAddr.LongAddr );
                }
                else
                {
                    currentPacket.info.Val = 0;
                    currentPacket.info.bits.data_request = 1;
                    {
                    #ifdef ZCP_DEBUG
                        if( macPIB.macShortAddress.Val == 0xfffe )
                        {
                            params.MCPS_DATA_request.SrcAddrMode = 0x03;
                            GetMACAddress(&(params.MCPS_DATA_request.SrcAddr.LongAddr));
                        } else
                    #endif
                        {
                            params.MCPS_DATA_request.SrcAddrMode = 0x02;
                            params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.LSB = macPIB.macShortAddress.byte.LSB;
                            params.MCPS_DATA_request.SrcAddr.ShortAddr.byte.MSB = macPIB.macShortAddress.byte.MSB;
                        }
                    }
                }

                params.MCPS_DATA_request.DstPANId.byte.MSB = params.MLME_POLL_request.CoordPANId.byte.MSB;
                params.MCPS_DATA_request.DstPANId.byte.LSB = params.MLME_POLL_request.CoordPANId.byte.LSB;

                currentPacket.sequenceNumber = macPIB.macDSN;
                currentPacket.startTime = TickGet();

                params.MCPS_DATA_request.msduHandle = macPIB.macDSN++;

                params.MCPS_DATA_request.frameType = MAC_FRAME_TYPE_CMD;
                PHYTasksPending.bits.PHY_DATA_REQUEST = 1;

                goto MCPS_DATA_request_label_skip_init;
            }   // End MLME_POLL_REQUEST
        }
    }
    return NO_PRIMITIVE;
}

/*********************************************************************
 * Function:        void MLME_SET_macPANId_hw( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine sets the transceiver's PAN ID to the
 *                  value in macPIB.macPANId.
 *
 * Note:            This function must be called after macPANId is set
 *                  for the PAN ID to be set in hardware.
 ********************************************************************/
void MLME_SET_macPANId_hw( void )
{
    PHYSetShortRAMAddr(0x01, macPIB.macPANId.byte.LSB);
    PHYSetShortRAMAddr(0x02, macPIB.macPANId.byte.MSB);
}

/*********************************************************************
 * Function:        void MLME_SET_macShortAddress_hw( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine sets the transceiver's short address to the
 *                  value in macPIB.macShortAddress.
 *
 * Note:            This function must be called after macShortAddress is set
 *                  for the short address to be set in hardware.
 ********************************************************************/
void MLME_SET_macShortAddress_hw( void )
{
    PHYSetShortRAMAddr(0x03, macPIB.macShortAddress.byte.LSB);
    PHYSetShortRAMAddr(0x04, macPIB.macShortAddress.byte.MSB);
}

#else
    #error Please link the appropriate MAC file for the selected transceiver.
#endif
