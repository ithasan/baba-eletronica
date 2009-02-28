/*********************************************************************
 *
 *                  MAC Header File for the UBEC 2400
 *
 *********************************************************************
 * FileName:        zMAC_UZ2400.h
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
 * DF/KO                01/09/06 Microchip ZigBee Stack v1.0-3.5
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
********************************************************************/

typedef struct _currentPacket
{
    BYTE sequenceNumber;
    TICK startTime;
    union _currentPacket_info
    {
        struct _currentPacket_info_bits
        {
            unsigned int retries :3;
            unsigned int association :1;   //this bit indicates if the packet was an association packet (results go to COMM_STATUS) or a normal packet (results go to DATA_confirm)
            unsigned int data_request :1;   //this bit indicates if the packet was an data request packet (results go to MLME_POLL_confirm)
            unsigned int expecting_data :1; //this bit indicates that a POLL_request got an ack back and the framePending bit was set
            unsigned int RX_association :1;
            unsigned int disassociation :1;  //this bit indicates that a MLME_DISASSOCIATION_confirm is required
        } bits;
        BYTE Val;
    } info;
    LONG_ADDR DstAddr;  //the destination of current packet if it was an association request or coordinator realignment
} CURRENT_PACKET;

typedef union _MAC_BACKGROUND_TASKS_PENDING
{
    BYTE Val;
    struct _background_bits
    {
        unsigned int indirectPackets :1;
        unsigned int packetPendingAck :1;
        unsigned int scanInProgress :1;
        unsigned int associationPending :1;
        unsigned int dataInBuffer :1;
        unsigned int bSendUpMACConfirm : 1;
        unsigned int channelScanning :1;
    } bits;
} MAC_TASKS_PENDING;

#define POSSIBLE_CHANNEL_MASK 0x07FFF800

void MACEnable(void);
void MACDisable(void);

