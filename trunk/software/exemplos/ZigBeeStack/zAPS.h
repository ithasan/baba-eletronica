/*********************************************************************
 *
 *                  ZigBee APS Header File
 *
 *********************************************************************
 * FileName:        zAPS.h
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
 * DF/KO                08/31/06 Microchip ZigBee Stack v1.0-3.6
 * DF/KO/YY				11/27/06 Microchip ZigBee Stack v1.0-3.7
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/

#ifndef _zAPS_H_
#define _zAPS_H_

// zAPS.h

// ******************************************************************************
// APS Layer Spec Constants

// The maximum number of Address Map entries.
#define apscMaxAddrMapEntries       MAX_APS_ADDRESSES

// The maximum number of octets contained in a non-complex descriptor.
#define apscMaxDescriptorSize       64

// The maximum number of octets that can be returned through the discovery process.
#define apscMaxDiscoverySize        64

// The maximum number of octets added by the APS sub-layer to its payload.
#ifdef I_SUPPORT_SECURITY
    #define apscMaxFrameOverhead    20
#else
    #define apscMaxFrameOverhead    6
#endif

// The maximum number of retries allowed after a transmission failure.
#define apscMaxFrameRetries         3

// The maximum number of octets that can be transmitted in the APS frame payload
// field (see [R3]).
#define apscMaxPayloadSize          (nwkcMaxPayloadSize –apscMaxFrameOverhead)

// The maximum number of seconds to wait for an acknowledgement to a transmitted frame.
#ifdef I_SUPPORT_SECURITY
    #define apscAckWaitDuration     (ONE_SECOND * (0.05 * (2*nwkcMaxDepth) + 0.1))
    // authorization time out, defined by profile
    #define AUTHORIZATION_TIMEOUT		(ONE_SECOND * 0.7)
#else
    #define apscAckWaitDuration     (ONE_SECOND * (0.05 * (2*nwkcMaxDepth)))
#endif


// ******************************************************************************
// Constant Definitions and Enumerations

#define APS_ADDRESS_MAP_VALID       0xC0DE
#define APS_ADDRESS_NOT_PRESENT     0x00
#define APS_ADDRESS_16_BIT          0x01
#define APS_ADDRESS_64_BIT          0x02

typedef enum _APS_STATUS_VALUES
{
    APS_SUCCESS             = 0x00,
    APS_NO_BOUND_DEVICE,    //0x01
    APS_SECURITY_FAIL,      //0x02
    APS_NO_ACK,             //0x03
    APS_INVALID_REQUEST     //0x04
} APS_STATUS_VALUES;

typedef enum _APS_COMMANDS
{
	APS_CMD_SKKE_1 = 0x01,
	APS_CMD_SKKE_2,
	APS_CMD_SKKE_3,
	APS_CMD_SKKE_4,
	APS_CMD_TRANSPORT_KEY,
	APS_CMD_UPDATE_DEVICE,
	APS_CMD_REMOVE_DEVICE,
	APS_CMD_REQUEST_KEY,
	APS_CMD_SWITCH_KEY
} APS_COMMANDS;


#define BIND_SUCCESS                0x00
#define BIND_NOT_SUPPORTED          0x84
#define BIND_TABLE_FULL             0x87
#define BIND_NO_ENTRY               0x88

typedef enum _BINDING_RESULTS
{
    BIND_ILLEGAL_DEVICE = 0x03,     //0x03
    BIND_ILLEGAL_REQUEST,           //0x04
    BIND_INVALID_BINDING            //0x05
} BINDING_RESULTS;

typedef enum _KEY_OPERATION_ERROR
{
	KEY_SUCCESS = 0x00,
	KEY_INVALID_PARAMETER,
	KEY_NO_MASTER_KEY,
	KEY_INVALID_CHALLENGE,
	KEY_INVALID_SKG,
	KEY_INVALID_MAC,
	KEY_INVALID_KEY,
	KEY_TIMEOUT,
	KEY_BAD_FRAME
} KEY_OPERATION_ERROR;
#define MAX_APS_FRAMES              (NUM_BUFFERED_INDIRECT_MESSAGES + MAX_APL_FRAMES)

// ******************************************************************************
// Structures

typedef struct _APS_ADDRESS_MAP
{
    LONG_ADDR   longAddr;
    SHORT_ADDR  shortAddr;
} APS_ADDRESS_MAP;


// This structure is used for storing source and destination binding
// records.  Within the array of binding records, a valid node is
// indicated by the corresponding bit in bindingTableUsageMap being
// set.  The type of record is indicated by the corresponding bit in
// bindingTableSourceNodeMap.  If the bit is set, then shortAddress
// and endPoint pertain to the source and clusterID is guaranteed to
// be correct.  If it is clear, then they pertain to the destination,
// and clusterID may be undefined.  Note that the first node in each
// list will be the source record, followed by one or more
// destination records.
// NOTE - the limitation is that a short address may only support one profile.
typedef struct _BINDING_RECORD
{
    SHORT_ADDR      shortAddr;
    BYTE            endPoint;
    BYTE            clusterID;
    BYTE            nextBindingRecord;
} BINDING_RECORD;    // 5 bytes long


// ******************************************************************************
// Macro Definitions

#define APSDiscardRx()          MACDiscardRx()
#define APSClearBindingTable()  ClearBindingTable()


// ******************************************************************************
// Function Prototypes

BYTE                APSGet( void ); // TODO can we consolidate all these get functions?
BOOL                APSHasBackgroundTasks( void );
void                APSInit( void );
ZIGBEE_PRIMITIVE    APSTasks(ZIGBEE_PRIMITIVE inputPrimitive);

#if defined(I_SUPPORT_BINDINGS)
    BYTE APSAddBindingInfo( SHORT_ADDR srcAddr, BYTE srcEP, BYTE clusterID,
                     SHORT_ADDR destAddr, BYTE destEP );
    BYTE APSRemoveBindingInfo( SHORT_ADDR srcAddr, BYTE srcEP, BYTE clusterID, SHORT_ADDR destAddr, BYTE destEP );
#endif

#if MAX_APS_ADDRESSES > 0
    void APSClearAPSAddressTable( void );
#endif

#endif
