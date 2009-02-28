/*********************************************************************

                    ZigBee NWK Layer

 NOTE: Route Request are always sent with Route Repair.

 *********************************************************************
 * FileName:        zNWK.c
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
 * DF/KO                01/09/06 Microchip ZigBee Stack v1.0-3.5
 * DF/KO                08/31/06 Microchip ZigBee Stack v1.0-3.6
 * DF/KO/YY             11/27/06 Microchip ZigBee Stack v1.0-3.7
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/

#include "generic.h"
#include "zigbee.def"
#include "sralloc.h"
#include "ZigbeeTasks.h"
#include "zNWK.h"
#include "zMAC.h"
#include "SymbolTime.h"
#include "zNVM.h"
#include "sralloc.h"
#include <string.h>         // for memcpy-type functions
#ifdef I_SUPPORT_SECURITY
#include "zSecurity.h"
#endif

#include "zPHY.h"
#ifdef ZCP_DEBUG
    #include "Console.h"
#else
    #define ConsolePutROMString(x)
    #undef printf
    #define printf(x)
    #define PrintChar(x)
#endif

// ******************************************************************************
// Configuration Definitions
#define NIB_STATIC_IMPLEMENTATION
#define CALCULATE_LINK_QUALITY

// Define how a 16-bit random value will be supplied.
// In this version, a snapshot of running TMR0 value is used as
// a random value.
// An application may select to provide random value from another source
#define RANDOM_LSB                      (TMR0L)
#define RANDOM_MSB                      (TMR0H)


// ******************************************************************************
// Constant Definitions

#define CONSTANT_PATH_COST              7
#define INVALID_ROUTE_DISCOVERY_INDEX   ROUTE_DISCOVERY_TABLE_SIZE
#define LAST_END_DEVICE_ADDRESS         0xFFFE
#define MAC_TX_OPTIONS_ACKNOWLEDGE      0x01   // acknowledged transmission
#define MAC_TX_OPTIONS_GTS              0x02   // GTS transmission
#define MAC_TX_OPTIONS_INDIRECT         0x04   // indirect transmission
#define MAC_TX_OPTIONS_SECURITY         0x08   // security enabled transmission
#define MAX_NWK_FRAMES                  (NUM_BUFFERED_BROADCAST_MESSAGES + \
                                         NUM_BUFFERED_ROUTING_MESSAGES + \
                                         MAX_APL_FRAMES + \
                                         NUM_BUFFERED_INDIRECT_MESSAGES * 2 + \
                                         MAX_APS_ACK_FRAMES )
#define NO_ROUTE_REPAIR                 0x00
#define NWK_FRAME_CMD                   0x01
#define NWK_FRAME_DATA                  0x00
#define NWK_FRAME_TYPE_MASK             0x03
#define NWK_LEAVE_INDICATION            0x00
#define NWK_LEAVE_REQUEST               0x40
#define NWK_LEAVE_REMOVE_CHILDREN       0x80
#define nwkProtocolVersion              0x01
#define ROUTE_REPAIR                    0x80
#define ZIGBEE_PROTOCOL_ID              0x00        // As per spec

// This value marks whether or not the neighbor table contains valid
// values left over from before a loss of power.
#define NEIGHBOR_TABLE_VALID    0xA53C

// NIB Default values not specified by the Profile
#define DEFAULT_nwkPassiveAckTimeout                0x03
#define DEFAULT_nwkMaxBroadcastRetries              0x03
#if defined(I_AM_END_DEVICE)
    #define DEFAULT_nwkNeighborTable                NULL
#else
    #define DEFAULT_nwkNeighborTable                neighborTable
#endif
#define DEFAULT_nwkNetworkBroadcastDeliveryTime     (DEFAULT_nwkPassiveAckTimeout*(DEFAULT_nwkMaxBroadcastRetries+1))
#define DEFAULT_nwkReportConstantCost               0
#define DEFAULT_nwkRouteDiscoveryRetriesPermitted   0x03
#if defined (I_SUPPORT_ROUTING)
    #define DEFAULT_nwkRouteTable                   routingTable
#else
    #define DEFAULT_nwkRouteTable                   NULL
#endif
//#define DEFAULT_nwkSecureAllFrames                  0 removed by errata
//#define DEFAULT_nwkSecurityLevel                    0 removed by errata
#define DEFAULT_nwkSymLink                          FALSE
#define DEFAULT_nwkCapabilityInformation            0
#define DEFAULT_nwkUseTreeAddrAlloc                 TRUE
#define DEFAULT_nwkUseTreeRouting                   TRUE
#define DEFAULT_nwkAllFresh             1

// We support only the address allocation called out by the ZigBee spec
//#define DEFAULT_nwkNextAddress
//#define DEFAULT_nwkAvailableAddresses
//#define DEFAULT_nwkAddressIncrement


#define BROADCAST_JITTER            ((MAC_LONG_ADDR_BYTE0 & (nwkcMaxBroadcastJitter-1)) * ONE_SECOND / 1000)
#if BROADCAST_JITTER == 0
    #define RREQ_BROADCAST_JITTER   (2 * nwkcMinRREQJitter * (DWORD)ONE_SECOND / 1000)
#else
    #define RREQ_BROADCAST_JITTER   (2 * BROADCAST_JITTER)
#endif

// ******************************************************************************
// Enumerations

typedef enum _IEEE_DEVICE_TYPE_VALUES
{
    DEVICE_IEEE_RFD                 = 0x00,
    DEVICE_IEEE_FFD                 = 0x01
} IEEE_DEVICE_TYPE_VALUES;

typedef enum _NWK_COMMANDS
{
    NWK_COMMAND_ROUTE_REQUEST       = 0x01,
    NWK_COMMAND_ROUTE_REPLY         = 0x02,
    NWK_COMMAND_ROUTE_ERROR         = 0x03,
    NWK_COMMAND_LEAVE               = 0x04
} NWK_COMMANDS;

typedef enum _ZIGBEE_DEVICE_TYPE_VALUES
{
    DEVICE_ZIGBEE_COORDINATOR       = 0x00,
    DEVICE_ZIGBEE_ROUTER            = 0x01,
    DEVICE_ZIGBEE_END_DEVICE        = 0x02
} ZIGBEE_DEVICE_TYPE_VALUES;

typedef enum _MAC_ASSOCIATION_STATUS_VALUES
{
    ASSOCIATION_SUCCESS             = 0x00,
    ASSOCIATION_PAN_AT_CAPACITY     = 0x01,
    ASSOCIATION_PAN_ACCESS_DENIED   = 0x02
} MAC_ASSOCIATION_STATUS_VALUES;

typedef enum _ROUTE_ERROR_CODES
{
    ROUTE_ERROR_NO_ROUTE_AVAILABLE = 0x00,
    ROUTE_ERROR_TREE_LINK_FAILURE,
    ROUTE_ERROR_NONTREE_LINK_FAILURE,
    ROUTE_ERROR_LOW_BATTERY,
    ROUTE_ERROR_NO_ROUTING_CAPACITY
} ROUTE_ERROR_CODES;

typedef enum _ROUTE_STATUS
{
    ROUTE_ACTIVE                    = 0x00,
    ROUTE_DISCOVERY_UNDERWAY        = 0x01,
    ROUTE_DISCOVERY_FAILED          = 0x02,
    ROUTE_INACTIVE                  = 0x03
} ROUTE_STATUS;

typedef enum _MESSAGE_ROUTING_STATUS
{
    ROUTE_SEND_TO_MAC_ADDRESS,
    ROUTE_MESSAGE_BUFFERED,
    ROUTE_FAILURE_TREE_LINK,
    ROUTE_FAILURE_NONTREE_LINK,
    ROUTE_FAILURE_NO_CAPACITY
} MESSAGE_ROUTING_STATUS;

// ******************************************************************************
// Data Structures


typedef struct _CHANNEL_INFO
{
    BYTE    channel;
    BYTE    energy;
    BYTE    networks;
} CHANNEL_INFO;


typedef struct _DISCOVERY_INFO
{
    BYTE            currentIndex;
    BYTE            numChannels;
    CHANNEL_INFO    *channelList;
} DISCOVERY_INFO;


typedef union _NWK_FRAME_CONTROL_LSB
{
    BYTE    Val;
    struct _NWK_FRAME_CONTROL_LSB_bits
    {
        BYTE    frameType       : 2;
        BYTE    protocolVersion : 4;
        BYTE    routeDiscovery  : 2;
    } bits;
} NWK_FRAME_CONTROL_LSB;


typedef union _NWK_FRAME_CONTROL_MSB
{
    BYTE    Val;
    struct _NWK_FRAME_CONTROL_MSB_bits
    {
        BYTE                    : 1;
        BYTE    security        : 4;
    } bits;
} NWK_FRAME_CONTROL_MSB;


typedef struct _BTR     // Broadcast Transaction Record
{
    BYTE                    *dataPtr;
    BYTE                    dataLength;
    BYTE                    nwkSequenceNumber;
    BYTE                    nwkRadius;
    SHORT_ADDR              nwkSourceAddress;
    SHORT_ADDR              nwkDestinationAddress;
    NWK_FRAME_CONTROL_MSB   nwkFrameControlMSB;
    NWK_FRAME_CONTROL_LSB   nwkFrameControlLSB;
    BYTE                    currentNeighbor;
    TICK                    broadcastJitterTimer;
    TICK                    broadcastTime;
    struct _BTR_flags1
    {
        BYTE    nRetries                : 4;
        BYTE    bMessageFromUpperLayers : 1;
        BYTE    bConfirmSent            : 1;
    #ifdef I_SUPPORT_SECURITY
        BYTE    bAlreadySecured         : 1;
    #endif
    } btrInfo;

    union _BTR_flags2
    {
        // TODO Since we are down to one flag per neighbor, would it be worth
        // bitmapping these?  Maybe not - a lot of overhead to calculate mask and index,
        // and this is only alloc'd.
        BYTE    byte;
        struct BTR__bits
        {
            BYTE    bMessageNotRelayed      : 1;
        } bits;
    } flags[MAX_NEIGHBORS];
} BTR;
#define BTR_FLAGS_INIT  0x03


typedef struct _BUFFERED_MESSAGE_INFO
{
    BYTE        *dataPtr;
    BYTE        dataLength;
    SHORT_ADDR  sourceAddress;
    SHORT_ADDR  destinationAddress;
    TICK        timeStamp;
} BUFFERED_MESSAGE_INFO;


typedef union _LEAVE_COMMAND_OPTIONS
{
    BYTE        Val;
    struct _LEAVE_BITS
    {
        BYTE    : 6;
        BYTE    bIsRequest      : 1;
        BYTE    bRemoveChildren : 1;
    } bits;
} LEAVE_COMMAND_OPTIONS;


typedef struct _NIB
{
    BYTE nwkBCSN;

    #ifndef NIB_STATIC_IMPLEMENTATION
        BYTE nwkPassiveAckTimeout;
        BYTE nwkMaxBroadcastRetries;
        BYTE nwkMaxChildren;
        BYTE nwkMaxDepth;
        BYTE nwkMaxRouters;
        #ifdef USE_EXTERNAL_NVM
            WORD nwkNeighborTable;
        #else
            ROM NEIGHBOR_RECORD* nwkNeighborTable;
        #endif
        BYTE nwkNetworkBroadcastDeliveryTime;
        BYTE nwkReportConstantCost;
        BYTE nwkRouteDiscoveryRetriesPermitted;
        #ifdef USE_EXTERNAL_NVM
            WORD nwkRouteTable;
        #else
            ROM ROUTING_ENTRY* nwkRouteTable;
        #endif
//        BYTE nwkSecureAllFrames;  removed as per NWK errata
//        BYTE nwkSecurityLevel;    removed as per NWK errata
        BYTE nwkSymLink;
        BYTE nwkCapabilityInformation;
        BOOL nwkUseTreeAddrAlloc;
        BOOL nwkUseTreeRouting;
        //SHORT_ADDR  nwkNextAddress;         // We support only the address allocation called out by the ZigBee spec
        //SHORT_ADDR  nwkAvailableAddresses;  // We support only the address allocation called out by the ZigBee spec
        //SHORT_ADDR  nwkAddressIncrement;    // We support only the address allocation called out by the ZigBee spec
    #endif
} NIB_TABLE;


typedef struct _NWK_FRAMES
{
    BYTE    msduHandle;
    BYTE    nsduHandle;
} NWK_FRAMES;


typedef union _NWK_FRAME_CONTROL_LSB
{
    BYTE    Val;
    struct _NWK_FRAME_CONTROL_LSB_bits
    {
        BYTE    frameType       : 2;
        BYTE    protocolVersion : 4;
        BYTE    routeDiscovery  : 2;
    } bits;
} NWK_FRAME_CONTROL_LSB;


typedef union _NWK_FRAME_CONTROL_MSB
{
    BYTE    Val;
    struct _NWK_FRAME_CONTROL_MSB_bits
    {
        BYTE                    : 1;
        BYTE    security        : 1;
    } bits;
} NWK_FRAME_CONTROL_MSB;


typedef struct _NWK_STATUS
{
    union _NWK_STATUS_flags
    {
        WORD    Val;
        struct _NWK_STATUS_bits
        {
            // Background Task Flags
            BYTE    bSendingBroadcastMessage    : 1;
            BYTE    bAwaitingRouteDiscovery     : 1;
            BYTE    bTimingJoinPermitDuration   : 1;
            BYTE    bLeavingTheNetwork          : 1;
            BYTE    bLeaveDisassociate          : 1;
            BYTE    bLeaveWaitForConfirm        : 1;
            BYTE    bLeaveReset                 : 1;

            // Status Flags
            BYTE    bCanRoute                   : 1;
            BYTE    bRemovingChildren           : 1;    // Not used by end device
            BYTE    bAllChildrenLeft            : 1;    // Not used by end device
            BYTE    bAllEndDeviceAddressesUsed  : 1;
            BYTE    bAllRouterAddressesUsed     : 1;
        } bits;
    } flags;

    #ifndef I_AM_RFD
        BTR                     *BTT[NUM_BUFFERED_BROADCAST_MESSAGES];  // Broadcast Transaction Table
    #endif
    #if defined( I_AM_COORDINATOR ) || defined( I_AM_ROUTER )
        TICK                    joinDurationStart;
        BYTE                    joinPermitDuration;
    #endif
    #if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
        BYTE                    routeRequestID;
        BUFFERED_MESSAGE_INFO   *routingMessages[NUM_BUFFERED_ROUTING_MESSAGES];
    #endif

    DISCOVERY_INFO              discoveryInfo;
    ASSOCIATE_CAPABILITY_INFO   lastCapabilityInformation;
    DWORD_VAL                   lastScanChannels;
    BYTE                        lastScanDuration;
    #ifndef I_AM_END_DEVICE
        BYTE                    leaveCurrentNode;
        TICK                    leaveStartTime;
    #endif
    BYTE                        leaveReason;
    #ifdef I_AM_COORDINATOR
        PAN_ADDR                requestedPANId;
    #endif
} NWK_STATUS;
#define NWK_BACKGROUND_TASKS    0x003F


typedef struct _ROUTE_DISCOVERY_ENTRY
{
    TICK            timeStamp;
    TICK            rebroadcastTimer;
    SHORT_ADDR      srcAddress;
    SHORT_ADDR      senderAddress;
    BYTE            *forwardRREQ;
    BYTE            routeRequestID;
    BYTE            forwardCost;
    BYTE            residualCost;
    BYTE            previousCost;
    BYTE            routingTableIndex;      // Added so we can find the routing entry during clean-up.
    struct          _status
    {
        BYTE        transmitCounter     : 3;
        BYTE        initialRebroadcast  : 1;
    }               status;
} ROUTE_DISCOVERY_ENTRY;


typedef struct _ROUTE_REPLY_COMMAND
{
    BYTE            commandFrameIdentifier;
    BYTE            commandOptions;
    BYTE            routeRequestIdentifier;
    SHORT_ADDR      originatorAddress;
    SHORT_ADDR      responderAddress;
    BYTE            pathCost;
} ROUTE_REPLY_COMMAND;


typedef struct _ROUTE_REQUEST_COMMAND
{
    BYTE            commandFrameIdentifier;
    BYTE            commandOptions;
    BYTE            routeRequestIdentifier;
    SHORT_ADDR      destinationAddress;
    BYTE            pathCost;
} ROUTE_REQUEST_COMMAND;


// ******************************************************************************
// Variable Definitions

NWK_FRAMES              nwkConfirmationHandles[MAX_NWK_FRAMES];

#ifdef NIB_STATIC_IMPLEMENTATION
    NIB_TABLE           NIB;                                  // nwkBCSN initialized in VariableAndTableInitialization

    #define NIB_nwkPassiveAckTimeout                DEFAULT_nwkPassiveAckTimeout
    #define NIB_nwkMaxBroadcastRetries              DEFAULT_nwkMaxBroadcastRetries
    #define NIB_nwkMaxChildren                      PROFILE_nwkMaxChildren
    #define NIB_nwkMaxDepth                         PROFILE_nwkMaxDepth
    #define NIB_nwkMaxRouters                       PROFILE_nwkMaxRouters
    #define NIB_nwkNeighborTable                    DEFAULT_nwkNeighborTable
    #define NIB_nwkNetworkBroadcastDeliveryTime     DEFAULT_nwkNetworkBroadcastDeliveryTime
    #define NIB_nwkReportConstantCost               DEFAULT_nwkReportConstantCost
    #define NIB_nwkRouteDiscoveryRetriesPermitted   DEFAULT_nwkRouteDiscoveryRetriesPermitted
    #define NIB_nwkRouteTable                       DEFAULT_nwkRouteTable
//    #define NIB_nwkSecureAllFrames                  DEFAULT_nwkSecureAllFrames
//    #define NIB_nwkSecurityLevel                    DEFAULT_nwkSecurityLevel
    #define NIB_nwkSymLink                          DEFAULT_nwkSymLink
    #define NIB_nwkCapabilityInformation            DEFAULT_nwkCapabilityInformation
    #define NIB_nwkUseTreeAddrAlloc                 DEFAULT_nwkUseTreeAddrAlloc
    #define NIB_nwkUseTreeRouting                   DEFAULT_nwkUseTreeRouting

    #if NIB_nwkMaxBroadcastRetries > 15
        #error nwkMaxBroadcastRetries must not be greater than 15.
    #endif
#else
    NIB_TABLE           NIB;                                // Values initialized in VariableAndTableInitialization
#endif

#if defined(I_SUPPORT_SECURITY)
    BYTE nwkSecurityLevel;
	extern SECURITY_STATUS securityStatus;
#endif

NWK_STATUS              nwkStatus;

#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
ROUTE_DISCOVERY_ENTRY   *routeDiscoveryTablePointer[ROUTE_DISCOVERY_TABLE_SIZE];
#endif

#if defined(I_SUPPORT_SECURITY)
    #pragma udata SecurityCounterLocation=SECURITY_COUNTER_LOCATION
    extern DWORD_VAL IncomingFrameCount[2][MAX_NEIGHBORS];
    extern DWORD_VAL   OutgoingFrameCount[2];
    #pragma udata
    extern volatile PHY_PENDING_TASKS  PHYTasksPending;
    extern TICK     AuthorizationTimeout;
    extern SECURITY_STATUS	securityStatus;
    #ifdef I_AM_RFD
        extern TICK lastPollTime;
    #endif
    #if defined(USE_EXTERNAL_NVM)
	    extern NETWORK_KEY_INFO plainSecurityKey[2];
	    extern BOOL SetSecurityKey(INPUT BYTE index, INPUT NETWORK_KEY_INFO newSecurityKey);
	    extern BOOL InitSecurityKey(void);
	#endif
#endif

#ifdef ZCP_DEBUG
	BYTE                accessDenied = 0;
#ifndef I_AM_COORDINATOR
    extern BYTE AllowJoin;
#endif

extern BOOL bDisableShortAddress;
#endif

// ******************************************************************************
// Function Prototypes

NEIGHBOR_KEY CanAddNeighborNode( void );
void VariableAndTableInitialization( BOOL force );
NEIGHBOR_KEY NWKLookupNodeByShortAddrVal( WORD shortAddrVal );
NEIGHBOR_KEY NWKLookupNodeByLongAddr(LONG_ADDR *longAddr);
BYTE NWKGet( void );
void Prepare_MCPS_DATA_request( WORD macDestAddressVal, BYTE *msduHandle );
void RemoveNeighborTableEntry( void );

#ifdef I_AM_COORDINATOR
	BOOL RequestedPANIDFound( BYTE channel );
#endif

#ifndef I_AM_END_DEVICE
	#ifdef I_SUPPORT_SECURITY
		void AddChildNode(BOOL bSecured);
	#else
		void AddChildNode( void );
	#endif
	BOOL CanAddChildNode( void );
	void SetBeaconPayload( void );
#endif
void RemoveNeighborTableEntry( void );

#ifndef I_AM_RFD
BOOL CreateNewBTR( BYTE *BTTIndex );
WORD GetCSkipVal( BYTE depth );
BOOL GetRoutingAddress( BOOL fromUpperLayers, SHORT_ADDR nwkAddress, BYTE discoverRoute, SHORT_ADDR *macAddress );
void MarkNeighborAsPasssiveACKed( BYTE BTTindex );
#endif

#ifdef I_SUPPORT_ROUTING
void CreateRouteReply( SHORT_ADDR originatorAddress, BYTE rdIndex, ROUTE_REQUEST_COMMAND *rreq );
BOOL IsDescendant( SHORT_ADDR parentAddr, SHORT_ADDR childAddr, BYTE parentDepth );
BOOL RouteAlongTree( SHORT_ADDR destTarget, SHORT_ADDR *destNextHop );

#if !defined(USE_TREE_ROUTING_ONLY)
BOOL CreateRoutingTableEntries( SHORT_ADDR targetAddr, BYTE *rdIndex, BYTE *rtIndex );
BOOL GetNextHop( SHORT_ADDR destAddr, SHORT_ADDR *nextHop, BYTE *routeStatus );
BOOL HaveRoutingCapacity( BOOL validID, BYTE routeRequestID, SHORT_ADDR routeSrcAddress, SHORT_ADDR routeDestAddress, BYTE commandOptions );
MESSAGE_ROUTING_STATUS InitiateRouteDiscovery( BOOL fromUpperLayers, BYTE discoverRoute );
#endif
#endif


/*********************************************************************
 * Function:        BOOL NWKHasBackgroundTasks( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - NWK layer has background tasks to run
 *                  FALSE - NWK layer does not have background tasks
 *
 * Side Effects:    None
 *
 * Overview:        Determines if the NWK layer has background tasks
 *                  that need to be run.
 *
 * Note:            None
 ********************************************************************/

BOOL NWKHasBackgroundTasks( void )
{
    return ((nwkStatus.flags.Val & NWK_BACKGROUND_TASKS) != 0);
}


/*********************************************************************
 * Function:        void NWKInit( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    NWK layer data structures are initialized.
 *
 * Overview:        This routine initializes all NWK layer data
 *                  structures.
 *
 * Note:            This routine is intended to be called as part of
 *                  a network or power-up initialization.  If called
 *                  after the network has been running, heap space
 *                  may be lost unless the heap is also reinitialized.
 *                  Routes will also be destroyed.
 ********************************************************************/

void NWKInit( void )
{
    BYTE    i;

    VariableAndTableInitialization( TRUE );

    // Initialize the neighbor table.
    // Read the record into RAM.
    GetNeighborTableInfo();

    if ( currentNeighborTableInfo.validityKey != NEIGHBOR_TABLE_VALID )
    {
        // The neighbor table is invalid, so clear our RAM and ROM table.
        NWKClearNeighborTable();
    }
    else
    {
        #ifndef I_AM_COORDINATOR
            // If we have our old parent in our neighbor table, we can
            // try to join as an orphan.
            if (currentNeighborTableInfo.parentNeighborTableIndex != INVALID_NEIGHBOR_KEY)
            {
                ZigBeeStatus.flags.bits.bTryOrphanJoin = 1;
            }
        #endif
        // Child address information may not be valid - we might not
        // have been able to join a netork last time.  So we will leave it
        // the value we read from the neighbor table info.
    }
}

/*********************************************************************
 * Function:        ZIGBEE_PRIMITIVE NWKTasks(ZIGBEE_PRIMITIVE inputPrimitive)
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
 * Note:            This routine may be called while the TX path is blocked.
 *                  Therefore, we must check the Tx path before doing any
 *                  processing that may generate a message.
 *                  It is the responsibility of this task to ensure that
 *                  only one output primitive is generated by any path.
 *                  If multiple output primitives are generated, they
 *                  must be generated one at a time by background processing.
 ********************************************************************/

ZIGBEE_PRIMITIVE NWKTasks(ZIGBEE_PRIMITIVE inputPrimitive)
{
    BYTE        i;
    BYTE        *ptr;
    SHORT_ADDR  macAddress;

    // *********************************************************************
    // Manage primitive- and Tx-independent tasks here.  These tasks CANNOT
    // produce a primitive or send a message.

    // ---------------------------------------------------------------------
    // Handle join permit time-out
    #if defined( I_AM_COORDINATOR ) || defined( I_AM_ROUTER )
        if (nwkStatus.flags.bits.bTimingJoinPermitDuration)
        {
            // NOTE: Compiler SSR27744, TickGet() output must be assigned to a variable.
            TICK tempTick;

            tempTick = TickGet();
            if (TickGetDiff( tempTick, nwkStatus.joinDurationStart) > ONE_SECOND*nwkStatus.joinPermitDuration)
//            if (TickGetDiff( TickGet(), nwkStatus.joinDurationStart) > ONE_SECOND*nwkStatus.joinPermitDuration)
            {
                macPIB.macAssociationPermit = FALSE;
                nwkStatus.flags.bits.bTimingJoinPermitDuration = 0;
            }
        }
    #endif

    // *********************************************************************
    // Handle other tasks and primitives that may require a message to be sent.

    if (inputPrimitive == NO_PRIMITIVE)
    {
        // If Tx is blocked, we cannot generate a message or send back another primitive.
        if (!ZigBeeReady())
        {
            return NO_PRIMITIVE;
        }

        // Manage background tasks here

        // ---------------------------------------------------------------------
        // Handle any pending broadcast messages

        #ifndef I_AM_RFD
            if (nwkStatus.flags.bits.bSendingBroadcastMessage)
            {
                struct _BROADCAST_INFO
                {
                    BYTE        destroyPacket   : 1;    // Well, not really any more.  Just send up a confirm.
                    BYTE        firstBroadcast  : 1;
                    BYTE        sendMessage     : 1;
                }           broadcastInfo;
                BYTE        BTTIndex;
                BYTE        neighborIndex;
                BYTE        queueIndex;
                TICK        tempTick;

                // NOTE: Compiler SSR27744, TickGet() output must be assigned to a variable.
                tempTick = TickGet();

                for (BTTIndex=0; BTTIndex<NUM_BUFFERED_BROADCAST_MESSAGES; BTTIndex++)
                {
                    if (nwkStatus.BTT[BTTIndex] != NULL)
                    {
                        // See if the packet's delivery time has expired and we can free the packet.  Otherwise,
                        // we need to keep it around in case someone thought they transmitted but didn't, and we get cascading
                        // messages.

                        #ifdef NIB_STATIC_IMPLEMENTATION
                        if ((TickGetDiff(tempTick, nwkStatus.BTT[BTTIndex]->broadcastTime)) > (ONE_SECOND * ((DWORD)NIB_nwkNetworkBroadcastDeliveryTime)))
//                        if((TickGetDiff(TickGet(), nwkStatus.BTT[BTTIndex]->broadcastTime)) > (ONE_SECOND * ((DWORD)NIB_nwkNetworkBroadcastDeliveryTime)))
                        #else
                        if ((TickGetDiff(tempTick, nwkStatus.BTT[BTTIndex]->broadcastTime)) > (ONE_SECOND * ((DWORD)NIB.nwkNetworkBroadcastDeliveryTime)))
//                        if((TickGetDiff(TickGet(), nwkStatus.BTT[BTTIndex]->broadcastTime)) > (ONE_SECOND * ((DWORD)NIB.nwkNetworkBroadcastDeliveryTime)))
                        #endif
                        {
                            SRAMfree( nwkStatus.BTT[BTTIndex]->dataPtr );
                            free( nwkStatus.BTT[BTTIndex] );
                        }
                        else if (!nwkStatus.BTT[BTTIndex]->btrInfo.bConfirmSent)
                        {
                            // If we have not sent up a confirm, see if we need to broadcast/rebroadcast/unicast.

                            broadcastInfo.sendMessage = 0;
                            broadcastInfo.firstBroadcast = 0;
                            if (nwkStatus.BTT[BTTIndex]->currentNeighbor != MAX_NEIGHBORS)
                            {
                                neighborIndex = nwkStatus.BTT[BTTIndex]->currentNeighbor;
                                if (neighborIndex == 0xFF)
                                {
                                    // See if we have waited the jitter time before sending the first broadcast.
                                    if((tempTick.Val - nwkStatus.BTT[BTTIndex]->broadcastJitterTimer.Val) >= (DWORD)BROADCAST_JITTER)
    //                                    if((TickGetDiff(TickGet(), nwkStatus.BTT[BTTIndex]->broadcastJitterTimer)) >= BROADCAST_JITTER)
                                    {
                                        // Send the first broadcast packet.
                                        macAddress.Val = 0xFFFF;
                                        broadcastInfo.sendMessage = 1;
                                        broadcastInfo.firstBroadcast = 1;
                                        nwkStatus.BTT[BTTIndex]->broadcastTime = TickGet();
                                        neighborIndex = 0;
                                    }
                                }
                                else
                                {
                                    // For each neighbor, see if we need to unicast the message.
                                    // If not, mark him as frame sent - he should have received the
                                    // original broadcast message.  If so, unicast it to him.  If the
                                    // neighbor record is not in use, mark it already done.  Mark as
                                    // many as we can until we find one that must be unicast.
                                    do
                                    {
                                        #ifdef USE_EXTERNAL_NVM
                                            pCurrentNeighborRecord = neighborTable + (WORD)neighborIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                                        #else
                                            pCurrentNeighborRecord = &(neighborTable[neighborIndex]);
                                        #endif
                                        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

                                        if (!currentNeighborRecord.deviceInfo.bits.bInUse)
                                        {
                                            nwkStatus.BTT[BTTIndex]->flags[neighborIndex].byte = 0;
                                        }
                                        else
                                        {
                                            if (currentNeighborRecord.shortAddr.Val == nwkStatus.BTT[BTTIndex]->nwkSourceAddress.Val)
                                            {
                                                // If this is the message source, we do not send it back to them!
                                                nwkStatus.BTT[BTTIndex]->flags[neighborIndex].byte = 0;
                                            }
                                            //else if (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)
                                            // This change is necessary to allow RFD devices whose receiver is always on to receive
                                            // the broadcast message.  FFD end devices will receive the message twice, but they
                                            // will filter it out.
                                            else if (currentNeighborRecord.deviceInfo.bits.deviceType == DEVICE_ZIGBEE_END_DEVICE)
                                            {
                                                // Unicast the message.
                                                macAddress.Val = currentNeighborRecord.shortAddr.Val;
                                                broadcastInfo.sendMessage = 1;
                                                // Clear the relayed bit - the MAC will handle any retransmission
                                                nwkStatus.BTT[BTTIndex]->flags[neighborIndex].bits.bMessageNotRelayed = 0;
                                            }
                                        }

                                        neighborIndex++;
                                    } while (!broadcastInfo.sendMessage && (neighborIndex < MAX_NEIGHBORS));

                                }
                                nwkStatus.BTT[BTTIndex]->currentNeighbor = neighborIndex;
                            }
                            else
                            {
                                // We've sent to all of the neighbors, so see if they've all responded.  If not, we need
                                // to rebroadcast.
                                broadcastInfo.destroyPacket = 1;
                                params.NLDE_DATA_confirm.Status = SUCCESS;
                                for (neighborIndex=0;
                                     (neighborIndex<MAX_NEIGHBORS) && (nwkStatus.BTT[BTTIndex]->flags[neighborIndex].bits.bMessageNotRelayed==0);
                                     neighborIndex++) {}

                                if (neighborIndex!=MAX_NEIGHBORS)
                                {
                                    params.NLDE_DATA_confirm.Status = TRANSACTION_EXPIRED;

                                    // Not everyone has retransmitted the message.  See if need to or can transmit.
                                    if (nwkStatus.BTT[BTTIndex]->btrInfo.nRetries != 0)
                                    {
                                        // See if this broadcast has timed out
                                        #ifdef NIB_STATIC_IMPLEMENTATION
                                        if((TickGetDiff(tempTick, nwkStatus.BTT[BTTIndex]->broadcastTime)) >= (ONE_SECOND * ((DWORD)NIB_nwkPassiveAckTimeout)))
    //                                    if((TickGetDiff(TickGet(), nwkStatus.BTT[BTTIndex]->broadcastTime)) >= (ONE_SECOND * ((DWORD)NIB_nwkPassiveAckTimeout)))
                                        #else
                                        if((TickGetDiff(tempTick, nwkStatus.BTT[BTTIndex]->broadcastTime)) >= (ONE_SECOND * ((DWORD)NIB.nwkPassiveAckTimeout)))
    //                                    if((TickGetDiff(TickGet(), nwkStatus.BTT[BTTIndex]->broadcastTime)) >= (ONE_SECOND * ((DWORD)NIB.nwkPassiveAckTimeout)))
                                        #endif
                                        {
                                            broadcastInfo.destroyPacket = 0;
                                            macAddress.Val = 0xFFFF;
                                            broadcastInfo.sendMessage = 1;
                                            nwkStatus.BTT[BTTIndex]->btrInfo.nRetries--;
                                            nwkStatus.BTT[BTTIndex]->broadcastTime = TickGet();
                                        }
                                        else
                                        {
                                            broadcastInfo.destroyPacket = 0;
                                        }
                                    }
                                }

                                // If all of the flags were cleared or the packet has timed out, destroy it.
                                if (broadcastInfo.destroyPacket)
                                {
                                    goto FinishBroadcastPacket;
                                }
                            }

                            // If we need to unicast or broadcast, do it here.
                            if (broadcastInfo.sendMessage)
                            {
                                // If this is the first broadcast of a packet from our upper layers, add the handle to the
                                // confirmation queue.
                                queueIndex = MAX_NWK_FRAMES;
                                if (broadcastInfo.firstBroadcast && (nwkStatus.BTT[BTTIndex]->nwkSourceAddress.Val == macPIB.macShortAddress.Val))
                                {
                                    // Add this frame to the list of frames waiting confirmation.
                                    // Try to find an empty slot.
                                    for (queueIndex=0; (queueIndex<MAX_NWK_FRAMES) && (nwkConfirmationHandles[queueIndex].nsduHandle!=INVALID_NWK_HANDLE); queueIndex++) {}

                                    // If we have no empty slots, destroy the packet and return an error.
                                    if (queueIndex == MAX_NWK_FRAMES)
                                    {
                                        params.NLDE_DATA_confirm.Status = TRANSACTION_OVERFLOW;

FinishBroadcastPacket:
                                        // We can send up a confirm now if the message came from us.  But don't free the message
                                        // until it times out. Otherwise, if we hear someone else's rebroadcast, we'll think it's a new
                                        // message!

                                        params.NLDE_DATA_confirm.NsduHandle = nwkStatus.BTT[BTTIndex]->nwkSequenceNumber;
                                        nwkStatus.BTT[BTTIndex]->btrInfo.bConfirmSent = 1;

                                        if (nwkStatus.BTT[BTTIndex]->btrInfo.bMessageFromUpperLayers)
                                        {
                                            return NLDE_DATA_confirm;
                                        }
                                        else
                                        {
                                            return NO_PRIMITIVE;
                                        }
                                    }
                                }

                                // Load up the NWK header information
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkSequenceNumber;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkRadius;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkSourceAddress.byte.MSB;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkSourceAddress.byte.LSB;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkDestinationAddress.byte.MSB;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkDestinationAddress.byte.LSB;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkFrameControlMSB.Val;
                                TxBuffer[TxHeader--] = nwkStatus.BTT[BTTIndex]->nwkFrameControlLSB.Val;

                                // Load up the NWK payload
                                for (i=0, ptr=nwkStatus.BTT[BTTIndex]->dataPtr; i<nwkStatus.BTT[BTTIndex]->dataLength; i++)
                                {
                                    TxBuffer[TxData++] = *ptr++;
                                }

                                // Load up the MCPS_DATA.request parameters
                                Prepare_MCPS_DATA_request( macAddress.Val, &i );
                                if (queueIndex != MAX_NWK_FRAMES)
                                {
                                    nwkConfirmationHandles[queueIndex].msduHandle = i;
                                    nwkConfirmationHandles[queueIndex].nsduHandle = nwkStatus.BTT[BTTIndex]->nwkSequenceNumber;
                                }
                                return MCPS_DATA_request;
                            }
                        }
                    }
                }

                // See if all of the broadcast messages are done
                for (BTTIndex=0; (BTTIndex<NUM_BUFFERED_BROADCAST_MESSAGES) && (nwkStatus.BTT[BTTIndex]==NULL); BTTIndex++) {}
                if (BTTIndex == NUM_BUFFERED_BROADCAST_MESSAGES)
                {
                    //ConsolePutROMString( (ROM char *)"NWK: All broadcast messages complete\r\n" );
                    nwkStatus.flags.bits.bSendingBroadcastMessage = 0;
                }
            }   // End of handling broadcast messages.
        #endif

        // ---------------------------------------------------------------------
        // Handle any pending route discovery maintenance

        #if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
            if (nwkStatus.flags.bits.bAwaitingRouteDiscovery)
            {
                TICK                currentTime;
                BYTE                rdIndex;

                currentTime = TickGet();

                // See if any of our route requests need to be rebroadcast.
                for (rdIndex = 0; rdIndex < ROUTE_DISCOVERY_TABLE_SIZE; rdIndex ++ )
                {
                    if (routeDiscoveryTablePointer[rdIndex] != NULL)
                    {
                        if (routeDiscoveryTablePointer[rdIndex]->forwardRREQ &&
                            (((routeDiscoveryTablePointer[rdIndex]->status.initialRebroadcast == 0) &&
                              ((currentTime.Val - routeDiscoveryTablePointer[rdIndex]->rebroadcastTimer.Val) >= (DWORD)(ONE_SECOND*nwkcRREQRetryInterval/1000))) ||
                             ((routeDiscoveryTablePointer[rdIndex]->status.initialRebroadcast == 1) &&
                              ((currentTime.Val - routeDiscoveryTablePointer[rdIndex]->rebroadcastTimer.Val) >= (DWORD)RREQ_BROADCAST_JITTER))))
                        {
                            ptr = routeDiscoveryTablePointer[rdIndex]->forwardRREQ;

                            // Load up the NWK payload - the route request command frame.
                            for (i=0; i<sizeof(ROUTE_REQUEST_COMMAND); i++)
                            {
                                TxBuffer[TxData++] = *ptr++;       // Command, options, ID, dest addr LSB, dest addr MSB, path cost
                            }

                            // Load up the old NWK header (backwards).
                            for (i=0; i<8; i++)
                            {
                                TxBuffer[TxHeader--] = *ptr++;
                            }

                            // We've handled the first rebroadcast of a received RREQ (if any), so the rest of the
                            // timing will be based off of the nwkcRREQRetryInterval.  See if we have any retries left.
                            // If not, free the message.
                            routeDiscoveryTablePointer[rdIndex]->status.initialRebroadcast = 0;
                            routeDiscoveryTablePointer[rdIndex]->status.transmitCounter--;
                            routeDiscoveryTablePointer[rdIndex]->rebroadcastTimer = currentTime;
                            if (routeDiscoveryTablePointer[rdIndex]->status.transmitCounter == 0)
                            {
                                free( routeDiscoveryTablePointer[rdIndex]->forwardRREQ );
                            }

                            // Load up the MCPS_DATA.request parameters.
                            Prepare_MCPS_DATA_request( 0xFFFF, &i );
                            return MCPS_DATA_request;
                        }
                    }
                }

                // See if any of our route discoveries have timed out.
                for (rdIndex = 0; rdIndex < ROUTE_DISCOVERY_TABLE_SIZE; rdIndex ++ )
                {
                    if ((routeDiscoveryTablePointer[rdIndex] != NULL) &&
                        ((currentTime.Val - routeDiscoveryTablePointer[rdIndex]->timeStamp.Val) > (DWORD)(ONE_SECOND*nwkcRouteDiscoveryTime/1000)))
                    {
                        #ifdef USE_EXTERNAL_NVM
                            pCurrentRoutingEntry = routingTable + (WORD)(routeDiscoveryTablePointer[rdIndex]->routingTableIndex) * (WORD)sizeof(ROUTING_ENTRY);
                        #else
                            pCurrentRoutingEntry = &(routingTable[routeDiscoveryTablePointer[rdIndex]->routingTableIndex]);
                        #endif
                        GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
                        // Make sure we don't destroy a valid route.
                        if (currentRoutingEntry.status == ROUTE_DISCOVERY_UNDERWAY)
                        {
                            // Mark the record as a failed discovery attempt.
                            currentRoutingEntry.status = ROUTE_DISCOVERY_FAILED;
                            PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );
                        }

                        // Free the route discovery table entry.
                        if (routeDiscoveryTablePointer[rdIndex]->forwardRREQ)
                        {
                           SRAMfree( routeDiscoveryTablePointer[rdIndex]->forwardRREQ );
                        }
                        free( routeDiscoveryTablePointer[rdIndex] );
                    }
                }

                // If we are waiting on route discovery for a message, see if we now
                // have an active route and can send the message, or if discovery
                // has timed out or failed and we can report an error and discard the message.
                for (i=0; i<NUM_BUFFERED_ROUTING_MESSAGES; i++)
                {
                    if (nwkStatus.routingMessages[i])
                    {
                        BYTE        j;
                        BYTE        queueIndex;
                        BYTE        routeStatus;

                        if (GetNextHop( nwkStatus.routingMessages[i]->destinationAddress, &(macAddress), &routeStatus ))
                        {
                            if (routeStatus == ROUTE_ACTIVE)
                            {
                                // We can send the message now.
                                // If the message is from the upper layers, get ready to add the frame to the confirmation queue.
                                if (nwkStatus.routingMessages[i]->sourceAddress.Val == macPIB.macShortAddress.Val)
                                {
                                    // Try to find an empty slot.
                                    for (queueIndex=0; (queueIndex<MAX_NWK_FRAMES) && (nwkConfirmationHandles[queueIndex].nsduHandle!=INVALID_NWK_HANDLE); queueIndex++) {}

                                    // If we have no empty slots, destroy the message and return an error.
                                    if (queueIndex == MAX_NWK_FRAMES)
                                    {
                                        params.NLDE_DATA_confirm.NsduHandle = *(nwkStatus.routingMessages[i]->dataPtr+7);
                                        params.NLDE_DATA_confirm.Status = TRANSACTION_OVERFLOW;
                                        SRAMfree( nwkStatus.routingMessages[i]->dataPtr );
                                        free( nwkStatus.routingMessages[i] );
                                        ZigBeeUnblockTx();
                                        return NLDE_DATA_confirm;
                                    }
                                }

                                // Load up the saved NWK header and payload.

                                ptr = nwkStatus.routingMessages[i]->dataPtr;
								#ifdef I_SUPPORT_SECURITY
                                	for (j = 0; j < 8; j++)
                                	{
                                    	TxHeader = TX_HEADER_START - 8;
                                    	TxBuffer[TxHeader+1+j] = *ptr++;
                                	}
                                	for(j = 0; j < nwkStatus.routingMessages[i]->dataLength-8; j++)
                                	{
                                    	TxBuffer[TxData++] = *ptr++;
                                	}
								#else
                                	for (j=0; j<nwkStatus.routingMessages[i]->dataLength; j++)
                                	{
                                    	TxBuffer[TxData++] = *ptr++;
                                	}
								#endif

                                // Load up the MCPS_DATA.request parameters.
                                Prepare_MCPS_DATA_request( macAddress.Val, &j );

                                // If the message is from the upper layers, add the frame to the confirmation queue.
                                if (nwkStatus.routingMessages[i]->sourceAddress.Val == macPIB.macShortAddress.Val)
                                {
                                    nwkConfirmationHandles[queueIndex].msduHandle = j;
                                    nwkConfirmationHandles[queueIndex].nsduHandle = *(nwkStatus.routingMessages[i]->dataPtr+7);
                                }

                                SRAMfree( nwkStatus.routingMessages[i]->dataPtr );
                                free( nwkStatus.routingMessages[i] );
                                return MCPS_DATA_request;
                            }
                            else if (routeStatus == ROUTE_DISCOVERY_FAILED)
                            {
                                // The route has failed.  Either notify the upper layers with a confirm or
                                // send a route error.  Destroy the buffered message.
                                if (nwkStatus.routingMessages[i]->sourceAddress.Val == macPIB.macShortAddress.Val)
                                {
SendUpNoRouteAvailable:
                                    params.NLDE_DATA_confirm.NsduHandle = *(nwkStatus.routingMessages[i]->dataPtr+7);
                                    params.NLDE_DATA_confirm.Status = ROUTE_ERROR_NO_ROUTE_AVAILABLE;
                                    SRAMfree( nwkStatus.routingMessages[i]->dataPtr );
                                    free( nwkStatus.routingMessages[i] );
                                    ZigBeeUnblockTx();
                                    return NLDE_DATA_confirm;
                                }
                                else
                                {
                                    TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                                    TxBuffer[TxHeader--] = DEFAULT_RADIUS;
                                    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                                    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                                    TxBuffer[TxHeader--] = nwkStatus.routingMessages[i]->sourceAddress.byte.MSB;
                                    TxBuffer[TxHeader--] = nwkStatus.routingMessages[i]->sourceAddress.byte.LSB;
									#ifdef I_SUPPORT_SECURITY
                                    	TxBuffer[TxHeader--] = 0x02;
									#else
                                    	TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
									#endif
                                    TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                                    // Load the NWK payload into the transmit buffer.
                                    TxBuffer[TxData++] = NWK_COMMAND_ROUTE_ERROR;
                                    TxBuffer[TxData++] = ROUTE_ERROR_NO_ROUTE_AVAILABLE;
                                    TxBuffer[TxData++] = nwkStatus.routingMessages[i]->destinationAddress.byte.LSB;
                                    TxBuffer[TxData++] = nwkStatus.routingMessages[i]->destinationAddress.byte.LSB;

                                    // Load up the MCPS_DATA.request parameters.
                                    Prepare_MCPS_DATA_request( nwkStatus.routingMessages[i]->sourceAddress.Val, &j );

                                    free( nwkStatus.routingMessages[i]->dataPtr );
                                    free( nwkStatus.routingMessages[i] );

                                    return MCPS_DATA_request;
                                }
                            }
                        }
                        else
                        {
                            // Somehow we buffered a routed message without having a routing table entry.
                            // Destroy the message.
                            goto SendUpNoRouteAvailable;
                        }
                    }
                }

                // See if we can clean up the routing table.  Purge any routes that failed.  We have
                // already sent errors for any messages buffered for these routes.
                #ifdef USE_EXTERNAL_NVM
                for (i=0, pCurrentRoutingEntry = routingTable; i<ROUTING_TABLE_SIZE; i++, pCurrentRoutingEntry += (WORD)sizeof(ROUTING_ENTRY))
                #else
                for (i=0, pCurrentRoutingEntry = routingTable; i<ROUTING_TABLE_SIZE; i++, pCurrentRoutingEntry++)
                #endif
                {
                    GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
                    if (currentRoutingEntry.status == ROUTE_DISCOVERY_FAILED)
                    {
                        // Mark the record as inactive and available for reuse.
                        currentRoutingEntry.status = ROUTE_INACTIVE;
                        currentRoutingEntry.destAddress.Val = 0xFFFF;
                        PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );
                    }
                }

                // See if we are done with all route discoveries and all buffered messages.
                for (i=0; (i<NUM_BUFFERED_ROUTING_MESSAGES) && (nwkStatus.routingMessages[i]==NULL); i++) {}
                if (i == NUM_BUFFERED_ROUTING_MESSAGES)
                {
                    for (i=0; (i<ROUTE_DISCOVERY_TABLE_SIZE) && (routeDiscoveryTablePointer[i]==NULL); i++) {}
                    if (i == ROUTE_DISCOVERY_TABLE_SIZE)
                    {
                        //ConsolePutROMString( (ROM char *)"NWK: All route discoveries complete\r\n" );
                        nwkStatus.flags.bits.bAwaitingRouteDiscovery = 0;
                    }
                }
            }
        #endif

        // ---------------------------------------------------------------------
        // Handle any pending network leave operation

        if (nwkStatus.flags.bits.bLeavingTheNetwork)
        {
            #ifndef I_AM_END_DEVICE
                // NOTE: Compiler SSR27744, TickGet() output must be assigned to a variable.
                TICK tempTick;
            #endif

            // See if I am doing the final leave clean-up.
            if (nwkStatus.flags.bits.bLeaveReset)
            {
                // Reset the MAC layer.  Note that the NWK layer will automatically reset upon
                // MLME_RESET_confirm, so we do not have to clear any flags here.
                params.MLME_RESET_request.SetDefaultPIB = TRUE;
                return MLME_RESET_request;
            }

            if (nwkStatus.flags.bits.bLeaveWaitForConfirm)
            {
                // NOTE - We should not stay here forever - the MLME_DISASSOCIATE_confirm will time out.
                return NO_PRIMITIVE;
            }

            // See if I am ready to send the disassociate request to my parent.
            if (nwkStatus.flags.bits.bLeaveDisassociate)
            {
                nwkStatus.flags.bits.bLeaveDisassociate   = 0;
                nwkStatus.flags.bits.bLeaveWaitForConfirm = 1;
                params.MLME_DISASSOCIATE_request.DisassociateReason = nwkStatus.leaveReason;
                for (i=0; i<8; i++)
                {
                    params.MLME_DISASSOCIATE_request.DeviceAddress.v[i] = macPIB.macCoordExtendedAddress.v[i];
                }
                // TODO how to set this? params.MLME_DISASSOCIATE_request.SecurityUse = ???
				#ifdef I_SUPPORT_SECURITY
                {
                    BYTE ActiveKeyIndex;
                    GetNwkActiveKeyNumber(&ActiveKeyIndex);
                    if( ActiveKeyIndex == 1 || ActiveKeyIndex == 2 )
                    {
                        params.MLME_DISASSOCIATE_request.SecurityUse = TRUE;
                    } else {
                        params.MLME_DISASSOCIATE_request.SecurityUse = FALSE;
                    }
                }
				#else
                	params.MLME_DISASSOCIATE_request.SecurityUse = FALSE;
				#endif
                return MLME_DISASSOCIATE_request;
            }

            #ifndef I_AM_END_DEVICE
                // Send a leave requests to one of my children.
                if (nwkStatus.leaveCurrentNode != INVALID_NEIGHBOR_KEY)
                {
                    #define childFound i

                    childFound = 0;  // Not found
                    do
                    {
                        #ifdef USE_EXTERNAL_NVM
                            pCurrentNeighborRecord = neighborTable + (WORD)nwkStatus.leaveCurrentNode * (WORD)sizeof(NEIGHBOR_RECORD);
                        #else
                            pCurrentNeighborRecord = &(neighborTable[nwkStatus.leaveCurrentNode]);
                        #endif
                        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
                        if (!currentNeighborRecord.deviceInfo.bits.bInUse ||
                            (currentNeighborRecord.deviceInfo.bits.Relationship != NEIGHBOR_IS_CHILD))
                        {
                            nwkStatus.leaveCurrentNode++;
                        }
                        else
                        {
                            childFound = 1; // Found
                        }
                    } while ((nwkStatus.leaveCurrentNode < MAX_NEIGHBORS) && !childFound);

                    if (childFound)
                    {
                        // Point to my next child for the next pass through the loop.
                        nwkStatus.leaveCurrentNode++;

                        // Send the leave request to my child.
                        TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                        TxBuffer[TxHeader--] = 1;   // radius of 1, as per errata
                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                        TxBuffer[TxHeader--] = currentNeighborRecord.shortAddr.byte.MSB;
                        TxBuffer[TxHeader--] = currentNeighborRecord.shortAddr.byte.LSB;
						#ifdef I_SUPPORT_SECURITY
                        	if( currentNeighborRecord.bSecured )
                        	{
                            	TxBuffer[TxHeader--] = 0x02;
                        	} else
						#endif
                        {
                            TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
                        }
                        TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                        // Load the NWK payload into the transmit buffer.
                        TxBuffer[TxData++] = NWK_COMMAND_LEAVE;
                        i = NWK_LEAVE_REQUEST;
                        if (nwkStatus.flags.bits.bRemovingChildren)
                        {
                            i |= NWK_LEAVE_REMOVE_CHILDREN;
                        }
                        TxBuffer[TxData++] = i;

                        Prepare_MCPS_DATA_request( currentNeighborRecord.shortAddr.Val, &i );
                        return MCPS_DATA_request;
                    }

                    #undef childFound
                }
            #endif

            // Try to send a leave indication to my parent.  I can send it if I didn't make my children
            // leave, if all of my children have already left, or if I've timed out. If I am an end device,
            // I have no children, so I do not need the test.
            #ifndef I_AM_END_DEVICE
            tempTick = TickGet();
            if (!nwkStatus.flags.bits.bRemovingChildren || !currentNeighborTableInfo.numChildren ||
                (TickGetDiff( tempTick, nwkStatus.leaveStartTime ) > (currentNeighborTableInfo.cSkip.Val * MAC_PIB_macTransactionPersistenceTime)))
//            if (!nwkStatus.flags.bits.bRemovingChildren || !currentNeighborTableInfo.numChildren ||
//                (TickGetDiff( TickGet(), nwkStatus.leaveStartTime ) > (currentNeighborTableInfo.cSkip.Val * MAC_PIB_macTransactionPersistenceTime)))
            #endif
            {
                nwkStatus.flags.bits.bLeaveDisassociate = 1;

                // Send the leave indication to my parent.
                TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                TxBuffer[TxHeader--] = 1;   // radius of 1, as per errata
                TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                TxBuffer[TxHeader--] = macPIB.macCoordShortAddress.byte.MSB;
                TxBuffer[TxHeader--] = macPIB.macCoordShortAddress.byte.LSB;
				#ifdef I_SUPPORT_SECURITY
                {
                    BYTE ActiveKeyIndex;
                    GetNwkActiveKeyNumber(&ActiveKeyIndex);
                    if( ActiveKeyIndex == 0x01 || ActiveKeyIndex == 0x02 )
                    {
                        TxBuffer[TxHeader--] = 0x02;
                    } else {
                        TxBuffer[TxHeader--] = 0;
                    }
                }
				#else
                	TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
				#endif
                TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                // Load the NWK payload into the transmit buffer.
                TxBuffer[TxData++] = NWK_COMMAND_LEAVE;
                // If we were removing children, all of our children are gone, and all of our children could remove their children,
                // indicate that all children are gone.  If we are an end device, we have no children, so they are all removed.
                #ifdef I_AM_END_DEVICE
                    i = NWK_LEAVE_INDICATION | NWK_LEAVE_REMOVE_CHILDREN;
                #else
                    i = NWK_LEAVE_INDICATION;
                    if (nwkStatus.flags.bits.bRemovingChildren && !currentNeighborTableInfo.numChildren && nwkStatus.flags.bits.bAllChildrenLeft)
                    {
                        i |= NWK_LEAVE_REMOVE_CHILDREN;
                    }
                #endif
                TxBuffer[TxData++] = i;

                Prepare_MCPS_DATA_request( macPIB.macCoordShortAddress.Val, &i );
                return MCPS_DATA_request;
            }
        }

        return NO_PRIMITIVE;

    }   // End of handling background processing.
    else
    {
        /* handle primitive here */
        switch (inputPrimitive)
        {
            // ---------------------------------------------------------------------
            case MCPS_DATA_confirm:
                {
                    BYTE    rc;

                    // Find the matching frame entry.
                    for (i=0; (i<MAX_NWK_FRAMES) && (nwkConfirmationHandles[i].msduHandle!=params.MCPS_DATA_confirm.msduHandle); i++) {}

                    // If we don't have a matching frame, ignore the primitive.
                    if (i == MAX_NWK_FRAMES)
                    {
                        return NO_PRIMITIVE;
                    }

                    // If the next layer handle is not invalid, set up parameters for the next level confirm.
                    if (nwkConfirmationHandles[i].nsduHandle != INVALID_NWK_HANDLE)
                    {
                        // These overlay - params.NLDE_DATA_confirm.Status     = params.MCPS_DATA_confirm.status;
                        params.NLDE_DATA_confirm.NsduHandle = nwkConfirmationHandles[i].nsduHandle;
                        rc = NLDE_DATA_confirm;
                    }
                    else
                    {
                        rc = NO_PRIMITIVE;
                    }

                    // Destroy the frame.
                    nwkConfirmationHandles[i].nsduHandle = INVALID_NWK_HANDLE;

                    return rc;
                }
                break;

            // ---------------------------------------------------------------------
            case MCPS_DATA_indication:
                {
                    NWK_FRAME_CONTROL_LSB   nwkFrameControlLSB;
                    NWK_FRAME_CONTROL_MSB   nwkFrameControlMSB;
                    SHORT_ADDR              nwkDestinationAddress;
                    SHORT_ADDR              nwkSourceAddress;
                    BYTE                    nwkRadius;
                    BYTE                    nwkSequenceNumber;
                    BYTE                    *nwkHeader = params.MCPS_DATA_indication.msdu;

                    // Extract the NWK header
                    nwkFrameControlLSB.Val         = NWKGet();
                    nwkFrameControlMSB.Val         = NWKGet();
                    nwkDestinationAddress.byte.LSB = NWKGet();
                    nwkDestinationAddress.byte.MSB = NWKGet();
                    nwkSourceAddress.byte.LSB      = NWKGet();
                    nwkSourceAddress.byte.MSB      = NWKGet();
                    nwkRadius                      = NWKGet();
                    nwkSequenceNumber              = NWKGet();
					#ifdef I_SUPPORT_SECURITY
                    	// in network layer, all frames are secured by network key
                    	if( nwkFrameControlMSB.Val & 0x02 )
                    	{
                       		if( !DataDecrypt(params.MCPS_DATA_indication.msdu, &params.MCPS_DATA_indication.msduLength, nwkHeader, 8, ID_NetworkKey, NULL) )
                        	{
                            	NWKDiscardRx();
                            	return NO_PRIMITIVE;
                        	}
                    	}
					#endif

#ifndef I_AM_RFD
                    // See if this is a broadcast packet.
                    if ((params.MCPS_DATA_indication.DstAddr.ShortAddr.Val == 0xFFFF) ||
                        (nwkDestinationAddress.Val == 0xFFFF))
                    {
                        BYTE    j;

                        // This is a broadcast packet.  If this packet is the same as a
                        // packet that I have already seen and is not a route request command frame,
                        // mark the appropriate BTR as having received the relay message, then discard it.
                        // Note that currently Route Request is the only
                        // broadcast command frame.  When there are more, we may have to
                        // peek at the first NWK payload byte.
                        if (nwkFrameControlLSB.bits.frameType != NWK_FRAME_CMD)
                        {
                            for (i=0; i<NUM_BUFFERED_BROADCAST_MESSAGES; i++)
                            {
                                if (nwkStatus.BTT[i] != NULL)
                                {
                                    if ((nwkSourceAddress.Val == nwkStatus.BTT[i]->nwkSourceAddress.Val) &&
                                        (nwkSequenceNumber == nwkStatus.BTT[i]->nwkSequenceNumber))
                                    {
                                        MarkNeighborAsPasssiveACKed( i );

                                        NWKDiscardRx();
                                        return NO_PRIMITIVE;
                                    }
                                }
                            }
                        }

                        // This is a new broadcast packet or a broadcast NWK command.

#ifdef I_SUPPORT_ROUTING
                        // See if the packet is a routing command
                        if (nwkFrameControlLSB.bits.frameType == NWK_FRAME_CMD)
                        {
                            i = NWKGet();

                            // See if it a route request packet
                            if ((i == NWK_COMMAND_ROUTE_REQUEST) && nwkStatus.flags.bits.bCanRoute)
                            {
                                // If we're receiving a broadcast packet from someone for
                                // our own route request, ignore it.
                                if (nwkSourceAddress.Val == macPIB.macShortAddress.Val)
                                {
                                    NWKDiscardRx();
                                    return NO_PRIMITIVE;
                                }

HandleRouteRequest:
                                {
                                    BOOL                    forwardRouteRequest;
                                    ROUTE_REQUEST_COMMAND   rreq;
                                    BYTE                    originalCost;

                                    forwardRouteRequest = FALSE;

                                    // Get the Route Request command frame
                                    for ( ptr = (BYTE *)&rreq, *ptr++ = NWK_COMMAND_ROUTE_REQUEST, i = 1;
                                          (i< sizeof(ROUTE_REQUEST_COMMAND));
                                          *ptr++ = NWKGet(), i++ ) {}

                                    originalCost = rreq.pathCost;
                                    // Update the path cost to get to us.
                                    #ifdef CALCULATE_LINK_QUALITY
                                        rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 216)
                                            rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 180)
                                            rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 144)
                                            rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 108)
                                            rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 72)
                                            rreq.pathCost += 1;
                                        if (params.MCPS_DATA_indication.mpduLinkQuality < 36)
                                            rreq.pathCost += 1;
                                    #else
                                        rreq.pathCost += CONSTANT_PATH_COST;
                                    #endif

                                #ifndef USE_TREE_ROUTING_ONLY
                                    if (HaveRoutingCapacity( TRUE, rreq.routeRequestIdentifier, nwkSourceAddress, rreq.destinationAddress, rreq.commandOptions ) &&
                                        (params.MCPS_DATA_indication.DstAddr.ShortAddr.Val == 0xFFFF))
                                    {
                                        BYTE    rdIndex;
                                        BYTE    rtIndex;

                                        // Try to find the matching route discovery entry.
                                        for (rdIndex = 0;
                                             (rdIndex < ROUTE_DISCOVERY_TABLE_SIZE) &&
                                              !((routeDiscoveryTablePointer[rdIndex] != NULL) &&
                                              (routeDiscoveryTablePointer[rdIndex]->routeRequestID == rreq.routeRequestIdentifier) &&
                                              (routeDiscoveryTablePointer[rdIndex]->srcAddress.Val == nwkSourceAddress.Val));
                                             rdIndex++ ) {}

                                        if (rdIndex < ROUTE_DISCOVERY_TABLE_SIZE)
                                        {
                                            // I am in the process of discovering this route.  Update the time stamp, in case
                                            // two nodes are trying to discover the same route, so we do not time out too soon
                                            // on one of them.
                                            routeDiscoveryTablePointer[rdIndex]->timeStamp = TickGet();

                                            // See if this new path is better than the old one.
                                            // NOTE: we have to go through this branch if the path cost equals
                                            // our current cost, in case another node is attempting a route
                                            // discovery of the same node.
                                            if (rreq.pathCost <= routeDiscoveryTablePointer[rdIndex]->forwardCost)
                                            {
                                                // Update Routing Tables with the route
                                                routeDiscoveryTablePointer[rdIndex]->senderAddress = params.MCPS_DATA_indication.SrcAddr.ShortAddr;
                                                routeDiscoveryTablePointer[rdIndex]->forwardCost = rreq.pathCost;
                                                routeDiscoveryTablePointer[rdIndex]->previousCost = rreq.pathCost - originalCost;


                                                // See if I am or one of my end devices is the destination of the route request.
                                                // Otherwise, update the path cost and forward the route request.
                                                if ((macPIB.macShortAddress.Val == rreq.destinationAddress.Val) ||
                                                    ((NWKLookupNodeByShortAddrVal( rreq.destinationAddress.Val ) != INVALID_NEIGHBOR_KEY) &&
                                                    (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD) &&
                                                    (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)))
                                                {
                                                    // Destroy the current packet and start the Route Reply
                                                    NWKDiscardRx();
                                                    if (ZigBeeReady())
                                                    {
                                                        rreq.pathCost = routeDiscoveryTablePointer[rdIndex]->previousCost;
                                                        CreateRouteReply( nwkSourceAddress, rdIndex, &rreq );
                                                        return MCPS_DATA_request;
                                                    }
                                                    else
                                                    {
                                                        return NO_PRIMITIVE;
                                                    }
                                                }
                                                else
                                                {
                                                    // Pass on the route request.
                                                    forwardRouteRequest = TRUE;
                                                }
                                            }
                                            else
                                            {
                                                // This path is worse than the one we currently have, so ignore it.
                                                NWKDiscardRx();
                                                return NO_PRIMITIVE;
                                            }
                                        }
                                        else // Route Discovery entry does not exist
                                        {
                                            // Create the Routing and Route Discovery table entries.
                                            if (!CreateRoutingTableEntries( rreq.destinationAddress, &rdIndex, &rtIndex ))
                                            {
                                                NWKDiscardRx();
                                                return NO_PRIMITIVE;
                                            }

                                            // Populate the remainder of the Route Discovery Table
                                            routeDiscoveryTablePointer[rdIndex]->routeRequestID             = rreq.routeRequestIdentifier;
                                            routeDiscoveryTablePointer[rdIndex]->srcAddress                 = nwkSourceAddress;
                                            routeDiscoveryTablePointer[rdIndex]->senderAddress              = params.MCPS_DATA_indication.SrcAddr.ShortAddr;
                                            routeDiscoveryTablePointer[rdIndex]->forwardCost                = rreq.pathCost;
                                            routeDiscoveryTablePointer[rdIndex]->status.transmitCounter     = nwkcRREQRetries + 1;
                                            routeDiscoveryTablePointer[rdIndex]->status.initialRebroadcast  = 1;
                                            routeDiscoveryTablePointer[rdIndex]->previousCost = rreq.pathCost - originalCost;

                                            // See if I am or one of my end devices is the destination of the route request.
                                            // Otherwise, unicast the route request.
                                            if ((macPIB.macShortAddress.Val == rreq.destinationAddress.Val) ||
                                                ((NWKLookupNodeByShortAddrVal( rreq.destinationAddress.Val ) != INVALID_NEIGHBOR_KEY) &&
                                                (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD) &&
                                                (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)))
                                            {
                                                // Destroy the current packet and start the Route Reply
                                                NWKDiscardRx();
                                                if (ZigBeeReady())
                                                {
                                                    rreq.pathCost -= originalCost;
                                                    CreateRouteReply( nwkSourceAddress, rdIndex, &rreq );
                                                    return MCPS_DATA_request;
                                                }
                                                else
                                                {
                                                    return NO_PRIMITIVE;
                                                }
                                            }
                                            else
                                            {
                                                 // Pass on the route request.
                                                forwardRouteRequest = TRUE;
                                            }
                                        }
                                        if (forwardRouteRequest && ZigBeeReady())
                                        {
                                            // Either this is a new route request we are forwarding, or we have a better path cost.
                                            // Buffer the RREQ so we can send it after the jitter has timed out.
                                            // If it's a new RREQ, allocate a new block of memory to buffer the message.  If it's
                                            // an existing one, overwrite the pervious information.
                                            if (!routeDiscoveryTablePointer[rdIndex]->forwardRREQ)
                                            {
                                                routeDiscoveryTablePointer[rdIndex]->forwardRREQ = SRAMalloc(sizeof(ROUTE_REQUEST_COMMAND)+8);
                                            }
                                            if (routeDiscoveryTablePointer[rdIndex]->forwardRREQ != NULL)
                                            {
                                                BYTE *ptr2;

                                                // Load the RREQ information.
                                                ptr = (BYTE *)&rreq;
                                                ptr2 = routeDiscoveryTablePointer[rdIndex]->forwardRREQ;

                                                for (i=0; i<sizeof(ROUTE_REQUEST_COMMAND); i++)
                                                {
                                                    *ptr2++ = *ptr++;       // Command, options, ID, dest addr LSB, dest addr MSB, path cost
                                                }

                                                // Load up the old NWK header (backwards).
                                                *ptr2++ = nwkSequenceNumber;
                                                *ptr2++ = nwkRadius - 1;
                                                *ptr2++ = nwkSourceAddress.byte.MSB;
                                                *ptr2++ = nwkSourceAddress.byte.LSB;
                                                *ptr2++ = nwkDestinationAddress.byte.MSB;
                                                *ptr2++ = nwkDestinationAddress.byte.LSB;
                                                *ptr2++ = nwkFrameControlMSB.Val;
                                                *ptr2++ = nwkFrameControlLSB.Val;

                                                // Reset the rebroadcast timer and the transmit counter
                                                routeDiscoveryTablePointer[rdIndex]->rebroadcastTimer       = TickGet();
                                                routeDiscoveryTablePointer[rdIndex]->status.transmitCounter = nwkcInitialRREQRetries + 1;
                                            }
                                            NWKDiscardRx();
                                            return NO_PRIMITIVE;

                                        }
                                    }
                                    else // Do not have route capacity
                                #endif
                                    {
                                        #ifndef I_AM_END_DEVICE
                                        // See if we got the message along a valid path.  The path is valid if
                                        // if was received from one of the device’s child devices and the source
                                        // device is a descendant of that child device (or that child device itself),
                                        // or if the frame was received from the device’s parent device and the
                                        // source device is not my descendant.
                                        if (((NWKLookupNodeByShortAddrVal( params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val ) != INVALID_NEIGHBOR_KEY) &&
                                                 (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD) &&
                                                 ((params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val == nwkSourceAddress.Val) ||
                                                  IsDescendant( params.MCPS_DATA_indication.SrcAddr.ShortAddr, nwkSourceAddress, currentNeighborTableInfo.depth+1 ))) ||
                                             ((params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val == macPIB.macCoordShortAddress.Val) &&
                                                !IsDescendant( macPIB.macShortAddress, nwkSourceAddress, currentNeighborTableInfo.depth )))
                                        {
                                            // See if I am or one of my end devices is the destination of the route request.
                                            // Otherwise, unicast the route request.
                                            if ((macPIB.macShortAddress.Val == rreq.destinationAddress.Val) ||
                                                ((NWKLookupNodeByShortAddrVal( rreq.destinationAddress.Val ) != INVALID_NEIGHBOR_KEY) &&
                                                 (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD) &&
                                                 (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)))
                                            {
                                                // Destroy the current packet and start the Route Reply
                                                NWKDiscardRx();
                                                if (ZigBeeReady())
                                                {
                                                    rreq.pathCost -= originalCost;
                                                    CreateRouteReply( nwkSourceAddress, INVALID_ROUTE_DISCOVERY_INDEX, &rreq );
                                                    return MCPS_DATA_request;
                                                }
                                                else
                                                {
                                                    return NO_PRIMITIVE;
                                                }
                                            }
                                            else
                                            {
                                                // Unicast the Route Request
                                                if (RouteAlongTree( nwkDestinationAddress, &(macAddress) ) && ZigBeeReady())
                                                {
                                                    // Prepare the Route Request message.
                                                    // Load up the NWK payload - the route request command frame.
                                                    ptr = (BYTE *)&rreq;
                                                    TxBuffer[TxData++] = *ptr++;       // Command
                                                    TxBuffer[TxData++] = *ptr++;       // options
                                                    TxBuffer[TxData++] = *ptr++;       // ID
                                                    TxBuffer[TxData++] = *ptr++;       // destination address LSB
                                                    TxBuffer[TxData++] = *ptr++;       // destination address MSB
                                                    TxBuffer[TxData++] = *ptr++;       // path cost

                                                    // Load up the NWK header (backwards).
                                                    TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                                                    TxBuffer[TxHeader--] = DEFAULT_RADIUS;
                                                    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                                                    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                                                    TxBuffer[TxHeader--] = nwkDestinationAddress.byte.MSB;
                                                    TxBuffer[TxHeader--] = nwkDestinationAddress.byte.LSB;
													#ifdef I_SUPPORT_SECURITY
                                                    	TxBuffer[TxHeader--] = 0x02;
													#else
                                                    	TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
													#endif
                                                    TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                                                    // Load up the MCPS_DATA.request parameters.
                                                    Prepare_MCPS_DATA_request( macAddress.Val, &i );
                                                }

                                                NWKDiscardRx();
                                                return MCPS_DATA_request;
                                            }
                                        }
                                        else
                                        #endif
                                        {
                                            // We didn't get this message from a valid path, so ignore it.
                                            NWKDiscardRx();
                                            return NO_PRIMITIVE;
                                        }
                                    }

                                }
                            }
                            // There are no other broadcast network commands.  Plus catch anywhere else
                            // that we didn't discard the packet.
                            NWKDiscardRx();
                            return NO_PRIMITIVE;
                        }
                        else
#endif  // I_SUPPORT_ROUTING
                        {
                            // The packet is a broadcast data frame.

                            BYTE    BTTIndex;
                            BYTE    dataLength;

                            // Set up the packet for rebroadcast.  The packet must have a radius > 1 to retransmit
                            if (nwkRadius > 1)
                            {
                                if (CreateNewBTR( &BTTIndex ))
                                {
                                    // We have room to store the broadcast packet.  Indicate that the message did not
                                    // originate from our upper layers.
                                    nwkStatus.BTT[BTTIndex]->btrInfo.bMessageFromUpperLayers = 0;

                                    // Mark the originator as having passive ACK'd
                                    MarkNeighborAsPasssiveACKed( BTTIndex );

                                    // Save off the NWK header information
                                    nwkStatus.BTT[BTTIndex]->dataLength                   = params.MCPS_DATA_indication.msduLength;
                                    nwkStatus.BTT[BTTIndex]->nwkFrameControlLSB           = nwkFrameControlLSB;
                                    nwkStatus.BTT[BTTIndex]->nwkFrameControlMSB           = nwkFrameControlMSB;
                                    nwkStatus.BTT[BTTIndex]->nwkDestinationAddress.Val    = nwkDestinationAddress.Val;
                                    nwkStatus.BTT[BTTIndex]->nwkSourceAddress.Val         = nwkSourceAddress.Val;
                                    nwkStatus.BTT[BTTIndex]->nwkRadius                    = nwkRadius - 1;
                                    nwkStatus.BTT[BTTIndex]->nwkSequenceNumber            = nwkSequenceNumber;

                                    // Save off the NWK payload
                                    if ((nwkStatus.BTT[BTTIndex]->dataPtr = (BYTE *)SRAMalloc( params.MCPS_DATA_indication.msduLength )) == NULL)
                                    {
                                        // We do not have room to store this broadcast packet.  Destroy the BTR
                                        // and ignore the packet.
                                        free( nwkStatus.BTT[BTTIndex] );

                                        // NWK errata - if we cannot rebroadcast the packet, then do not send up to higher layers.  We'll
                                        // see it again when the source doesn't see our rebroadcast and sends it again.
                                        NWKDiscardRx();
                                        return NO_PRIMITIVE;
                                    }
                                    else
                                    {
                                        i = 0;
                                        ptr = params.MCPS_DATA_indication.msdu;
                                        dataLength = params.MCPS_DATA_indication.msduLength;
                                        while (params.MCPS_DATA_indication.msduLength)
                                        {
                                            nwkStatus.BTT[BTTIndex]->dataPtr[i++] = NWKGet();
                                        }
                                    #ifdef I_SUPPORT_SECURITY
                                        nwkStatus.BTT[BTTIndex]->btrInfo.bAlreadySecured = 0x01;
                                    #endif

                                        // Restore the data packet so we can send it to our upper layers
                                        params.MCPS_DATA_indication.msdu = ptr;
                                        params.MCPS_DATA_indication.msduLength = dataLength;

                                        // Set the network status so we can begin transmitting these messages
                                        // in the background.  We have to process the message ourselves first!
                                        nwkStatus.flags.bits.bSendingBroadcastMessage = 1;
                                    }
                                }
                                else
                                {
                                    // NWK errata - if we cannot rebroadcast the packet, then do not send up to higher layers.
                                    NWKDiscardRx();
                                    return NO_PRIMITIVE;
                                }
                            }

                            // If we're receiving a broadcast packet from someone, but we originated it, ignore it.
                            // We still have to rebroadcast it for everyone else's ACK.
                            if (nwkSourceAddress.Val == macPIB.macShortAddress.Val)
                            {
                                NWKDiscardRx();
                                return NO_PRIMITIVE;
                            }

                            // Now we need to process the broadcast packet.  Send up an NLDE-DATA.indication.
                            // Don't discard the packet!  The upper layers are going to use it.
                            params.NLDE_DATA_indication.SrcAddress.Val = nwkSourceAddress.Val;
                            // These overlay - params.NLDE_DATA_indication.NsduLength = params.MCPS_DATA_indication.msduLength;
                            // These overlay - params.NLDE_DATA_indication.LinkQuality = params.MCPS_DATA_indication.mpduLinkQuality;
                            return NLDE_DATA_indication;
                        }
                    }
                    else
#endif // I_AM_RFD
                    {
                        // Handle all unicast frames.

                        if (nwkFrameControlLSB.bits.frameType == NWK_FRAME_CMD)
                        {
                            // This is a network command frame.
                            i = NWKGet();

                            if (i == NWK_COMMAND_LEAVE)
                            {
                            	#ifdef ZCP_DEBUG
                              		#ifndef I_AM_COORDINATOR
                                		AllowJoin = 0;
                              		#endif
                            	#endif
                            	
                                i = NWKGet();   // Get the command options byte.
                                NWKDiscardRx();
                                if (i & NWK_LEAVE_REQUEST)
                                {
                                    // We are being told to leave the network
                                    nwkStatus.flags.bits.bLeavingTheNetwork = 1;

                                    // Do not allow anyone else to join.
                                    macPIB.macAssociationPermit = FALSE;

                                    #ifndef I_AM_END_DEVICE
                                        nwkStatus.flags.bits.bRemovingChildren = i && NWK_LEAVE_REMOVE_CHILDREN;
                                        nwkStatus.flags.bits.bAllChildrenLeft = 1;
                                        nwkStatus.leaveCurrentNode = 0;
                                        if (!nwkStatus.flags.bits.bRemovingChildren)
                                        {
                                            nwkStatus.leaveCurrentNode = INVALID_NEIGHBOR_KEY;
                                        }
                                        nwkStatus.leaveStartTime = TickGet();
                                    #endif
                                    nwkStatus.leaveReason = COORDINATOR_FORCED_LEAVE;
                                }
                                #ifndef I_AM_END_DEVICE
                                else
                                {
                                    // Someone else is leaving the network
                                    i = NWKLookupNodeByShortAddrVal( nwkSourceAddress.Val );
                                    if (i != INVALID_NEIGHBOR_KEY)
                                    {
                                        RemoveNeighborTableEntry();

                                        params.NLME_LEAVE_confirm.Status = NWK_SUCCESS;
                                        memcpy( (void *)&(params.NLME_LEAVE_confirm.DeviceAddress), (void *)&(currentNeighborRecord.longAddr), 8 );
                                        return NLME_LEAVE_confirm;
                                    }
                                }
                              #endif
                                return NO_PRIMITIVE;
                            }

#ifdef I_SUPPORT_ROUTING
                            // Check for routing commands.  If MLME-START.request has not come back successful,
                            // we cannot perform any routing functions.
                            if (nwkStatus.flags.bits.bCanRoute)
                            {
                                // See if the packet is a route request
                                if (i == NWK_COMMAND_ROUTE_REQUEST)
                                {
                                    goto HandleRouteRequest;
                                }

                                else if ((i == NWK_COMMAND_ROUTE_ERROR) && ZigBeeReady())
                                {
                                    SHORT_ADDR      routeDestAddr;
                                    BYTE            routeStatus;

                                    // Read out the NWK payload.  We will probably retransmit this, so
                                    // load it into the transmit buffer.
                                    TxBuffer[TxData++] = NWK_COMMAND_ROUTE_ERROR;
                                    TxBuffer[TxData++] = NWKGet();          //errorCode
                                    routeDestAddr.byte.LSB  = NWKGet();
                                    TxBuffer[TxData++] = routeDestAddr.byte.LSB;
                                    routeDestAddr.byte.MSB  = NWKGet();
                                    TxBuffer[TxData++] = routeDestAddr.byte.MSB;

                                    // We're done with the received message.
                                    NWKDiscardRx();

                                    if (nwkDestinationAddress.Val == macPIB.macShortAddress.Val)
                                    {
                                        // We were the originator of the Route Request.
                                        // We'll let the request time out and have the background
                                        // processing clean up to conserve space.
                                        TxData = TX_DATA_START;
                                        return NO_PRIMITIVE;
                                    }
                                    else
                                    {
                                        // Forward the route error command frame.
                                        // Load up the NWK header (backwards).
                                        TxBuffer[TxHeader--] = nwkSequenceNumber;
                                        TxBuffer[TxHeader--] = nwkRadius - 1;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.LSB;
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.LSB;
                                        TxBuffer[TxHeader--] = nwkFrameControlMSB.Val;
                                        TxBuffer[TxHeader--] = nwkFrameControlLSB.Val;
        #ifndef USE_TREE_ROUTING_ONLY
                                        // Try to route back.
                                        if (!GetNextHop( nwkDestinationAddress, &macAddress, &routeStatus ))
        #endif
                                        {
                                            // If no route is available, try to route along the tree.
                                            if (!RouteAlongTree( nwkDestinationAddress, &(macAddress) ))
                                            {
                                                // We're already sending a route error.  If we can't route the
                                                // route error, there is not much we can do.
                                                TxData = TX_DATA_START;
                                                TxHeader = TX_HEADER_START;
                                                return NO_PRIMITIVE;
                                            }
                                        }
                                    }

                                    // Load up the MCPS_DATA.request parameters.
                                    Prepare_MCPS_DATA_request( macAddress.Val, &i );
                                    return MCPS_DATA_request;
                                }

                                // See if the packet is a route reply
                                else if (i == NWK_COMMAND_ROUTE_REPLY)
                                {
                                    BOOL                    forwardRouteReply;
                                    BYTE                    rdIndex;
                                    ROUTE_REPLY_COMMAND     rrep;

                                    forwardRouteReply = FALSE;
                                    rdIndex = INVALID_ROUTE_DISCOVERY_INDEX;

                                    // Get the Route Reply command frame
                                    for ( ptr = (BYTE *)&rrep, *ptr++ = NWK_COMMAND_ROUTE_REPLY, i = 1;
                                         i< sizeof(ROUTE_REPLY_COMMAND);
                                        *ptr++ = NWKGet(), i++ ) {}

        #ifndef USE_TREE_ROUTING_ONLY
                                    if (HaveRoutingCapacity( TRUE, rrep.routeRequestIdentifier, nwkSourceAddress, rrep.responderAddress, rrep.commandOptions ))
                                    {
                                        // Try to find the matching route discovery entry.
                                        for (rdIndex = 0;
                                             (rdIndex < ROUTE_DISCOVERY_TABLE_SIZE) &&
                                             !((routeDiscoveryTablePointer[rdIndex] != NULL) &&
                                             ((routeDiscoveryTablePointer[rdIndex]->routeRequestID == rrep.routeRequestIdentifier) &&
                                              (routeDiscoveryTablePointer[rdIndex]->srcAddress.Val == rrep.originatorAddress.Val)));
                                             rdIndex++ ) {}

                                        if (rdIndex != ROUTE_DISCOVERY_TABLE_SIZE)  //TableEntriesExist
                                        {
                                            #ifdef USE_EXTERNAL_NVM
                                                GetRoutingEntry( &currentRoutingEntry, routingTable + (WORD)routeDiscoveryTablePointer[rdIndex]->routingTableIndex * (WORD)sizeof(ROUTING_ENTRY) );
                                            #else
                                                GetRoutingEntry( &currentRoutingEntry, &routingTable[routeDiscoveryTablePointer[rdIndex]->routingTableIndex] );
                                            #endif
                                            currentRoutingEntry.status = ROUTE_ACTIVE;
                                            if (rrep.pathCost < routeDiscoveryTablePointer[rdIndex]->residualCost)
                                            {
                                                routeDiscoveryTablePointer[rdIndex]->residualCost = rrep.pathCost;
                                                currentRoutingEntry.nextHop = params.MCPS_DATA_indication.SrcAddr.ShortAddr;
                                            }
                                            #ifdef USE_EXTERNAL_NVM
                                                PutRoutingEntry( routingTable + (WORD)routeDiscoveryTablePointer[rdIndex]->routingTableIndex * (WORD)sizeof(ROUTING_ENTRY), &currentRoutingEntry  );
                                            #else
                                                PutRoutingEntry( &routingTable[routeDiscoveryTablePointer[rdIndex]->routingTableIndex], &currentRoutingEntry  );
                                            #endif
                                            if (macPIB.macShortAddress.Val != rrep.originatorAddress.Val) //I am not Destination
                                            {
                                                forwardRouteReply = TRUE;
                                                rrep.pathCost += routeDiscoveryTablePointer[rdIndex]->previousCost;
                                            }
                                        }
                                    }
                                    else // Do not have routing capacity
        #endif // USE_TREE_ROUTING_ONLY
                                    {
                                        if (macPIB.macShortAddress.Val != nwkDestinationAddress.Val) //!IAmDestination
                                        {
                                            rrep.pathCost += CONSTANT_PATH_COST; // We don't know the previous cost, so use a constant cost.
                                            forwardRouteReply = TRUE;
                                        }
                                    }

                                    if (forwardRouteReply && ZigBeeReady())
                                    {
                                        NWKDiscardRx();
        #ifndef USE_TREE_ROUTING_ONLY
                                        if (rdIndex != INVALID_ROUTE_DISCOVERY_INDEX)
                                        {
                                            macAddress = routeDiscoveryTablePointer[rdIndex]->senderAddress;
                                        }
                                        else
        #endif
                                        {
                                            if (!RouteAlongTree( nwkSourceAddress, &macAddress ))
                                            {
                                                return NO_PRIMITIVE;
                                            }
                                        }

                                        // Prepare the Route Reply message.
                                        // Load up the NWK payload - the route reply command frame.
                                        for ( ptr = (BYTE *)&rrep, i = 0; i< sizeof(ROUTE_REPLY_COMMAND); i++ )
                                        {
                                            TxBuffer[TxData++] = *ptr++;
                                        }

                                        // Load up the NWK header (backwards).
                                        TxBuffer[TxHeader--] = nwkSequenceNumber;           // sequence number
                                        TxBuffer[TxHeader--] = nwkRadius;                   // radius
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.MSB;   // source address MSB
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.LSB;   // source address LSB
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.MSB;  // destination MSB
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.LSB;  // destination MSB
                                        TxBuffer[TxHeader--] = nwkFrameControlMSB.Val;      // frame control MSB
                                        TxBuffer[TxHeader--] = nwkFrameControlLSB.Val;      // frame control LSB

                                        // Load up the MCPS_DATA.request parameters.
                                        Prepare_MCPS_DATA_request( macAddress.Val, &i );
                                        return MCPS_DATA_request;
                                    }
                                }
                            }

                            // I cannot route the frame, so discard it.
                            NWKDiscardRx();
                            return NO_PRIMITIVE;
#endif // I_SUPPORT_ROUTING
                        }
                        else
                        {
                            // The current frame is a data frame.
                            MESSAGE_ROUTING_STATUS  routingStatus;

                            if ((nwkDestinationAddress.Val == macPIB.macShortAddress.Val) ||
                                (nwkDestinationAddress.Val == 0xFFFF))
                            {

                                // I am the final destination for this packet.  Send up an NLDE-DATA.indication.
                                // Do not discard the packet - the upper layers need it.
                                params.NLDE_DATA_indication.SrcAddress = nwkSourceAddress;
                                // These overlay - params.NLDE_DATA_indication.NsduLength = params.MCPS_DATA_indication.msduLength;
                                // These overlay - params.NLDE_DATA_indication.LinkQuality = params.MCPS_DATA_indication.mpduLinkQuality;
                                return NLDE_DATA_indication;
                            }
#ifdef I_SUPPORT_ROUTING
                            // The packet needs to be routed.  We can route it if the radius has not reached 0,
                            // MLME-START.request has come back successful, and we're not blocked.
                            else if (nwkStatus.flags.bits.bCanRoute && (nwkRadius > 1) && ZigBeeReady())
                            {
                                routingStatus = GetRoutingAddress( FALSE, nwkDestinationAddress, nwkFrameControlLSB.bits.routeDiscovery, &macAddress );
                                switch( routingStatus )
                                {
                                    case ROUTE_SEND_TO_MAC_ADDRESS:
                                        // We know the next hop address, so send the message.
                                        // Load up the NWK header information
                                        TxBuffer[TxHeader--] = nwkSequenceNumber;
                                        TxBuffer[TxHeader--] = nwkRadius - 1;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.LSB;
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = nwkDestinationAddress.byte.LSB;
                                        TxBuffer[TxHeader--] = nwkFrameControlMSB.Val;
                                        TxBuffer[TxHeader--] = nwkFrameControlLSB.Val;

                                        // Load up the NWK payload
                                        while (params.MCPS_DATA_indication.msduLength)
                                        {
                                            TxBuffer[TxData++] = NWKGet();
                                        }

                                        // Load up the MCPS_DATA.request parameters
                                        Prepare_MCPS_DATA_request( macAddress.Val, &i );

                                        NWKDiscardRx();
                                        return MCPS_DATA_request;
                                        break;

                                    case ROUTE_MESSAGE_BUFFERED:
                                        // Send the Route Request
                                        NWKDiscardRx();
                                        return NO_PRIMITIVE;
                                        break;

                                    case ROUTE_FAILURE_NONTREE_LINK:
                                    case ROUTE_FAILURE_TREE_LINK:
                                    case ROUTE_FAILURE_NO_CAPACITY:
                                    default:
                                        // Send a Route Error command frame.
                                        // Load up the NWK header (backwards).
                                        TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                                        TxBuffer[TxHeader--] = DEFAULT_RADIUS;
                                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.MSB;
                                        TxBuffer[TxHeader--] = nwkSourceAddress.byte.LSB;
										#ifdef I_SUPPORT_SECURITY
                                        	TxBuffer[TxHeader--] = 0x02;
										#else
                                        	TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
										#endif
                                        TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                                        // Load the NWK payload into the transmit buffer.
                                        TxBuffer[TxData++] = NWK_COMMAND_ROUTE_ERROR;
                                        if (routingStatus == ROUTE_FAILURE_TREE_LINK)
                                            TxBuffer[TxData++] = ROUTE_ERROR_TREE_LINK_FAILURE;
                                        else if (routingStatus == ROUTE_FAILURE_NONTREE_LINK)
                                            TxBuffer[TxData++] = ROUTE_ERROR_NONTREE_LINK_FAILURE;
                                        else if (routingStatus == ROUTE_FAILURE_NO_CAPACITY)
                                            TxBuffer[TxData++] = ROUTE_ERROR_NO_ROUTING_CAPACITY;
                                        else
                                            TxBuffer[TxData++] = ROUTE_ERROR_NO_ROUTE_AVAILABLE;
                                        TxBuffer[TxData++] = nwkSourceAddress.byte.LSB;
                                        TxBuffer[TxData++] = nwkSourceAddress.byte.MSB;

                                        // Load up the MCPS_DATA.request parameters.
                                        // We'll just send it back to where it came from.
                                        Prepare_MCPS_DATA_request( params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val, &i );
                                        NWKDiscardRx();
                                        return MCPS_DATA_request;
                                        break;
                                }
                            }
#endif  // I_SUPPORT_ROUTING
                        }
                    }
                }
                NWKDiscardRx();
                return NO_PRIMITIVE;


            // ---------------------------------------------------------------------
//            This primitive is not referenced in the ZigBee spec.
//            case MCPS_PURGE_confirm:
//
//                return NO_PRIMITIVE;
//                break;

            // ---------------------------------------------------------------------
            #ifndef I_AM_END_DEVICE
            case MLME_ASSOCIATE_indication:
				#ifdef I_SUPPORT_SECURITY
          			// omit since both securityEnable overlap
          			//params.MLME_ASSOCIATE_response.SecurityEnable = params.MLME_ASSOCIATE_indication.SecurityEnable;
				#endif
                // Copy the indication DeviceAddress to the response DeviceAddress
                for (i=0; i<8; i++)
                {
                    params.MLME_ASSOCIATE_response.DeviceAddress.v[i] = params.MLME_ASSOCIATE_indication.DeviceAddress.v[i];
                }

                if (i = NWKLookupNodeByLongAddr( &(params.MLME_ASSOCIATE_indication.DeviceAddress) ) != INVALID_NEIGHBOR_KEY)
                {
					#ifdef I_SUPPORT_SECURITY
                    	IncomingFrameCount[0][i].Val = 0;
                    	IncomingFrameCount[1][i].Val = 0;
                    	if( params.MLME_ASSOCIATE_indication.SecurityUse != currentNeighborRecord.bSecured )
                    	{
                        	currentNeighborRecord.bSecured = params.MLME_ASSOCIATE_indication.SecurityUse;
                        	#ifdef USE_EXTERNAL_NVM
                            	PutNeighborRecord( neighborTable + (WORD)i * (WORD)sizeof(NEIGHBOR_RECORD), &currentNeighborRecord);
                        	#else
                            	PutNeighborRecord( &(neighborTable[i]), &currentNeighborRecord );
                        	#endif
                    	}
					#endif
                    params.MLME_ASSOCIATE_response.AssocShortAddress = currentNeighborRecord.shortAddr;
                    params.MLME_ASSOCIATE_response.status = ASSOCIATION_SUCCESS;
                    // DeviceAddress already copied from input
                    // SecurityEnable unchanged from input
                    return MLME_ASSOCIATE_response;
                }
                else
                {
                    // See if we can allow this node to join.
                    // NOTE: if the application wishes to disallow the join, this is the place to do it.
				#ifdef ZCP_DEBUG
                    if( accessDenied == 1 )
                    {
                        params.MLME_ASSOCIATE_response.status = ASSOCIATION_PAN_ACCESS_DENIED;
                        params.MLME_ASSOCIATE_response.AssocShortAddress.Val = 0xFFFF;
                        return MLME_ASSOCIATE_response;
                    } else if( accessDenied == 2 )
                    {
                        return NO_PRIMITIVE;
                    }
				#endif
                    if (CanAddChildNode())
                    {
                        if ((CanAddNeighborNode()) != INVALID_NEIGHBOR_KEY)
                        {
							#ifdef I_SUPPORT_SECURITY
                            	AddChildNode(params.MLME_ASSOCIATE_indication.SecurityUse);
							#else
                            	AddChildNode();
							#endif
                            // Save off the Capability Information, so we can send it back with the NLME_JOIN_indication
                            // after we get the MLME_ASSOCIATE_response.
                            nwkStatus.lastCapabilityInformation = params.MLME_ASSOCIATE_indication.CapabilityInformation;

                        #ifdef ZCP_DEBUG
                            if( bDisableShortAddress )
                            {
                                params.MLME_ASSOCIATE_response.AssocShortAddress.Val = 0xfffe;
                                params.MLME_ASSOCIATE_response.status = ASSOCIATION_SUCCESS;
                                return MLME_ASSOCIATE_response;
                            } else
                        #endif
                            params.MLME_ASSOCIATE_response.AssocShortAddress = currentNeighborRecord.shortAddr;
                            params.MLME_ASSOCIATE_response.status = ASSOCIATION_SUCCESS;
                            // DeviceAddress already copied from input
                            // SecurityEnable unchanged from input
                            return MLME_ASSOCIATE_response;
                        }
                    }
                    params.MLME_ASSOCIATE_response.status = ASSOCIATION_PAN_AT_CAPACITY;
                    params.MLME_ASSOCIATE_response.AssocShortAddress.Val = 0xFFFF;
                    // DeviceAddress already copied from input
                    // SecurityEnable unchanged from input
                    return MLME_ASSOCIATE_response;
                }
                break;
            #endif

            // ---------------------------------------------------------------------
            #ifndef I_AM_COORDINATOR
            case MLME_ASSOCIATE_confirm:
                ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 0;
                if (params.MLME_ASSOCIATE_confirm.status)
                {
                    // Association failed, so set our PAN ID back to 0xFFFF
                    ZigBeeStatus.flags.bits.bNetworkJoined = 0;
                    macPIB.macPANId.Val = 0xFFFF;
                    MLME_SET_macPANId_hw();

                    // If we need to rejoin, it will have to be as a new node.
                    ZigBeeStatus.flags.bits.bTryOrphanJoin = 0;

                    // Clear the potential parent bit and other parent fields, so we
                    // do not try to associate with this device again.
                    #ifdef USE_EXTERNAL_NVM
                        pCurrentNeighborRecord = neighborTable + (WORD)currentNeighborTableInfo.parentNeighborTableIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                    #else
                        pCurrentNeighborRecord = &neighborTable[currentNeighborTableInfo.parentNeighborTableIndex];
                    #endif
                    GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
                    currentNeighborRecord.deviceInfo.bits.PotentialParent = 0;
                    currentNeighborTableInfo.parentNeighborTableIndex = INVALID_NEIGHBOR_KEY;
                    PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );

                    // If there are any more potential parents, we have to try again...
                    goto TryToJoinPotentialParent;
                }
                else
                {
                    // I have joined a network.  Set my address information
                    macPIB.macShortAddress = params.MLME_ASSOCIATE_confirm.AssocShortAddress;

SetMyAddressInformation:
                    ZigBeeStatus.flags.bits.bNetworkJoined = 1;
                    MLME_SET_macShortAddress_hw();

                    // Set parent relationship in neighbor table
                    #ifdef USE_EXTERNAL_NVM
                        pCurrentNeighborRecord = neighborTable + (WORD)currentNeighborTableInfo.parentNeighborTableIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
                    #else
                        pCurrentNeighborRecord = &neighborTable[currentNeighborTableInfo.parentNeighborTableIndex];
                        GetNeighborRecord( &currentNeighborRecord, (ROM void*)pCurrentNeighborRecord );
                    #endif
                    currentNeighborRecord.deviceInfo.bits.Relationship = NEIGHBOR_IS_PARENT;
                    memcpy( (void *)&currentNeighborRecord.longAddr, (void *)&macPIB.macCoordExtendedAddress, 8 );
                    PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );

                    macPIB.macCoordShortAddress = currentNeighborRecord.shortAddr;

                    // If we need to rejoin later, we can do it as an orphan.
                    ZigBeeStatus.flags.bits.bTryOrphanJoin = 1;

                    // Initialize my information.  We'll do it here since we have our parent's depth.
                    #ifndef I_AM_END_DEVICE
                        currentNeighborTableInfo.depth                  = currentNeighborRecord.deviceInfo.bits.Depth+1; // Our depth in the network
                        currentNeighborTableInfo.cSkip.Val              = GetCSkipVal( currentNeighborTableInfo.depth ); // Address block size
                        if (!currentNeighborTableInfo.flags.bits.bChildAddressInfoValid)
                        {
                            currentNeighborTableInfo.nextEndDeviceAddr.Val  = macPIB.macShortAddress.Val + currentNeighborTableInfo.cSkip.Val * NIB_nwkMaxRouters + 1;  // Next address available to give to an end device
                            currentNeighborTableInfo.nextRouterAddr.Val     = macPIB.macShortAddress.Val + 1;     // Next address available to give to a router
                            currentNeighborTableInfo.numChildren            = 0;
                            currentNeighborTableInfo.numChildRouters        = 0;
                            currentNeighborTableInfo.flags.bits.bChildAddressInfoValid = 1;
                        }
                        SetBeaconPayload();
                    #endif

                    // Store the final parentNeighborTableIndex and other information in NVM.
                    PutNeighborTableInfo();
                }

				#ifdef I_SUPPORT_SECURITY
                	securityStatus.flags.bits.bAuthorization = TRUE;
                	AuthorizationTimeout = TickGet();
    				#ifdef I_AM_RFD
                		PHYTasksPending.bits.PHY_AUTHORIZE = 1;
                		lastPollTime = AuthorizationTimeout;
    				#endif
				#endif

                #ifdef I_AM_RFD
                    ZigBeeStatus.flags.bits.bRequestingData = 0;
//                    ZigBeeStatus.flags.bits.bDataRequestComplete = 1;
                #endif

                // Status is already in place.
                params.NLME_JOIN_confirm.PANId = macPIB.macPANId;
                return NLME_JOIN_confirm;
                break;
            #endif

            // ---------------------------------------------------------------------
            //#ifndef I_AM_END_DEVICE
//            We do not do anything with this indication, as allowed by the ZigBee spec.
//            We handle everything with the NWK LEAVE command.
			#ifdef ZCP_DEBUG
            case MLME_DISASSOCIATE_indication:
            	#ifdef ZCP_PRINTOUT
                printf("MLME_DISASSOCIATE_indication Result: ");
                printf("\r\nDevice Address: ");
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[7]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[6]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[5]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[4]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[3]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[2]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[1]);
                PrintChar(params.MLME_DISASSOCIATE_indication.DeviceAddress.v[0]);
                printf("\r\nDisassociate Reason: ");
                PrintChar(params.MLME_DISASSOCIATE_indication.DisassociateReason);
                printf("\r\nACL Entry: ");
                PrintChar(params.MLME_DISASSOCIATE_indication.ACLEntry);
                printf("\r\n");
                if( params.MLME_DISASSOCIATE_indication.DisassociateReason == COORDINATOR_FORCED_LEAVE )
                {
                    params.MLME_DISASSOCIATE_confirm.status = SUCCESS;
                    macPIB.macPANId.Val = 0xFFFF;
                    macPIB.macShortAddress.Val = 0xFFFF;
                    printf("\r\nMAC PIB: ");
                    printf("\r\nMAC PANID: ");
                    PrintChar(macPIB.macPANId.v[1]);
                    PrintChar(macPIB.macPANId.v[0]);
                    printf("\r\nMAC Short Address: ");
                    PrintChar(macPIB.macShortAddress.v[1]);
                    PrintChar(macPIB.macShortAddress.v[0]);
                    printf("\r\n");
                    MLME_SET_macShortAddress_hw();
                }
            	#endif
                break;
			#endif

            // ---------------------------------------------------------------------
            case MLME_DISASSOCIATE_confirm:
                // The only way we get this is if we are in the process of leaving the network.
                // The leave is almost complete.  Set the final flag to perform a reset.
                nwkStatus.flags.bits.bLeaveWaitForConfirm = 0;
                nwkStatus.flags.bits.bLeaveReset = 1;

                // We have left the network.  Remove our parent from the neighbor table.
                i = NWKLookupNodeByShortAddrVal( macPIB.macCoordShortAddress.Val );
                if (i != INVALID_NEIGHBOR_KEY)
                {
                    RemoveNeighborTableEntry();
                }

                // Notify our upper layers that we have left the network.  We need to send
                // back a different primitive depending on why we are leaving the network.
                // But both have DeviceAddress at the same location
                GetMACAddress( &params.NLME_LEAVE_confirm.DeviceAddress );

                if (nwkStatus.leaveReason == SELF_INITIATED_LEAVE)
                {
                    #ifdef I_AM_END_DEVICE
                    if (!params.MLME_DISASSOCIATE_confirm.status)
                    #else
                    if (!params.MLME_DISASSOCIATE_confirm.status &&
                        (!nwkStatus.flags.bits.bRemovingChildren ||
                         (!currentNeighborTableInfo.numChildren && nwkStatus.flags.bits.bAllChildrenLeft)))
                    #endif
                    {
                        params.NLME_LEAVE_confirm.Status = NWK_SUCCESS;
                    }
                    else
                    {
                        params.NLME_LEAVE_confirm.Status = NWK_LEAVE_UNCONFIRMED;
                    }
                    return NLME_LEAVE_confirm;
                }
                else
                {
                    return NLME_LEAVE_indication;
                }
                break;

            // ---------------------------------------------------------------------
            case MLME_BEACON_NOTIFY_indication:
                // If we did not get a short address or if the protocol identifier is wrong,
                // then the node is not a ZigBee node.
			#ifdef ZCP_PRINTOUT
                printf("\r\nBeacon Notify Indication Result:");
                printf("\r\nCoordAddrMode: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddrMode);
                printf("\r\nCoordPANId: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordPANId.v[1]);
                PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordPANId.v[0]);
                printf("\r\nCoordAddress: ");
                if( params.MLME_BEACON_NOTIFY_indication.CoordAddrMode == 0x02)
                {
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.ShortAddr.v[1]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.ShortAddr.v[0]);
                } else {
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[7]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[6]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[5]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[4]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[3]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[2]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[1]);
                    PrintChar(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[0]);
                }
                printf("\r\nLogical Channel: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.LogicalChannel);
                printf("\r\nSuperframeSpec: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.byte.MSB);
                PrintChar(params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.byte.LSB);
                printf("\r\nLinkQuality: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.LinkQuality);
                printf("\r\nTimeStamp: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.TimeStamp.v[3]);
                PrintChar(params.MLME_BEACON_NOTIFY_indication.TimeStamp.v[2]);
                PrintChar(params.MLME_BEACON_NOTIFY_indication.TimeStamp.v[1]);
                PrintChar(params.MLME_BEACON_NOTIFY_indication.TimeStamp.v[0]);
                printf("\r\nACLEntry: ");
                PrintChar(params.MLME_BEACON_NOTIFY_indication.ACLEntry);
			#endif
                if ((params.MLME_BEACON_NOTIFY_indication.CoordAddrMode == 0x02) &&
                    (params.MLME_BEACON_NOTIFY_indication.sdu[0] == ZIGBEE_PROTOCOL_ID))
                {
                    if (NWKLookupNodeByShortAddrVal( params.MLME_BEACON_NOTIFY_indication.CoordAddress.ShortAddr.Val ) != INVALID_NEIGHBOR_KEY)
                    {
                        // This is an old node.  Reset some of the fields.
                        goto ResetNeighborTableFields;
                    }
                    else
                    {
                        // This is a new ZigBee node, so try to add it to the neighbor table.
                        if (CanAddNeighborNode() != INVALID_NEIGHBOR_KEY)
                        {
                            // Set all the information for a new node, that could get updated later.
                            currentNeighborRecord.deviceInfo.bits.bInUse            = 1;
                            currentNeighborRecord.deviceInfo.bits.Relationship      = NEIGHBOR_IS_NONE;

                            // We don't know the long address from the beacon, so set it to 0's.
                            for (i=0; i<8; i++)
                            {
                                currentNeighborRecord.longAddr.v[i] = 0x00;
                            }

                            // Update the neighbor table size.
                            currentNeighborTableInfo.neighborTableSize++;
                            PutNeighborTableInfo();

ResetNeighborTableFields:
                            // Set/Update the fields that may need updating.
                            if (params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.bits.PANCoordinator)
                            {
                                currentNeighborRecord.deviceInfo.bits.deviceType        = DEVICE_ZIGBEE_COORDINATOR;
                            }
                            else
                            {
                                currentNeighborRecord.deviceInfo.bits.deviceType        = DEVICE_ZIGBEE_ROUTER;
                            }
                            currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle      = 1;
                            currentNeighborRecord.deviceInfo.bits.Depth             = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x78) >> 3;
                            currentNeighborRecord.deviceInfo.bits.StackProfile      = (params.MLME_BEACON_NOTIFY_indication.sdu[1] & 0x0F);
                            currentNeighborRecord.deviceInfo.bits.ZigBeeVersion     = (params.MLME_BEACON_NOTIFY_indication.sdu[1] & 0xF0) >> 4;
                            currentNeighborRecord.deviceInfo.bits.LQI               = params.MLME_BEACON_NOTIFY_indication.LinkQuality;
                            currentNeighborRecord.deviceInfo.bits.PermitJoining     = params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.bits.AssociationPermit;

                            #ifdef I_AM_ROUTER
                                currentNeighborRecord.deviceInfo.bits.PotentialParent   = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x04) == 0x04;
                            #else
                                currentNeighborRecord.deviceInfo.bits.PotentialParent   = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x80) == 0x80;
                            #endif

                            currentNeighborRecord.panID             = params.MLME_BEACON_NOTIFY_indication.CoordPANId;
                            currentNeighborRecord.shortAddr         = params.MLME_BEACON_NOTIFY_indication.CoordAddress.ShortAddr;
                            currentNeighborRecord.LogicalChannel    = params.MLME_BEACON_NOTIFY_indication.LogicalChannel;

                            PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord);
                        }
                    }
                }
			#ifdef ZCP_DEBUG
                else if(params.MLME_BEACON_NOTIFY_indication.CoordAddrMode == 0x03)
                {
                    if (NWKLookupNodeByLongAddr( &(params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr) ) != INVALID_NEIGHBOR_KEY)
                    {
                        // This is an old node.  Reset some of the fields.
                        goto ResetNeighborTableFields2;
                    }
                    else
                    {
                        // This is a new ZigBee node, so try to add it to the neighbor table.
                        if (CanAddNeighborNode() != INVALID_NEIGHBOR_KEY)
                        {
                            // Set all the information for a new node, that could get updated later.
                            currentNeighborRecord.deviceInfo.bits.bInUse            = 1;
                            currentNeighborRecord.deviceInfo.bits.Relationship      = NEIGHBOR_IS_NONE;

                            // We don't know the long address from the beacon, so set it to 0's.
                            for (i=0; i<8; i++)
                            {
                                currentNeighborRecord.longAddr.v[i] = params.MLME_BEACON_NOTIFY_indication.CoordAddress.LongAddr.v[i];
                            }

                            // Update the neighbor table size.
                            currentNeighborTableInfo.neighborTableSize++;
                            PutNeighborTableInfo();

ResetNeighborTableFields2:
                            // Set/Update the fields that may need updating.
                            if (params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.bits.PANCoordinator)
                            {
                                currentNeighborRecord.deviceInfo.bits.deviceType        = DEVICE_ZIGBEE_COORDINATOR;
                            }
                            else
                            {
                                currentNeighborRecord.deviceInfo.bits.deviceType        = DEVICE_ZIGBEE_ROUTER;
                            }
                            currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle      = 1;
                            currentNeighborRecord.deviceInfo.bits.Depth             = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x78) >> 3;
                            currentNeighborRecord.deviceInfo.bits.StackProfile      = (params.MLME_BEACON_NOTIFY_indication.sdu[1] & 0x0F);
                            currentNeighborRecord.deviceInfo.bits.ZigBeeVersion     = (params.MLME_BEACON_NOTIFY_indication.sdu[1] & 0xF0) >> 4;
                            currentNeighborRecord.deviceInfo.bits.LQI               = params.MLME_BEACON_NOTIFY_indication.LinkQuality;
                            currentNeighborRecord.deviceInfo.bits.PermitJoining     = params.MLME_BEACON_NOTIFY_indication.SuperframeSpec.bits.AssociationPermit;

                            #ifdef I_AM_ROUTER
                                currentNeighborRecord.deviceInfo.bits.PotentialParent   = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x04) == 0x04;
                            #else
                                currentNeighborRecord.deviceInfo.bits.PotentialParent   = (params.MLME_BEACON_NOTIFY_indication.sdu[2] & 0x80) == 0x80;
                            #endif

                            currentNeighborRecord.panID             = params.MLME_BEACON_NOTIFY_indication.CoordPANId;
                            currentNeighborRecord.shortAddr.Val         = 0xfffe;
                            currentNeighborRecord.LogicalChannel    = params.MLME_BEACON_NOTIFY_indication.LogicalChannel;

                            PutNeighborRecord( pCurrentNeighborRecord,  &currentNeighborRecord);
                        }
                    }

                }

			#endif

                NWKDiscardRx();
                return NO_PRIMITIVE;
                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive.  We just get the values directly.
//            case MLME_GET_confirm:
//                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive - we only support non-beacon networks.
//            case MLME_GTS_confirm:
//                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive - we only support non-beacon networks.
//            case MLME_GTS_indication:
//                break;

            // ---------------------------------------------------------------------
            #ifndef I_AM_END_DEVICE
            case MLME_ORPHAN_indication:
                i = NWKLookupNodeByLongAddr( &(params.MLME_ORPHAN_indication.OrphanAddress) );
                if (i != INVALID_NEIGHBOR_KEY)
                {
                    // currentNeighborRecord already has the neighbor table entry.
                    params.MLME_ORPHAN_response.ShortAddress = currentNeighborRecord.shortAddr;
                    params.MLME_ORPHAN_response.AssociatedMember = TRUE;
					#ifdef I_SUPPORT_SECURITY
						#ifdef I_SUPPORT_SECURITY_SPEC
                    		params.MLME_ORPHAN_response.SecurityEnable = params.MLME_ORPHAN_indication.SecurityUse;
                    		IncomingFrameCount[0][i].Val = 0;
                    		IncomingFrameCount[1][i].Val = 0;
						#else
                    		IncomingFrameCount[0][i].Val = 0;
                    		IncomingFrameCount[1][i].Val = 0;
                    		params.MLME_ORPHAN_response.SecurityEnable = FALSE;
						#endif
					#endif
                    return MLME_ORPHAN_response;
                }

                //(DF) - this block is not needed because any call of MLME_ORPHAN_response with
                //      AssociatedMember = FALSE will be ignored by the spec.  I have removed the
                //      check for it in MLME_ORPHAN_response and thus must remove the call when it
                //      is false here as well.
//                else
//                {
//                    params.MLME_ORPHAN_response.AssociatedMember = FALSE;
//                    // TODO how to set security?  params.MLME_ORPHAN_response.SecurityEnable
//                    return MLME_ORPHAN_response;
//                }
                break;
            #endif

            // ---------------------------------------------------------------------
            case MLME_RESET_confirm:
                // Reset all NWK layer variables and routing tables.  We do NOT clear the
                // neighbor table.
                // Clear all static variables.
                VariableAndTableInitialization( FALSE );

                // Status is already in place
                return NLME_RESET_confirm;
                break;

            // ---------------------------------------------------------------------
//            TODO this is not referenced in the ZigBee spec.  It seems rather important...
//            case MLME_RX_ENABLE_confirm:
//                return NO_PRIMITIVE;
//                break;

            // ---------------------------------------------------------------------
            case MLME_SCAN_confirm:
                #ifdef I_AM_COORDINATOR
                    if (params.MLME_SCAN_confirm.ScanType == MAC_SCAN_ENERGY_DETECT)
                    {
                        BYTE            j;
                        CHANNEL_INFO    tempChannel;

                        if (!params.MLME_SCAN_confirm.ResultListSize)
                        {
NetworkStartupFailure:
                            ZigBeeStatus.flags.bits.bTryingToFormNetwork = 0;
                            params.NLME_NETWORK_FORMATION_confirm.Status = NWK_STARTUP_FAILURE;
                            return NLME_NETWORK_FORMATION_confirm;
                        }

                        // Reserve space for the largest number of results, since a run time multiplication can be expensive.
                        if ((nwkStatus.discoveryInfo.channelList = (CHANNEL_INFO *)SRAMalloc( 16 * sizeof(CHANNEL_INFO) )) == NULL)
                        {
                            if (params.MLME_SCAN_confirm.EnergyDetectList)
                            {
                                free( params.MLME_SCAN_confirm.EnergyDetectList );
                            }
                            goto NetworkStartupFailure;
                        }

                        // Create the list of scanned channels.  If we asked to scan the channel
                        // before, and the channel wasn't skipped, and the energy is below the threshold,
                        // then record the channel and its energy.
                        nwkStatus.discoveryInfo.numChannels = 0;
                        ptr = params.MLME_SCAN_confirm.EnergyDetectList;
					#ifdef ZCP_PRINTOUT
                        printf("Scan Confirm Result(Energy Scan):");
                        printf("\r\nUnscanned Channels: ");
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[3]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[2]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[1]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[0]);
                        printf("\r\nResultListSize: ");
                        PrintChar(params.MLME_SCAN_confirm.ResultListSize);
                        printf("\r\nEnergyDetectList: ");
                        for(i=0; i < params.MLME_SCAN_confirm.ResultListSize; i++)
                            PrintChar(params.MLME_SCAN_confirm.EnergyDetectList[i]);
                        printf("\r\n");
					#endif
                        for (i=0; i<32; i++)
                        {
                            if (nwkStatus.lastScanChannels.Val & 0x00000001)
                            {
                                if (!(params.MLME_SCAN_confirm.UnscannedChannels.Val & 0x00000001))
                                {
                                    if (*ptr < MAX_ENERGY_THRESHOLD)
                                    {
                                        nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.numChannels].channel = i;
                                        nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.numChannels].energy = *ptr;
                                        nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.numChannels].networks = 0;
                                        nwkStatus.discoveryInfo.numChannels++;
                                    }
                                    ptr++;
                                }
                            }
                            // Get ready to check the next channel.
                            params.MLME_SCAN_confirm.UnscannedChannels.Val >>= 1;
                            nwkStatus.lastScanChannels.Val >>= 1;
                        }

                        // Sort the channels from lowest to highest energy.  Since i is unsigned,
                        // our i is always one greater than normal so we can exit on 0.
                        for (i=nwkStatus.discoveryInfo.numChannels; i>0; i--)
                        {
                            for (j=1; j<=i-1; j++)
                            {
                                if (nwkStatus.discoveryInfo.channelList[j].energy < nwkStatus.discoveryInfo.channelList[j-1].energy)
                                {
                                    tempChannel = nwkStatus.discoveryInfo.channelList[j-1];
                                    nwkStatus.discoveryInfo.channelList[j-1] = nwkStatus.discoveryInfo.channelList[j];
                                    nwkStatus.discoveryInfo.channelList[j] = tempChannel;
                                }
                            }
                        }

                        // Destroy the energy detect list.
                        if (params.MLME_SCAN_confirm.EnergyDetectList)
                        {
                            free( params.MLME_SCAN_confirm.EnergyDetectList );
                        }

                        if (!nwkStatus.discoveryInfo.numChannels)
                        {
                            // If there were no channels with energy below the threshold, return.
                            free( nwkStatus.discoveryInfo.channelList );
                            goto NetworkStartupFailure;
                        }
                        else
                        {
                            // Perform an active scan to check for networks on the first channel.
                            nwkStatus.discoveryInfo.currentIndex = 0;

StartActiveScan:
                            params.MLME_SCAN_request.ScanType = MAC_SCAN_ACTIVE_SCAN;
                            params.MLME_SCAN_request.ScanChannels.Val = (DWORD)1 << nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].channel;
                            params.MLME_SCAN_request.ScanDuration = nwkStatus.lastScanDuration;
                            return MLME_SCAN_request;
                        }
                    }
                    else // active scan
                    {
                        nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].networks = params.MLME_SCAN_confirm.ResultListSize;

                        // We just need the count of networks.  The beacon info is in our neighbor table.
                        while ((BYTE *)params.MLME_SCAN_confirm.PANDescriptorList)
                        {
                            ptr = (BYTE *)params.MLME_SCAN_confirm.PANDescriptorList->next;
                            free( params.MLME_SCAN_confirm.PANDescriptorList );
                            params.MLME_SCAN_confirm.PANDescriptorList = (PAN_DESCRIPTOR *)ptr;
                        }

                        if (++nwkStatus.discoveryInfo.currentIndex != nwkStatus.discoveryInfo.numChannels)
                        {
                            // Perform an active scan on the next channel.
                            goto StartActiveScan;
                        }

                        // We have a count of all networks on all available channels.  Now find the channel with the
                        // least number of networks.

                        nwkStatus.discoveryInfo.currentIndex = 0;
                        params.MLME_START_request.PANId = nwkStatus.requestedPANId;

                        // If the upper layers requested a PAN ID, set our index to the first network that
                        // does not have a PAN ID conflict.  If we cannot find a channel, we will override
                        // the requested PAN ID.
                        if (params.MLME_START_request.PANId.Val != 0xFFFF)
                        {
                            while (RequestedPANIDFound(nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].channel) &&
                                   (nwkStatus.discoveryInfo.currentIndex < nwkStatus.discoveryInfo.numChannels))
                            {
                                nwkStatus.discoveryInfo.currentIndex++;
                            }
                        }
                        if (nwkStatus.discoveryInfo.currentIndex == nwkStatus.discoveryInfo.numChannels)
                        {
                            // We cannot use the requested PAN ID, so we'll override it.
                            params.MLME_START_request.PANId.Val = 0xFFFF;
                            nwkStatus.discoveryInfo.currentIndex = 0;
                        }

                        // Find the channel with the least number of networks that does not have a PAN ID conflict.
                        for (i=1; i<nwkStatus.discoveryInfo.numChannels; i++)
                        {
                            if (nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].networks > nwkStatus.discoveryInfo.channelList[i].networks)
                            {
                                if ((params.MLME_START_request.PANId.Val == 0xFFFF) || !RequestedPANIDFound( nwkStatus.discoveryInfo.channelList[i].channel ))
                                {
                                    nwkStatus.discoveryInfo.currentIndex = i;
                                }
                            }
                        }

                        // If the user did not specify a PAN ID, go pick one now.  There's a possibility
                        // here for an infinite loop, but in practice, we should be able to find a unique value.
                        if (params.MLME_START_request.PANId.Val == 0xFFFF)
                        {
                            do
                            {
                                params.MLME_START_request.PANId.byte.LSB    = RANDOM_LSB;
                                params.MLME_START_request.PANId.byte.MSB    = RANDOM_MSB & 0x3F;
                            } while (RequestedPANIDFound( nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].channel ));
                        }

                        // Set our address as ZigBee Coordinator.  Do not set the PAN ID in the PIB -
                        // we might have to send out a realignment first.  The MAC layer will set the value in the PIB.
                        macPIB.macShortAddress.Val = 0x0000;
                        MLME_SET_macShortAddress_hw();

                        // Clear the ability to route frames.
                        nwkStatus.flags.bits.bCanRoute = 0;

                        // Clean up the neighbor table to remove nodes that are not our children.
                        NWKTidyNeighborTable();

                        params.MLME_START_request.LogicalChannel    = nwkStatus.discoveryInfo.channelList[nwkStatus.discoveryInfo.currentIndex].channel;
                        params.MLME_START_request.BeaconOrder       = MAC_PIB_macBeaconOrder;
                        params.MLME_START_request.SuperframeOrder   = MAC_PIB_macSuperframeOrder;
                        params.MLME_START_request.fields.Val        = MLME_START_IS_PAN_COORDINATOR;
                        params.MLME_START_request.fields.bits.BatteryLifeExtension = macPIB.macBattLifeExtPeriods;
                        //TODO add security  params.MLME_START_request.fields.bits.SecurityEnable = ???;
                        return MLME_START_request;
                    }

                #else

                    // Router and End Device
                    if (params.MLME_SCAN_confirm.ScanType == MAC_SCAN_ACTIVE_SCAN)
                    {
                        // A Router or End Device is doing an active scan to find a network to join

                        NETWORK_DESCRIPTOR *newNetwork;
						PAN_DESCRIPTOR *panDes;

                        if (params.MLME_SCAN_confirm.status)
                        {
                            // The scan was not successful.  Status is already in place, and PANDescriptorList is already free.
                            ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 0;
                            return NLME_NETWORK_DISCOVERY_confirm;
                        }
					#ifdef ZCP_PRINTOUT
                        printf("\r\nScan Confirm Result(Active Scan): \r\n");
                        panDes = params.MLME_SCAN_confirm.PANDescriptorList;
                        for(i = 0; i < params.MLME_SCAN_confirm.ResultListSize; i++)
                        {
                            printf("PAN ");
                            PrintChar(i);
                            printf(": ");
                            PrintChar(panDes->LogicalChannel);
                            printf(" ");
                            PrintChar(panDes->CoordPANId.v[1]);
                            PrintChar(panDes->CoordPANId.v[0]);
                            printf(" ");

                            if( panDes->CoordAddrMode == 0x00 )
                            {
                                PrintChar(panDes->CoordAddress.ShortAddr.v[1]);
                                PrintChar(panDes->CoordAddress.ShortAddr.v[0]);
                            } else {
                                PrintChar(panDes->CoordAddress.LongAddr.v[7]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[6]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[5]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[4]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[3]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[2]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[1]);
                                PrintChar(panDes->CoordAddress.LongAddr.v[0]);
                            }
                            printf(" ");
                            PrintChar(panDes->SuperframeSpec.byte.MSB);
                            PrintChar(panDes->SuperframeSpec.byte.LSB);
                            printf(" ");
                            PrintChar(panDes->LinkQuality);
                            printf("\r\n");
                            panDes = panDes->next;
                        }
					#endif


                        params.NLME_NETWORK_DISCOVERY_confirm.NetworkCount = 0;
                        params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor = NULL;

                        // Process each PAN descriptor in the list
                        while ((BYTE *)params.MLME_SCAN_confirm.PANDescriptorList)
                        {
                            // Make sure we don't already have this network in our list
                            ptr = (BYTE *)params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor;

                            while (ptr && (((NETWORK_DESCRIPTOR *)ptr)->PanID.Val != params.MLME_SCAN_confirm.PANDescriptorList->CoordPANId.Val))
                            {
                                ptr = (BYTE *)params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor->next;
                            }

                            if (ptr == NULL)
                            {
                                // The PAN is not in our list.  Create an entry for it.

                                // Make sure we have the node in our neighbor table.  If not, we cannot fill out the info.
                                if (params.MLME_SCAN_confirm.PANDescriptorList->CoordAddrMode)
                                {
                                    i = NWKLookupNodeByLongAddr( &(params.MLME_SCAN_confirm.PANDescriptorList->CoordAddress.LongAddr) );
                                }
                                else
                                {
                                    i = NWKLookupNodeByShortAddrVal( params.MLME_SCAN_confirm.PANDescriptorList->CoordAddress.ShortAddr.Val );
                                }

                                if (i != INVALID_NEIGHBOR_KEY)
                                {
                                    if ((newNetwork = (NETWORK_DESCRIPTOR *)SRAMalloc( sizeof(NETWORK_DESCRIPTOR) )) != NULL)
                                    {
                                        newNetwork->PanID           = params.MLME_SCAN_confirm.PANDescriptorList->CoordPANId;
                                        newNetwork->LogicalChannel  = params.MLME_SCAN_confirm.PANDescriptorList->LogicalChannel;
                                        newNetwork->StackProfile    = currentNeighborRecord.deviceInfo.bits.StackProfile;
                                        newNetwork->ZigBeeVersion   = currentNeighborRecord.deviceInfo.bits.ZigBeeVersion;
                                        newNetwork->BeaconOrder     = params.MLME_SCAN_confirm.PANDescriptorList->SuperframeSpec.bits.BeaconOrder;
                                        newNetwork->SuperframeOrder = params.MLME_SCAN_confirm.PANDescriptorList->SuperframeSpec.bits.SuperframeOrder;
                                        newNetwork->PermitJoining   = params.MLME_SCAN_confirm.PANDescriptorList->SuperframeSpec.bits.AssociationPermit;
                                        //newNetwork->SecurityLevel   = can't get this now - it's no longer in the beacon payload
                                        newNetwork->next            = params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor;

                                        params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor = newNetwork;
                                        params.NLME_NETWORK_DISCOVERY_confirm.NetworkCount++;
                                    }
                                }
                            }

                            ptr = (BYTE *)params.MLME_SCAN_confirm.PANDescriptorList->next;
                            free( params.MLME_SCAN_confirm.PANDescriptorList );
                            params.MLME_SCAN_confirm.PANDescriptorList = (PAN_DESCRIPTOR *)ptr;
                        }

                        // If we didn't find any networks, we are no longer in the process of trying to join a network
                        if (!params.NLME_NETWORK_DISCOVERY_confirm.NetworkCount)
                        {
                            ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 0;
                        }

                        // Status is already in place
                        return NLME_NETWORK_DISCOVERY_confirm;
                    }
                    else
                    {
                        // We just did an orphan scan.  Return the result to the next layer.  If we got SUCCESS,
                        // pass it back up and go set all of our address, parent, and join information.  Otherwise,
                        // change the status to NWK_NO_NETWORKS (instead of NO_BEACON - NWK errata)
                        ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 0;
					#ifdef ZCP_PRINTOUT
                        printf("\r\nScan Confirm Result(Orphan Scan):");
                        printf("\r\nUnscanned Channels: ");
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[3]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[2]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[1]);
                        PrintChar(params.MLME_SCAN_confirm.UnscannedChannels.v[0]);
                        printf("\r\nResultListSize: ");
                        PrintChar(params.MLME_SCAN_confirm.ResultListSize);
                        printf("\r\nmacShortAddress: ");
                        PrintChar(macPIB.macShortAddress.byte.MSB);
                        PrintChar(macPIB.macShortAddress.byte.LSB);
                        printf("\r\nmacPANID: ");
                        PrintChar(macPIB.macPANId.byte.MSB);
                        PrintChar(macPIB.macPANId.byte.LSB);
                        printf("\r\n");
					#endif
                        if (!params.MLME_SCAN_confirm.status)
                        {
                            // The orphan scan was successful.  Set our PAN ID in the hardware.
                            MLME_SET_macPANId_hw();

                            // Set the rest of my address and parent information, and return NLME_JOIN_confirm.
                            // Note that parentNeighborTableIndex is already set.
                            goto SetMyAddressInformation;
                        }
                        else
                        {
                            // Moved to allow more than one orphan attempt.
//                            // The orphan scan failed.  Clear our old parent relationship.
//                            pCurrentNeighborRecord = &(neighborTable[currentNeighborTableInfo.parentNeighborTableIndex]);
//                            GetNeighborRecord(&currentNeighborRecord, (ROM void*)pCurrentNeighborRecord );
//                            currentNeighborRecord.deviceInfo.bits.Relationship = NEIGHBOR_IS_NONE;
//                            PutNeighborRecord((NVM_ADDR*)pCurrentNeighborRecord,(BYTE*)&currentNeighborRecord );
//
//                            currentNeighborTableInfo.parentNeighborTableIndex = INVALID_NEIGHBOR_KEY;
//                            PutNeighborTableInfo();
//
//                            // If we try to join again, it will have to be as a new node.
//                            ZigBeeStatus.flags.bits.bTryOrphanJoin = 0;

                            params.NLME_JOIN_confirm.Status = NWK_NO_NETWORKS;
                        }
                        return NLME_JOIN_confirm;
                    }
                #endif
                break;

            // ---------------------------------------------------------------------
            case MLME_COMM_STATUS_indication:
                #if defined(I_AM_COORDINATOR) || defined(I_AM_ROUTER)
					#ifdef ZCP_PRINTOUT
                    printf("\r\nMLME_COMM_STATUS_indication Result: ");
                    printf("\r\nPANID: ");
                    PrintChar(params.MLME_COMM_STATUS_indication.PANId.v[1]);
                    PrintChar(params.MLME_COMM_STATUS_indication.PANId.v[0]);
                    printf("\r\nSrcAddress: ");
                    if( params.MLME_COMM_STATUS_indication.SrcAddrMode == 0x03 )
                    {
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[7]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[6]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[5]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[4]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[3]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[2]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[1]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.LongAddr.v[0]);
                    } else {
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.ShortAddr.v[1]);
                        PrintChar(params.MLME_COMM_STATUS_indication.SrcAddr.ShortAddr.v[0]);
                    }
                    printf("\r\nDstAddress: ");
                    if( params.MLME_COMM_STATUS_indication.DstAddrMode == 0x03 )
                    {
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[7]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[6]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[5]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[4]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[3]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[2]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[1]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr.v[0]);
                    } else {
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.ShortAddr.v[1]);
                        PrintChar(params.MLME_COMM_STATUS_indication.DstAddr.ShortAddr.v[0]);
                    }
					#endif
                    i = NWKLookupNodeByLongAddr( &(params.MLME_COMM_STATUS_indication.DstAddr.LongAddr) );
                    if (params.MLME_COMM_STATUS_indication.status)
                    {
                        // We could not send the message to the node.
                        if (i != INVALID_NEIGHBOR_KEY)
                        {
                            // Remove the neighbor entry.  We cannot reuse the address, because we could only
                            // reuse the last address, and another node might have already received that address.
                            RemoveNeighborTableEntry();
                        }
                        return NO_PRIMITIVE;
                    }
                    else
                    {
                        // Notify the upper layers that a new node has joined.
                        // NOTE - since the node wasn't on the network yet, we know the destination
                        // destination address is the 64-bit address, not the 16-bit address.
                        if (i != INVALID_NEIGHBOR_KEY)
                        {
                            // ExtendedAddress already in place
                            params.NLME_JOIN_indication.ShortAddress = currentNeighborRecord.shortAddr;
                            params.NLME_JOIN_indication.CapabilityInformation = nwkStatus.lastCapabilityInformation;
							#ifdef I_SUPPORT_SECURITY
                            	params.NLME_JOIN_indication.secureJoin = currentNeighborRecord.bSecured;
                        		#ifndef I_SUPPORT_SECURITY_SPEC
                            		if( !currentNeighborRecord.bSecured )
                            		{
                                		currentNeighborRecord.bSecured = TRUE;
                                		#ifdef USE_EXTERNAL_NVM
                                    		PutNeighborRecord( neighborTable + (WORD)i * (WORD)sizeof(NEIGHBOR_RECORD), &currentNeighborRecord);
                                		#else
                                    		PutNeighborRecord( &(neighborTable[i]), &currentNeighborRecord );
                                		#endif
                            		}
                        		#endif
							#else
                            	params.NLME_JOIN_indication.secureJoin = FALSE; // TODO set TRUE if MAC security was on during join
							#endif
                            return NLME_JOIN_indication;
                        }
                    }
                #endif
                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive. We just set the values directly
//            case MLME_SET_confirm:
//                break;

            // ---------------------------------------------------------------------
            case MLME_START_confirm:
                #if defined (I_AM_COORDINATOR)
                    // NOTE: ZigBeeStatus.flags.bits.bNetworkFormed = 1;
                    // should be set in the application code upon receipt of this
                    // primitive to avoid duplication.

                    // Destroy the network discovery information.
                    if (nwkStatus.discoveryInfo.channelList)
                    {
                        free( nwkStatus.discoveryInfo.channelList );
                    }

                    // If the start was successful, enable routing capability.
                    if (!params.MLME_START_confirm.status)
                    {
                        nwkStatus.flags.bits.bCanRoute = 1;
                    }

                    // Initialize other internal variables
                    currentNeighborTableInfo.depth                  = 0;
                    currentNeighborTableInfo.cSkip.Val              = CSKIP_DEPTH_0;
                    if (!currentNeighborTableInfo.flags.bits.bChildAddressInfoValid)
                    {
                        currentNeighborTableInfo.nextEndDeviceAddr.Val  = 0x0000 + (WORD)CSKIP_DEPTH_0 * (WORD)NIB_nwkMaxRouters + (WORD)1;
                        currentNeighborTableInfo.nextRouterAddr.Val     = 0x0000 + 1;
                        currentNeighborTableInfo.numChildren            = 0;
                        currentNeighborTableInfo.numChildRouters        = 0;
                        currentNeighborTableInfo.flags.bits.bChildAddressInfoValid = 1;
                    }
                    PutNeighborTableInfo();
                    SetBeaconPayload();

                    // Status already in place
                    ZigBeeStatus.flags.bits.bTryingToFormNetwork = 0;
                    ZigBeeStatus.flags.bits.bNetworkFormed = 1;
                    return NLME_NETWORK_FORMATION_confirm;
                #elif defined(I_AM_FFD)
                    // If the start was successful, enable routing capability.
                    if (!params.MLME_START_confirm.status)
                    {
                        nwkStatus.flags.bits.bCanRoute = 1;
                    }

                    // Status already in place
                    return NLME_START_ROUTER_confirm;
                #else
                    return NO_PRIMITIVE;
                #endif
                break;

            // ---------------------------------------------------------------------
//            case MLME_SYNC_LOSS_indication:
//                This portion is for beacon networks, which are not supported.
//                if (params.MLME_SYNC_LOSS_indication.LossReason == BEACON_LOST)
//                {
//                    return NLME_SYNC_indication;
//                }

//TODO - We can also get these two, which are not detailed in the ZigBee spec
//PAN_ID_CONFLICT
//REALIGNMENT
//                return NO_PRIMITIVE;
//                break;

            // ---------------------------------------------------------------------
            case MLME_POLL_confirm:
                // If we received SUCCESS, leave the value and don't set bDataRequestComplete -
                // we need to stay awake to receive the data that will be coming.  If we get
                // NO_DATA, return SUCCESS and set bDataRequestComplete - we've heard from
                // our parent, but there is no incoming data.  Otherwise, return SYNC_FAILURE
                // because we cannot talk to our parent.
                #ifdef I_AM_RFD
                    ZigBeeStatus.flags.bits.bRequestingData = 0;
                #endif
                if (params.MLME_POLL_confirm.status)
                {
                    #ifdef I_AM_RFD
                        ZigBeeStatus.flags.bits.bDataRequestComplete = 1;
                    #endif
                    if (params.MLME_POLL_confirm.status == NO_DATA)
                    {
                        params.NLME_SYNC_confirm.Status = SUCCESS;
                    }
                    else
                    {
                        params.NLME_SYNC_confirm.Status = NWK_SYNC_FAILURE;
                    }
                }
                return NLME_SYNC_confirm;
                break;

            // ---------------------------------------------------------------------
            case NLDE_DATA_request:
                {
                    BYTE            msduHandle;
                    #ifndef I_AM_RFD
                        BYTE        BTTIndex;
                        BYTE        routeStatus;
                    #endif

                    //if (macPIB.macPANId.Val == 0xFFFF)
                    #ifdef I_AM_COORDINATOR
                    if (!ZigBeeStatus.flags.bits.bNetworkFormed)
                    #else
                    if (!ZigBeeStatus.flags.bits.bNetworkJoined)
                    #endif
                    {
                        // We are not associated.  We can't send any messages until we are part of a network.
                        params.NLDE_DATA_confirm.NsduHandle = params.NLDE_DATA_request.NsduHandle;
                        params.NLDE_DATA_confirm.Status = NWK_INVALID_REQUEST;
                        ZigBeeUnblockTx();
                        return NLDE_DATA_confirm;
                    }

//                    NIB_nwkSecurityLevel removed as per NWK errata
//                    #ifdef NIB_STATIC_IMPLEMENTATION
//                    if (NIB_nwkSecurityLevel != 0)
//                    #else
//                    if (NIB.nwkSecurityLevel != 0)
//                    #endif

                    #ifndef I_AM_RFD
                        if ( params.NLDE_DATA_request.DstAddr.Val == 0xFFFF)
                        {
                            // Handle broadcast messages
                            if (!CreateNewBTR( &BTTIndex ))
                            {
                                params.NLDE_DATA_confirm.Status = TRANSACTION_OVERFLOW;
                                ZigBeeUnblockTx();
                                return NLDE_DATA_confirm;
                            }

                            // Indicate that the message was from our upper layers.
                            nwkStatus.BTT[BTTIndex]->btrInfo.bMessageFromUpperLayers = 1;

                            // Set the jitter time so it will expire and transmit right away.
                            nwkStatus.BTT[BTTIndex]->broadcastJitterTimer.Val += BROADCAST_JITTER;

                            // Set the true length of the entire NWK data - current header and data combined.
                            params.NLDE_DATA_request.NsduLength = (TX_BUFFER_SIZE-1-TxHeader) + TxData;

                            // Save off the NWK header information
                            nwkStatus.BTT[BTTIndex]->dataLength                   = params.NLDE_DATA_request.NsduLength;
                            nwkStatus.BTT[BTTIndex]->nwkFrameControlLSB.Val       = NWK_FRAME_DATA | (nwkProtocolVersion<<2) | (params.NLDE_DATA_request.DiscoverRoute<<6);
                            nwkStatus.BTT[BTTIndex]->nwkFrameControlMSB.Val       = (params.NLDE_DATA_request.SecurityEnable<<1);
                            nwkStatus.BTT[BTTIndex]->nwkDestinationAddress        = params.NLDE_DATA_request.DstAddr;
                            nwkStatus.BTT[BTTIndex]->nwkSourceAddress             = macPIB.macShortAddress;    //params.NLDE_DATA_request.SrcAddr;
                            nwkStatus.BTT[BTTIndex]->nwkRadius                    = params.NLDE_DATA_request.BroadcastRadius;
                            nwkStatus.BTT[BTTIndex]->nwkSequenceNumber            = params.NLDE_DATA_request.NsduHandle;

                            // Allocate space to save off the message
                            if ((nwkStatus.BTT[BTTIndex]->dataPtr = (BYTE *)SRAMalloc( params.NLDE_DATA_request.NsduLength )) == NULL)
                            {
                                // We do not have room to store this broadcast packet.  Destroy the BTR.
                                free( nwkStatus.BTT[BTTIndex] );
                                params.NLDE_DATA_confirm.NsduHandle = params.NLDE_DATA_request.NsduHandle;
                                params.NLDE_DATA_confirm.Status = TRANSACTION_OVERFLOW;
                                ZigBeeUnblockTx();
                                return NLDE_DATA_confirm;
                            }

                            // Save off the NWK payload - APS header plus APS payload
                            i = 0;
                            // Copy the APS Header
                            while (TxHeader < TX_BUFFER_SIZE-1)
                            {
                                nwkStatus.BTT[BTTIndex]->dataPtr[i++] = TxBuffer[++TxHeader];
                            }
                            // Copy the APS Payload
                            params.NLDE_DATA_request.NsduLength = 0;
                            while (TxData)
                            {
                                nwkStatus.BTT[BTTIndex]->dataPtr[i++] = TxBuffer[params.NLDE_DATA_request.NsduLength++];
                                TxData--;
                            }
                        #ifdef I_SUPPORT_SECURITY
                            nwkStatus.BTT[BTTIndex]->btrInfo.bAlreadySecured = 0x00;
                        #endif

                            // Set the network status so we can begin transmitting these messages
                            // in the background.
                            nwkStatus.flags.bits.bSendingBroadcastMessage = 1;

                            // The message is buffered, so unblock Tx.
                            ZigBeeUnblockTx();

                            return NO_PRIMITIVE;
                        }
                    #endif

                    #ifdef I_AM_RFD
                        macAddress = macPIB.macCoordShortAddress;
                    #else
                        routeStatus = GetRoutingAddress( TRUE, params.NLDE_DATA_request.DstAddr, params.NLDE_DATA_request.DiscoverRoute, &macAddress );
                    #endif
                    #ifndef I_AM_RFD
                    switch (routeStatus)
                    {
                        case ROUTE_SEND_TO_MAC_ADDRESS:
                    #endif
                            // Add this frame to the list of frames waiting confirmation.
                            // Try to find an empty slot.
                            for (i=0; (i<MAX_NWK_FRAMES) && (nwkConfirmationHandles[i].nsduHandle!=INVALID_NWK_HANDLE); i++) {}

                            // If we have no empty slots, return an error.
                            if (i == MAX_NWK_FRAMES)
                            {
                                params.NLDE_DATA_confirm.NsduHandle = params.NLDE_DATA_request.NsduHandle;
                                params.NLDE_DATA_confirm.Status = TRANSACTION_OVERFLOW;
                                ZigBeeUnblockTx();
                                return NLDE_DATA_confirm;
                            }

                            // Save the NWK handle before we overwrite it.
                            nwkConfirmationHandles[i].nsduHandle = params.NLDE_DATA_request.NsduHandle;

                            // Load up the NWK header information (backwards)
                            TxBuffer[TxHeader--] = params.NLDE_DATA_request.NsduHandle;
                            TxBuffer[TxHeader--] = params.NLDE_DATA_request.BroadcastRadius;
                            TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;    //params.NLDE_DATA_request.SrcAddr.byte.MSB;
                            TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;    //params.NLDE_DATA_request.SrcAddr.byte.LSB;
                            TxBuffer[TxHeader--] = params.NLDE_DATA_request.DstAddr.byte.MSB;
                            TxBuffer[TxHeader--] = params.NLDE_DATA_request.DstAddr.byte.LSB;
                            TxBuffer[TxHeader--] = (params.NLDE_DATA_request.SecurityEnable<<1); // Frame Control byte MSB
                            TxBuffer[TxHeader--] = NWK_FRAME_DATA | (nwkProtocolVersion<<2) | (params.NLDE_DATA_request.DiscoverRoute<<6); // Frame Control byte LSB

                            // Load up the MCPS_DATA.request parameters
                            Prepare_MCPS_DATA_request( macAddress.Val, &msduHandle );
                            nwkConfirmationHandles[i].msduHandle = msduHandle;
                            return MCPS_DATA_request;

                    #ifndef I_AM_RFD
                            break;

                        case ROUTE_MESSAGE_BUFFERED:
                            // The Route Request is buffered for later transmission
                            return NO_PRIMITIVE;
                            break;

                        default:
                            // Cannot route or start a route request
                            params.NLDE_DATA_confirm.NsduHandle = params.NLDE_DATA_request.NsduHandle;
                            params.NLDE_DATA_confirm.Status = NWK_CANNOT_ROUTE;
                            ZigBeeUnblockTx();
                            return NLDE_DATA_confirm;
                    }
                    #endif
                }
                break;

            // ---------------------------------------------------------------------
            case NLME_NETWORK_DISCOVERY_request:
                #ifdef I_AM_COORDINATOR
                    ZigBeeStatus.flags.bits.bTryingToFormNetwork = 0;
                    params.NLME_NETWORK_FORMATION_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_NETWORK_FORMATION_confirm;
                #else
                    if (currentNeighborTableInfo.parentNeighborTableIndex != INVALID_NEIGHBOR_KEY)
                    {
                        // An orphan scan failed.  Clear our old parent relationship.
                        #ifdef USE_EXTERNAL_NVM
                            pCurrentNeighborRecord = neighborTable + (WORD)currentNeighborTableInfo.parentNeighborTableIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                        #else
                            pCurrentNeighborRecord = &(neighborTable[currentNeighborTableInfo.parentNeighborTableIndex]);
                        #endif
                        GetNeighborRecord(&currentNeighborRecord, pCurrentNeighborRecord );
                        currentNeighborRecord.deviceInfo.bits.Relationship = NEIGHBOR_IS_NONE;
                        PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );

                        currentNeighborTableInfo.parentNeighborTableIndex = INVALID_NEIGHBOR_KEY;
                        PutNeighborTableInfo();

                        // If we try to join again, it will have to be as a new node.
                        ZigBeeStatus.flags.bits.bTryOrphanJoin = 0;
                    }

                    // Reset all the current neighbor table entries
                    for (i=0; i < MAX_NEIGHBORS; i++)
                    {
                        #ifdef USE_EXTERNAL_NVM
                            pCurrentNeighborRecord = neighborTable + (WORD)i * (WORD)sizeof(NEIGHBOR_RECORD);
                        #else
                            pCurrentNeighborRecord = &(neighborTable[i]);
                        #endif
                        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
                        if (currentNeighborRecord.deviceInfo.bits.bInUse)
                        {
                            currentNeighborRecord.deviceInfo.bits.PotentialParent = 0;
                            PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );
                        }
                    }

                    ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 1;
                    if (nwkStatus.discoveryInfo.channelList)
                    {
                        free( nwkStatus.discoveryInfo.channelList );
                    }

                    // Save off ScanChannels and ScanDuration - we'll need them again.
                    nwkStatus.lastScanChannels = params.NLME_NETWORK_DISCOVERY_request.ScanChannels;
                    nwkStatus.lastScanDuration = params.NLME_NETWORK_DISCOVERY_request.ScanDuration;

                    params.MLME_SCAN_request.ScanType = MAC_SCAN_ACTIVE_SCAN;
                    // ScanChannels is already in place
                    // ScanDuration is already in place
                    return MLME_SCAN_request;
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_NETWORK_FORMATION_request:
                #ifndef I_AM_COORDINATOR
                    params.NLME_NETWORK_FORMATION_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_NETWORK_FORMATION_confirm;
                #else
                    ZigBeeStatus.flags.bits.bTryingToFormNetwork = 1;

                    if (nwkStatus.discoveryInfo.channelList)
                    {
                        free( nwkStatus.discoveryInfo.channelList );
                    }

                    // Save off ScanChannels and ScanDuration - we'll need them again.
                    nwkStatus.lastScanChannels.Val = params.NLME_NETWORK_FORMATION_request.ScanChannels.Val;
                    nwkStatus.lastScanDuration = params.NLME_NETWORK_FORMATION_request.ScanDuration;
                    nwkStatus.requestedPANId = params.NLME_NETWORK_FORMATION_request.PANId;

                    params.MLME_SCAN_request.ScanType = MAC_SCAN_ENERGY_DETECT;
                    // ScanChannels is already in place
                    // ScanDuration is already in place
                    return MLME_SCAN_request;
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_PERMIT_JOINING_request:
                #if defined( I_AM_COORDINATOR ) || defined( I_AM_ROUTER )
                    // Disable any join time countdown that was in progress.
                    nwkStatus.flags.bits.bTimingJoinPermitDuration = 0;

                    if (params.NLME_PERMIT_JOINING_request.PermitDuration == 0x00)
                    {
                        macPIB.macAssociationPermit = FALSE;
                    }
                    else
                    {
                        macPIB.macAssociationPermit = TRUE;
                        if (params.NLME_PERMIT_JOINING_request.PermitDuration != 0xFF)
                        {
                            // Set up the join time countdown.
                            nwkStatus.joinPermitDuration = params.NLME_PERMIT_JOINING_request.PermitDuration;
                            nwkStatus.joinDurationStart = TickGet();
                            nwkStatus.flags.bits.bTimingJoinPermitDuration = 1;
                        }
                    }
                    SetBeaconPayload();
                    params.NLME_PERMIT_JOINING_confirm.Status = NWK_SUCCESS;
                    return NLME_PERMIT_JOINING_confirm;
                #else
                    params.NLME_PERMIT_JOINING_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_PERMIT_JOINING_confirm;
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_START_ROUTER_request:
                // TODO The spec implies that a ZigBee Coordinator can issue this primitive,
                // but it doesn't make sense - especially where it's supposed to pull info
                // from its parent's neighbor table entry.  So we'll just let FFD routers
                // and end devices do it for now.
                #if defined(I_AM_RFD) || defined(I_AM_COORDINATOR)
                    params.NLME_START_ROUTER_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_START_ROUTER_confirm;
                #else
                    // If we are not associated with a network, we cannot do this.
                    //if (macPIB.macPANId.Val = 0xFFFF)
                    #ifdef I_AM_COORDINATOR
                    if (!ZigBeeStatus.flags.bits.bNetworkFormed)
                    #else
                    if (!ZigBeeStatus.flags.bits.bNetworkJoined)
                    #endif
                    {
                        params.NLME_START_ROUTER_confirm.Status = NWK_INVALID_REQUEST;
                        return NLME_START_ROUTER_confirm;
                    }

                    i = (BYTE)params.NLME_START_ROUTER_request.BatteryLifeExtension;

                    // Set PANID, Beacon Order, and Superframe to my parent's values.
                    // Since we only support nonbeacon networks, we'll use the constants for
                    // Beacon Order and Superframe Order.
                    #ifdef USE_EXTERNAL_NVM
                        pCurrentNeighborRecord = neighborTable + (WORD)currentNeighborTableInfo.parentNeighborTableIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                    #else
                        pCurrentNeighborRecord = &neighborTable[currentNeighborTableInfo.parentNeighborTableIndex];
                    #endif
                    GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

                    // Clear the ability to route frames.
                    nwkStatus.flags.bits.bCanRoute = 0;

                    params.MLME_START_request.PANId = currentNeighborRecord.panID;
                    params.MLME_START_request.LogicalChannel = currentNeighborRecord.LogicalChannel;
                    params.MLME_START_request.BeaconOrder = MAC_PIB_macBeaconOrder;
                    params.MLME_START_request.SuperframeOrder = MAC_PIB_macSuperframeOrder;
                    params.MLME_START_request.fields.Val = 0; // PANCoordinator and CoordRealignment = FALSE
                    if (i)
                    {
                        params.MLME_START_request.fields.bits.BatteryLifeExtension = 1;
                    }
                    // TODO how to set params.MLME_START_request.fields.bits.SecurityEnable

                    return MLME_START_request;
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_JOIN_request:
                #ifdef I_AM_COORDINATOR
                    params.NLME_JOIN_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_JOIN_confirm;
                #else
                    //if (macPIB.macPANId.Val != 0xFFFF)
                    if (ZigBeeStatus.flags.bits.bNetworkJoined)
                    {
                        params.NLME_JOIN_confirm.Status = NWK_INVALID_REQUEST;
                        return NLME_JOIN_confirm;
                    }
                    else
                    {
                        ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 1;

                        if (!params.NLME_JOIN_request.RejoinNetwork)
                        {
                            BYTE    parentDepth;

                            // We are trying to join as a new node.  Make sure our address allocation
                            // information will be updated.  NOTE - the application is responsible for
                            // making sure all the child nodes have been kicked off!
                            #ifndef I_AM_END_DEVICE
                                currentNeighborTableInfo.flags.bits.bChildAddressInfoValid = 0;
                                PutNeighborTableInfo();
                            #endif

                            // Set the status here in case we cannot even try to associate.  If we
                            // try and fail, we'll come back to TryToJoinPotentialParent.  If we fail
                            // from that, we need to return the MLME_ASSOCIATE_confirm status, which
                            // will be already in place.
                            params.NLME_JOIN_confirm.Status = NWK_NOT_PERMITTED;

                            // Look for a parent on the specified PAN that is allowing joins and has a good link quality.
                            // If there are more than one, choose the lowest depth.
TryToJoinPotentialParent:
                            currentNeighborTableInfo.parentNeighborTableIndex = MAX_NEIGHBORS;
                            parentDepth = 0xFF;
                            for (i=0; i < MAX_NEIGHBORS; i++)
                            {
                                #ifdef USE_EXTERNAL_NVM
                                    pCurrentNeighborRecord = neighborTable + (WORD)i * (WORD)sizeof(NEIGHBOR_RECORD);
                                #else
                                    pCurrentNeighborRecord = &(neighborTable[i]);
                                #endif
                                GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

                                if (currentNeighborRecord.deviceInfo.bits.bInUse &&
                                    currentNeighborRecord.deviceInfo.bits.PermitJoining &&
                                    currentNeighborRecord.deviceInfo.bits.PotentialParent &&
                                    (currentNeighborRecord.panID.Val == params.NLME_JOIN_request.PANId.Val) &&
                                    (currentNeighborRecord.deviceInfo.bits.LQI >= MINIMUM_JOIN_LQI))
                                {
                                    if (parentDepth > currentNeighborRecord.deviceInfo.bits.Depth)
                                    {
                                        parentDepth = currentNeighborRecord.deviceInfo.bits.Depth;
                                        currentNeighborTableInfo.parentNeighborTableIndex = i;
                                    }
                                }
                            }
                            if (currentNeighborTableInfo.parentNeighborTableIndex != MAX_NEIGHBORS)
                            {
                                #ifdef USE_EXTERNAL_NVM
                                    pCurrentNeighborRecord = neighborTable + (WORD)currentNeighborTableInfo.parentNeighborTableIndex * (WORD)sizeof(NEIGHBOR_RECORD);
                                #else
                                    pCurrentNeighborRecord = &(neighborTable[currentNeighborTableInfo.parentNeighborTableIndex]);
                                #endif
                                GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

                                // Set the PAN ID to the PAN that we are trying to join.  If this fails, we'll have to clear it.
                                macPIB.macPANId = params.NLME_JOIN_request.PANId;
                                MLME_SET_macPANId_hw();

                                // CoordPANId already in place
                                // SecurityEnable already in place
                                ZDOGetCapabilityInfo( &i );
                                params.MLME_ASSOCIATE_request.LogicalChannel = currentNeighborRecord.LogicalChannel;
							#ifdef ZCP_DEBUG
                                if( currentNeighborRecord.shortAddr.Val == 0xfffe)
                                {
                                    macPIB.macCoordShortAddress.Val = 0xfffe;
                                    params.MLME_ASSOCIATE_request.CoordAddrMode = 0x03;
                                    params.MLME_ASSOCIATE_request.CoordAddress.LongAddr = currentNeighborRecord.longAddr;
                                }  else
							#endif
                                {
                                    params.MLME_ASSOCIATE_request.CoordAddrMode = 0x02; // short address
                                    params.MLME_ASSOCIATE_request.CoordAddress.ShortAddr = currentNeighborRecord.shortAddr;
                                }
                                params.MLME_ASSOCIATE_request.CapabilityInformation.Val = i;
								#ifdef I_SUPPORT_SECURITY_SPEC
                               {
                                    BYTE ActiveKeyIndex;
                                    GetNwkActiveKeyNumber(&ActiveKeyIndex);

                                    if( ActiveKeyIndex == 1 || ActiveKeyIndex == 2 )
                                    {
                                        params.MLME_ASSOCIATE_request.SecurityEnable = TRUE;
                                    } else {
                                        params.MLME_ASSOCIATE_request.SecurityEnable = FALSE;
                                    }
                                }
								#endif

                                return MLME_ASSOCIATE_request;
                            }
                            else
                            {
                                ZigBeeStatus.flags.bits.bTryingToJoinNetwork = 0;
                                // Status either set above or left over from MLME_ASSOCIATE_confirm
                                return NLME_JOIN_confirm;
                            }
                        }
                        else
                        {
                            // Scan Channels and Scan Duration are already in place.
                            params.MLME_SCAN_request.ScanType = MAC_SCAN_ORPHAN_SCAN;
                            return MLME_SCAN_request;
                        }
                    }
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_DIRECT_JOIN_request:
                #ifdef I_AM_END_DEVICE
                    params.NLME_DIRECT_JOIN_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_DIRECT_JOIN_confirm;
                #else
                    if (NWKLookupNodeByLongAddr( &(params.NLME_DIRECT_JOIN_request.DeviceAddress) ) != INVALID_NEIGHBOR_KEY)
                    {
                        // The node is already in our neighbor table.
                        params.NLME_DIRECT_JOIN_confirm.Status = NWK_ALREADY_PRESENT;
                        return NLME_DIRECT_JOIN_confirm;
                    }
                    if (!CanAddChildNode() || ((i = CanAddNeighborNode()) == INVALID_NEIGHBOR_KEY))
                    {
                        // We do not have room to add this node to the neighbor table.
                        params.NLME_DIRECT_JOIN_confirm.Status = NWK_TABLE_FULL;
                        return NLME_DIRECT_JOIN_confirm;
                    }
                    else
                    {
                        // Add the node to our network
						#ifdef I_SUPPORT_SECURITY
                        	AddChildNode(TRUE);
						#else
                        	AddChildNode();
						#endif
                        params.NLME_DIRECT_JOIN_confirm.Status = NWK_SUCCESS;
                        return NLME_DIRECT_JOIN_confirm;
                    }
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_LEAVE_request:
                // If we are not associated with a network, then we cannot leave it.
                //if (macPIB.macPANId.Val == 0xFFFF)
                #ifdef I_AM_COORDINATOR
                if (!ZigBeeStatus.flags.bits.bNetworkFormed)
                #else
                if (!ZigBeeStatus.flags.bits.bNetworkJoined)
                #endif
                {
                    params.NLME_LEAVE_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_LEAVE_confirm;
                }

                // See if we are being told to leave the network.
                if (NWKThisIsMyLongAddress( &(params.NLME_LEAVE_request.DeviceAddress) ))
                {
                    // My upper layers have told me to leave the network.
                    // Do not allow anyone else to join.
                    macPIB.macAssociationPermit = FALSE;

                    nwkStatus.flags.bits.bLeavingTheNetwork = 1;
                    #ifndef I_AM_END_DEVICE
                        nwkStatus.flags.bits.bRemovingChildren = params.NLME_LEAVE_request.RemoveChildren;
                        nwkStatus.flags.bits.bAllChildrenLeft = 1;
                        nwkStatus.leaveCurrentNode = 0;
                        if (!nwkStatus.flags.bits.bRemovingChildren)
                        {
                            nwkStatus.leaveCurrentNode = INVALID_NEIGHBOR_KEY;
                        }
                        nwkStatus.leaveStartTime = TickGet();
                    #endif
                    nwkStatus.leaveReason = SELF_INITIATED_LEAVE;

                    return NO_PRIMITIVE;
                }

                #ifdef I_AM_END_DEVICE
                    params.NLME_LEAVE_confirm.Status = NWK_INVALID_REQUEST;
                    return NLME_LEAVE_confirm;
                #else
                    // The ZigBee spec only allows for a parent to remove its child.  It doesn't allow for a higher node,
                    // such as the coordinator, to remove a lower node.
                    if (((i = NWKLookupNodeByLongAddr( &(params.NLME_LEAVE_request.DeviceAddress))) != INVALID_NEIGHBOR_KEY) &&
                        (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD))
                    {
                        TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();
                        TxBuffer[TxHeader--] = 1;   // radius of 1, as per errata
                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB;
                        TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB;
                        TxBuffer[TxHeader--] = currentNeighborRecord.shortAddr.byte.MSB;
                        TxBuffer[TxHeader--] = currentNeighborRecord.shortAddr.byte.LSB;
						#ifdef I_SUPPORT_SECURITY
                        	if( currentNeighborRecord.bSecured )
                        	{
                            	TxBuffer[TxHeader--] = 0x02;
                        	}
                        	else
						#else
                            TxBuffer[TxHeader--] = 0; // nwkFrameControlMSB TODO check security setting for this
						#endif
                        TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

                        // Load the NWK payload into the transmit buffer.
                        TxBuffer[TxData++] = NWK_COMMAND_LEAVE;
                        i = NWK_LEAVE_REQUEST;
                        if (params.NLME_LEAVE_request.RemoveChildren)
                        {
                            i |= NWK_LEAVE_REMOVE_CHILDREN;
                        }
                        TxBuffer[TxData++] = i;

                        Prepare_MCPS_DATA_request( currentNeighborRecord.shortAddr.Val, &i );
                        return MCPS_DATA_request;
                    }
                    else
                    {
                        params.NLME_LEAVE_confirm.Status = NWK_UNKNOWN_DEVICE;
                        return NLME_LEAVE_confirm;
                    }
                #endif
                break;

            // ---------------------------------------------------------------------
            case NLME_RESET_request:
                params.MLME_RESET_request.SetDefaultPIB = TRUE;
                return MLME_RESET_request;
                break;

            // ---------------------------------------------------------------------
            case NLME_SYNC_request:
                // We only support non-beacon networks, so some of the functionality
                // of this primitive has been omitted.
                if (!params.NLME_SYNC_request.Track)
                {
                    {
                        params.MLME_POLL_request.SecurityEnabled = FALSE;
                    }

                    #ifdef I_AM_RFD
                        ZigBeeStatus.flags.bits.bRequestingData = 1;
                        ZigBeeStatus.flags.bits.bDataRequestComplete = 0;
                    #endif

                    //TODO set this up params.MLME_POLL_request.SecurityEnabled = ???;
                    params.MLME_POLL_request.CoordPANId             = macPIB.macPANId;
                #ifdef ZCP_DEBUG
                    if( macPIB.macCoordShortAddress.Val == 0xfffe )
                    {
                        params.MLME_POLL_request.CoordAddrMode = 0x03;
                        params.MLME_POLL_request.CoordAddress.LongAddr = macPIB.macCoordExtendedAddress;
                    } else
                #endif
                    {
                        params.MLME_POLL_request.CoordAddrMode          = 0x02;
                        params.MLME_POLL_request.CoordAddress.ShortAddr = macPIB.macCoordShortAddress;
                    }
                    return MLME_POLL_request;
                }
                else
                {
                    #ifdef I_AM_RFD
                        ZigBeeStatus.flags.bits.bRequestingData = 0;
                        ZigBeeStatus.flags.bits.bDataRequestComplete = 1;
                    #endif

                    params.NLME_SYNC_confirm.Status = NWK_INVALID_PARAMETER;
                    return NLME_SYNC_confirm;
                }
                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive. We just access the values directly.
//            case NLME_GET_request:
//                break;

            // ---------------------------------------------------------------------
//            We don't support this primitive. We just access the values directly.
//            case NLME_SET_request:
//                break;
        }
    }
    return NO_PRIMITIVE;
}


/*********************************************************************
 * Function:        void AddChildNode( void )
 *
 * PreCondition:    The node must not be already in the neighbor table,
 *                  and pCurrentNeighborRecord must point to a free location.
 *                  This routine can be called from either
 *                  MLME_ASSOCIATE_indication or NLME_DIRECT_JOIN_request.
 *
 * Input:           index       - handle to a free location
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Creates new child entry for given node at the input
 *                  location.
 *
 * Note:            MLME_ASSOCIATE_indication.CapabilityInformation must
 *                  overlay NLME_DIRECT_JOIN_request.CapabilityInformation,
 *                  and MLME_ASSOCIATE_indication.DeviceAddress must overlay
 *                  NLME_DIRECT_JOIN_request.DeviceAddress, since this
 *                  routine can be called from either function.
 ********************************************************************/
#ifndef I_AM_END_DEVICE
#ifdef I_SUPPORT_SECURITY
void AddChildNode(BOOL bSecured)
#else
void AddChildNode( void  )
#endif
{
    SHORT_ADDR                  shortAddr;
    BYTE i;

    memcpy( (void*)&currentNeighborRecord.longAddr, (void*)&(params.MLME_ASSOCIATE_indication.DeviceAddress), (size_t)(sizeof(LONG_ADDR)));
    currentNeighborRecord.panID = macPIB.macPANId;
    currentNeighborRecord.LogicalChannel = 0xFF;
	#ifdef I_SUPPORT_SECURITY
		#ifdef I_SUPPORT_SECURITY_SPEC
    		currentNeighborRecord.bSecured = bSecured;
		#else
    		#ifdef PRECONFIGURE_KEY
        		currentNeighborRecord.bSecured = TRUE;
    		#else
        		currentNeighborRecord.bSecured = FALSE;
    		#endif
		#endif
	#endif

    if (params.MLME_ASSOCIATE_indication.CapabilityInformation.CapBits.DeviceType == DEVICE_IEEE_RFD)
    {
        currentNeighborRecord.deviceInfo.bits.deviceType = DEVICE_ZIGBEE_END_DEVICE;
	#ifdef ZCP_DEBUG
        if( bDisableShortAddress )
            currentNeighborRecord.shortAddr.Val = 0xfffe;
        else
	#endif
        currentNeighborRecord.shortAddr = currentNeighborTableInfo.nextEndDeviceAddr;
        currentNeighborTableInfo.nextEndDeviceAddr.Val ++;
        if (currentNeighborTableInfo.nextEndDeviceAddr.Val == LAST_END_DEVICE_ADDRESS)
        {
            nwkStatus.flags.bits.bAllEndDeviceAddressesUsed = 1;
        }
    }
    else
    {
        currentNeighborRecord.deviceInfo.bits.deviceType = DEVICE_ZIGBEE_ROUTER;
	#ifdef ZCP_DEBUG
        if( bDisableShortAddress )
            currentNeighborRecord.shortAddr.Val = 0xfffe;
        else
	#endif
        currentNeighborRecord.shortAddr = currentNeighborTableInfo.nextRouterAddr;
        currentNeighborTableInfo.numChildRouters++;
        currentNeighborTableInfo.nextRouterAddr.Val += currentNeighborTableInfo.cSkip.Val;
        if (currentNeighborTableInfo.nextRouterAddr.Val > macPIB.macShortAddress.Val + currentNeighborTableInfo.cSkip.Val * NIB_nwkMaxRouters)
        {
            nwkStatus.flags.bits.bAllRouterAddressesUsed = 1;
        }
    }
    currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle  = params.MLME_ASSOCIATE_indication.CapabilityInformation.CapBits.RxOnWhenIdle;
    currentNeighborRecord.deviceInfo.bits.Relationship  = NEIGHBOR_IS_CHILD;
    currentNeighborRecord.deviceInfo.bits.bInUse        = TRUE;

    PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord);

    currentNeighborTableInfo.numChildren++;
    currentNeighborTableInfo.neighborTableSize++;
    PutNeighborTableInfo();

    SetBeaconPayload();
}
#endif

/*********************************************************************
 * Function:        BOOL CanAddChildNode ( void )
 *
 * PreCondition:    This routine must be called from either the
 *                  MLME_ASSOCIATE_indication or NLME_DIRECT_JOIN_request
 *                  primitive.
 *
 * Input:           None
 *
 * Output:          If we have the capacity to add the node as our child.
 *
 * Side Effects:    None
 *
 * Overview:        This routine determines if there is capacity to add
 *                  a child node of the given type.  First, macAssociationPermit
 *                  must be TRUE.  Then, we must not have exceeded our
 *                  maximum number of children.  If the device is an
 *                  RFD, we must not have run out of RFD addresses (since
 *                  we cannot reuse them).  If the device is an FFD, then
 *                  there must be depth available for the FFD to add its
 *                  own children, we must not have exceeded our maximum
 *                  number of routers, and we must not have run out of
 *                  FFD addresses.
 *
 * Note:            MLME_ASSOCIATE_indication.CapabilityInformation must
 *                  overlay NLME_DIRECT_JOIN_request.CapabilityInformation,
 *                  and MLME_ASSOCIATE_indication.DeviceAddress must overlay
 *                  NLME_DIRECT_JOIN_request.DeviceAddress, since this
 *                  routine can be called from either function.
 ********************************************************************/
#ifndef I_AM_END_DEVICE
BOOL CanAddChildNode( void )
{
    return (macPIB.macAssociationPermit &&
            (currentNeighborTableInfo.numChildren < NIB_nwkMaxChildren) &&
            (((params.MLME_ASSOCIATE_indication.CapabilityInformation.CapBits.DeviceType == DEVICE_IEEE_RFD) &&
              (!nwkStatus.flags.bits.bAllEndDeviceAddressesUsed)) ||
             ((currentNeighborTableInfo.depth < (NIB_nwkMaxDepth-1)) &&
              (currentNeighborTableInfo.numChildRouters < NIB_nwkMaxRouters) &&
              (!nwkStatus.flags.bits.bAllRouterAddressesUsed))));
}
#endif

/*********************************************************************
 * Function:        NEIGHBOR_KEY CanAddNeighborNode( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Handle to available space
 *
 * Side Effects:    None
 *
 * Overview:        This routine determines if there is a free space
 *                  in the neighbor table and returns a handle to it.
 *
 * Note:            If the function is successful, pCurrentNeighborRecord
 *                  points to the available location.
 ********************************************************************/
NEIGHBOR_KEY CanAddNeighborNode( void )
{
    BYTE    index;
    BOOL    spaceFound;

    // Get ROM address of neighborTable in RAM.
    pCurrentNeighborRecord = neighborTable;

    #ifdef USE_EXTERNAL_NVM
    for (index = 0, spaceFound = FALSE; (index < MAX_NEIGHBORS) && !spaceFound; pCurrentNeighborRecord+=(WORD)sizeof(NEIGHBOR_RECORD), index++)
    #else
    for (index = 0, spaceFound = FALSE; (index < MAX_NEIGHBORS) && !spaceFound; pCurrentNeighborRecord++, index++)
    #endif
    {
        // Read the record into RAM.
        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

        // Now check to see if it is in use or not.
        if ( !currentNeighborRecord.deviceInfo.bits.bInUse )
        {
            spaceFound = TRUE;
            break;
        }
    }

    if ( spaceFound )
    {
        return index;
    }
    else
    {
        return INVALID_NEIGHBOR_KEY;
    }
}

/*********************************************************************
 * Function:        BOOL CreateNewBTR( BYTE *BTTIndex )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - new BTR created at indext BTT
 *                  FALSE - could not create a new BTR
 *
 * Side Effects:    None
 *
 * Overview:        This function tries to create a new BTR.  If if
 *                  cannot because there is no room in the BTT or it
 *                  cannot allocate enough memory, it returns FALSE.
 *                  Otherwise, it returns TRUE, and BTTIndex indicates
 *                  the position of the new BTR.  The flags of the BTR
 *                  are initialized and currentNeighbor is set to 0,
 *                  but no other data is populated.
 *
 * Note:            bMessageFromUpperLayers is set to TRUE.
 ********************************************************************/
#ifndef I_AM_RFD
BOOL CreateNewBTR( BYTE *BTTIndex )
{
    BYTE    i;
    BYTE    j;

    // Find an empty place in the BTT.
    for (i=0; (i<NUM_BUFFERED_BROADCAST_MESSAGES) && (nwkStatus.BTT[i]!=NULL); i++) {}
    if (i==NUM_BUFFERED_BROADCAST_MESSAGES)
    {
        return FALSE;
    }

    // Try to allocate space for a new BTR.
    if ((nwkStatus.BTT[i] = (BTR *)SRAMalloc(sizeof(BTR))) == NULL)
    {
        return FALSE;
    }

    // Set the BTR index to the new entry.
    *BTTIndex = i;

    // Initialize the BTR the best we can.  Set the broadcast time so it
    // won't time out before send it.
    nwkStatus.BTT[i]->currentNeighbor = 0xFF;
    nwkStatus.BTT[i]->broadcastTime = nwkStatus.BTT[i]->broadcastJitterTimer = TickGet();
    nwkStatus.BTT[i]->btrInfo.bConfirmSent = 0;

    #ifdef NIB_STATIC_IMPLEMENTATION
    nwkStatus.BTT[i]->btrInfo.nRetries         = NIB_nwkMaxBroadcastRetries;
    #else
    nwkStatus.BTT[i]->btrInfo.nRetries         = NIB.nwkMaxBroadcastRetries;
    #endif
    for (j=0; j<MAX_NEIGHBORS; j++)
    {
        nwkStatus.BTT[i]->flags[j].byte = BTR_FLAGS_INIT;
    }
    #ifdef I_SUPPORT_SECURITY
    nwkStatus.BTT[i]->btrInfo.bAlreadySecured = 0x00;
    #endif
    return TRUE;
}
#endif

/*********************************************************************
 * Function:        void CreateRouteReply( SHORT_ADDR originatorAddress,
 *                      BYTE rdIndex, ROUTE_REQUEST_COMMAND *rreq )
 *
 * PreCondition:    Must be called from MCPS_DATA_indication
 *
 * Input:           nwkSourceAddress - NWK source address of message
 *                  rdIndex - index into Route Discovery Table
 *                  rreq - pointer to route request command packet
 *
 * Side Effects:    None
 *
 * Overview:        Sends the Route Reply command frame to the sender
 *                  of the previous message.
 *
 * Note:            None
 ********************************************************************/
#ifdef I_SUPPORT_ROUTING
void CreateRouteReply( SHORT_ADDR nwkSourceAddress, BYTE rdIndex, ROUTE_REQUEST_COMMAND *rreq )
{
    BYTE                temp;

#ifndef USE_TREE_ROUTING_ONLY
    if (rdIndex != INVALID_ROUTE_DISCOVERY_INDEX)
    {
        // Mark the routing table entry as valid for use.
        #ifdef USE_EXTERNAL_NVM
            pCurrentRoutingEntry = routingTable + (WORD)(routeDiscoveryTablePointer[rdIndex]->routingTableIndex) * (WORD)sizeof(ROUTING_ENTRY);
        #else
            pCurrentRoutingEntry = &(routingTable[routeDiscoveryTablePointer[rdIndex]->routingTableIndex]);
        #endif
        GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
        currentRoutingEntry.status = ROUTE_ACTIVE;
        PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );

        // Do not free the route discovery table entry.  We might get a better
        // route within the allowed time.  We'll let the clean-up routine
        // free the route discovery table entry.
    }
#endif

    // Load up the NWK payload.
    TxBuffer[TxData++] = NWK_COMMAND_ROUTE_REPLY;               // rrep.commandFrameIdentifier
    TxBuffer[TxData++] = rreq->commandOptions;              // rrep.commandOptions
    TxBuffer[TxData++] = rreq->routeRequestIdentifier;      // rrep.routeRequestIdentifier
    TxBuffer[TxData++] = nwkSourceAddress.byte.LSB;         // rrep.originatorAddress
    TxBuffer[TxData++] = nwkSourceAddress.byte.MSB;         // rrep.originatorAddress
    TxBuffer[TxData++] = rreq->destinationAddress.byte.LSB; // rrep.responderAddress
    TxBuffer[TxData++] = rreq->destinationAddress.byte.MSB; // rrep.responderAddress
    TxBuffer[TxData++] = rreq->pathCost;                    // rrep.pathCost

    // Load up the NWK header information (backwards)
    TxBuffer[TxHeader--] = NLME_GET_nwkBCSN();                          // sequence number
    TxBuffer[TxHeader--] = DEFAULT_RADIUS;                              // radius
    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.MSB; //  rreq->destinationAddress.byte.MSB;           // source address
    TxBuffer[TxHeader--] = macPIB.macShortAddress.byte.LSB; //  rreq->destinationAddress.byte.LSB;
    TxBuffer[TxHeader--] = nwkSourceAddress.byte.MSB;                   // destination address
    TxBuffer[TxHeader--] = nwkSourceAddress.byte.LSB;
	#ifdef I_SUPPORT_SECURITY
    	TxBuffer[TxHeader--] = 0x02;
	#else
    	TxBuffer[TxHeader--] = 0;       // TODO how to handle security?     // Frame Control byte MSB
	#endif
    TxBuffer[TxHeader--] = NWK_FRAME_CMD | (nwkProtocolVersion<<2);     // Frame Control byte LSB

    // Load up the MCPS_DATA.request parameters
    // We'll just send it back to where it came from, since rdIndex may be invalid.
    Prepare_MCPS_DATA_request( params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val, &temp );
}
#endif

/*********************************************************************
 * Function:        BOOL CreateRoutingTableEntries( SHORT_ADDR targetAddr,
 *                      BYTE *rdIndex, BYTE *rtIndex )
 *
 * PreCondition:    HasRoutingCapacity must be TRUE
 *
 * Input:           None
 *
 * Output:          *rdIndex - Route Discovery Table index of new entry
 *                  *rtIndex - Routing Table index of new entry
 *                  TRUE if entries created successfully.
 *
 * Side Effects:    None
 *
 * Overview:        This routine creates Route Discovery Table and
 *                  Routing Table entries.
 *
 * Note:            None
 ********************************************************************/
#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
BOOL CreateRoutingTableEntries( SHORT_ADDR targetAddr, BYTE *rdIndex, BYTE *rtIndex )
{
    BYTE                temp;

    // See if an entry already exists for the target address
    // in the routing table.  Otherwise, find a free location in the
    // Routing Table.  We know one or the other is possible since
    // HasRoutingCapacity was true.  We need to see if the destination
    // address is already in the table in case the route is being
    // re-discovered.  Since we have a local variable, use it instead
    // of the pointer.
    temp = 0;
    pCurrentRoutingEntry = routingTable;
    GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
    while ((temp < ROUTING_TABLE_SIZE) &&
           (currentRoutingEntry.destAddress.Val != targetAddr.Val))
    {
        temp++;
        #ifdef USE_EXTERNAL_NVM
            pCurrentRoutingEntry += (WORD)sizeof(ROUTING_ENTRY);
        #else
            pCurrentRoutingEntry++;
        #endif
        GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
    }
    if (temp == ROUTING_TABLE_SIZE)
    {
        temp = 0;
        pCurrentRoutingEntry = routingTable;
        GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
        while ((temp < ROUTING_TABLE_SIZE) &&
               (currentRoutingEntry.status != ROUTE_INACTIVE))
        {
            temp++;
            #ifdef USE_EXTERNAL_NVM
                pCurrentRoutingEntry += (WORD)sizeof(ROUTING_ENTRY);
            #else
                pCurrentRoutingEntry++;
            #endif
            GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
        }
    }
    *rtIndex = temp;


    // Find a free location in the Route Discovery Table.  We have to use a
    // temporary variable - the compiler cannot properly index off of *rdIndex.
    for (temp = 0;
         (temp < ROUTE_DISCOVERY_TABLE_SIZE) &&
         (routeDiscoveryTablePointer[temp] != NULL);
         temp++ ) {}
    *rdIndex = temp;

    if ((routeDiscoveryTablePointer[temp] = (ROUTE_DISCOVERY_ENTRY *)SRAMalloc( sizeof(ROUTE_DISCOVERY_ENTRY) )) == NULL)
    {
        return FALSE;
    }

    // Populate the Routing Table entry.
    currentRoutingEntry.destAddress = targetAddr;
    currentRoutingEntry.status = ROUTE_DISCOVERY_UNDERWAY;
    PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );

    // Populate what we can in the Route Discovery Table entry.
    routeDiscoveryTablePointer[temp]->residualCost = 0xff;
    routeDiscoveryTablePointer[temp]->timeStamp = TickGet();
    routeDiscoveryTablePointer[temp]->routingTableIndex = *rtIndex;
    routeDiscoveryTablePointer[temp]->forwardRREQ = NULL;

    // We just made a Route Discovery Table entry, so set the status flag
    // so we can watch it for time out.
    nwkStatus.flags.bits.bAwaitingRouteDiscovery = 1;

    return TRUE;
}
#endif

/*********************************************************************
 * Function:        WORD GetCSkipVal( BYTE depth )
 *
 * PreCondition:    None
 *
 * Input:           Depth of a node in the tree
 *
 * Output:          Cskip value for a router at the given depth
 *
 * Side Effects:    None
 *
 * Overview:        This routine determines the Cskip value for
 *                  address allocation at a given tree depth.  Since
 *                  this calculation is pretty nasty, the values are
 *                  placed in a header file as constants rather than
 *                  determining them at run time.
 *
 * Note:            If allowed tree depths are expanded, this routine
 *                  will have to be expanded as well.
 ********************************************************************/
#ifdef I_SUPPORT_ROUTING
WORD GetCSkipVal( BYTE depth )
{
    WORD    cSkip;

    switch (depth)
    {
            case 0:
                cSkip = CSKIP_DEPTH_0;
                break;
        #ifdef CSKIP_DEPTH_1
            case 1:
                cSkip = CSKIP_DEPTH_1;
                break;
        #endif
        #ifdef CSKIP_DEPTH_2
            case 2:
                cSkip = CSKIP_DEPTH_2;
                break;
        #endif
        #ifdef CSKIP_DEPTH_3
            case 3:
                cSkip = CSKIP_DEPTH_3;
                break;
        #endif
        #ifdef CSKIP_DEPTH_4
            case 4:
                cSkip = CSKIP_DEPTH_4;
                break;
        #endif
        #ifdef CSKIP_DEPTH_5
            case 5:
                cSkip = CSKIP_DEPTH_5;
                break;
        #endif
        #ifdef CSKIP_DEPTH_6
            case 6:
                cSkip = CSKIP_DEPTH_6;
                break;
        #endif
        #ifdef CSKIP_DEPTH_7
            case 7:
                cSkip = CSKIP_DEPTH_7;
                break;
        #endif
        #ifdef CSKIP_DEPTH_8
            case 8:
                cSkip = CSKIP_DEPTH_8;
                break;
        #endif
        #ifdef CSKIP_DEPTH_9
            #error CSKIP look-up table needs to be expanded.
        #endif
    }

    return cSkip;
}
#endif

/*********************************************************************
 * Function:        BOOL GetNextHop(    SHORT_ADDR destAddr,
 *                                      SHORT_ADDR *nextHop,
 *                                      BYTE *routeStatus )
 *
 * PreCondition:    None
 *
 * Input:           destAddr  - desired short address
 *                  *nextHop - pointer to next hop address
 *                  *routeStatus - pointer to route status
 *
 * Output:          TRUE if an entry found in the routing table.
 *                      currentRoutingEntry and pCurrentRoutingEntry
 *                      will be set the the Routing Table entry
 *                      that matches the request.
 *                  FALSE if no entry found.
 *
 * Side Effects:    None
 *
 * Overview:        This routine finds an entry in the routing table
 *                  for the given destination address.  Note that the
 *                  caller must check the route status to see how to
 *                  use the next hop destination.
 *
 * Note:            None
 ********************************************************************/
#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
BOOL GetNextHop( SHORT_ADDR destAddr, SHORT_ADDR *nextHop, BYTE *routeStatus )
{
    BOOL                foundNextHop;
    BYTE                i;

    foundNextHop = FALSE;
    pCurrentRoutingEntry = routingTable;
    i = 0;
    do
    {
        GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
        if (currentRoutingEntry.destAddress.Val == destAddr.Val)

        {
            foundNextHop = TRUE;
        }
        else
        {
            i++;
            #ifdef USE_EXTERNAL_NVM
                pCurrentRoutingEntry += (WORD)sizeof(ROUTING_ENTRY);
            #else
                pCurrentRoutingEntry++;
            #endif
        }
    } while ((i<ROUTING_TABLE_SIZE) && !foundNextHop );

    if (foundNextHop)
    {
        *nextHop = currentRoutingEntry.nextHop;
        *routeStatus = currentRoutingEntry.status;
    }
    else
    {
        *routeStatus = ROUTE_INACTIVE;
    }

    return foundNextHop;
}
#endif

/*******************************************************************************
 * Function:        BYTE GetRoutingAddress( BOOL fromUpperLayers, SHORT_ADDR nwkAddress,
 *                      BYTE discoverRoute, SHORT_ADDR *macAddress )
 *
 * PreCondition:    none
 *
 * Input:           fromUpperLayers - if we are currently in a NLDE_DATA_request or
 *                          MCPS_DATA_indication
 *                  nwkAddress - final destination address
 *                  discoverRoute - Route Discovery indication
 *
 * Output:          *macAddress - address of next hop to the final destination
 *                  ROUTE_FAILURE               message cannot be sent or buffered
 *                  ROUTE_MESSAGE_BUFFERED      message buffered pending route discovery
 *                  ROUTE_SEND_TO_MAC_ADDRESS   valid destination address found
 *
 * Side Effects:    None
 *
 * Overview:        This function takes the target NWK address and determines
 *                  the next hop MAC address to send the message to.  If the
 *                  destination address is not our child and we do not have a
 *                  route, this routine buffers the message and initiates
 *                  Route Discovery.  If macAddress is a valid next hop
 *                  destination, the routine returns TRUE.
 *
 * Note:            There is a hole in this algorithm.  HaveRoutingCapacity is
 *                  checked before we look for the next hop address in the routing
 *                  table.  It is possible that the route discovery table could
 *                  be full discovering other routes, which causes HaveRoutingCapacity
 *                  to fail, while a route already exists in the routing table.
 *                  The spec may change...
 ******************************************************************************/
#ifndef I_AM_RFD
MESSAGE_ROUTING_STATUS GetRoutingAddress( BOOL fromUpperLayers, SHORT_ADDR nwkAddress, BYTE discoverRoute, SHORT_ADDR *macAddress )
{
    if ((NWKLookupNodeByShortAddrVal( nwkAddress.Val ) != INVALID_NEIGHBOR_KEY) &&
             (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD))
    {
        // The destination is one of my children.
        *macAddress = nwkAddress;
        // We can send the message.
        return ROUTE_SEND_TO_MAC_ADDRESS;
    }
    else
    {
#ifndef USE_TREE_ROUTING_ONLY
        if (HaveRoutingCapacity( FALSE, 0, macPIB.macShortAddress, nwkAddress, ROUTE_REPAIR ))
        {
            // Try to route the current packet.
            if (discoverRoute == ROUTE_DISCOVERY_FORCE)
            {
                // We don't want the next hop to force a route discovery again!
                return InitiateRouteDiscovery( fromUpperLayers, ROUTE_DISCOVERY_ENABLE );
            }
            else
            {
                BYTE    routeStatus;

                if (GetNextHop( nwkAddress, macAddress, &routeStatus ))
                {
                    if (routeStatus == ROUTE_ACTIVE)
                    {
                        // We can send the message.
                        return ROUTE_SEND_TO_MAC_ADDRESS;
                    }
                    else
                    {
                        discoverRoute = ROUTE_DISCOVERY_SUPPRESS;
                        goto DoTreeRouting;
                    }
                }
                else
                {
                    if (discoverRoute == ROUTE_DISCOVERY_ENABLE)
                    {
                        return InitiateRouteDiscovery( fromUpperLayers, ROUTE_DISCOVERY_ENABLE );
                    }
                    else
                    {
                        goto DoTreeRouting;
                    }
                }
            }
        }
        else
        {
DoTreeRouting:
            if (!RouteAlongTree( nwkAddress, macAddress ))
            {
                // We cannot route the message.
                return ROUTE_FAILURE_TREE_LINK;
            }
            else
            {
                // We can send the message.
                return ROUTE_SEND_TO_MAC_ADDRESS;
            }
        }
#else
        if (!RouteAlongTree( nwkAddress, macAddress ))
        {
            // We cannot route the message.
            return ROUTE_FAILURE_TREE_LINK;
        }
        else
        {
            // Fall through and send message.
            return ROUTE_SEND_TO_MAC_ADDRESS;
        }
#endif  // USE_TREE_ROUTING_ONLY
    }
}
#endif

/*********************************************************************
 * Function:        BOOL HaveRoutingCapacity( BOOL validID, BYTE routeRequestID,
 *                      SHORT_ADDR routeSourceAddress,
 *                      SHORT_ADDR routeDestAddress,
 *                      BYTE commandOptions )
 *
 * PreCondition:    None
 *
 * Input:           validID - routeRequestID is a valid ID
 *                  routeRequestID - ID of current route request.  Valid
 *                      if routeSourceAddress is not our address
 *                  routeSourceAddress - source of the route
 *                  routeDestAddress - destination of the route
 *                  routeRepair - if this is a route repair operation
 *
 * Output:          TRUE - we have routing capacity
 *                  FALSE - we do not have routing capacity
 *
 * Side Effects:    None
 *
 * Overview:        This function determines if we have routing
 *                  capacity based on the spec definition:
 *
 *                  A device is said to have routing table capacity if:
 *                    1. It is a ZigBee coordinator or ZigBee router.
 *                    2. It maintains a routing table.
 *                    3. It has a free routing table entry or it already
 *                       has a routing table entry corresponding to the
 *                       destination.
 *                    4. The device is attempting route repair and it has
 *                       reserved some entries for this purpose as described
 *                       above.
 *                  A device is said to have “route discovery table capacity” if:
 *                    1. It maintains a route discovery table.
 *                    2. It has a free entry in its route discovery table.
 *                  If a device has both routing table capacity and route
 *                  discovery table capacity then it may be said the have
 *                  “routing capacity”.
 *
 * Note:            "Route Discovery Table Capacity" does not take into
 *                  account if we are already discovering the current
 *                  route.  This will occur often, since route discovery
 *                  is broadcast.  It has therefore been added to this
 *                  implementation.
 *
 *                  If we are in this routine as a response to upper
 *                  layers sending an NLME_DATA_request, we do not
 *                  have a route ID. Therefore, the route is new, and
 *                  routeRequestID is invalid.
 ********************************************************************/
#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
BOOL HaveRoutingCapacity( BOOL validID, BYTE routeRequestID, SHORT_ADDR routeSourceAddress,
    SHORT_ADDR routeDestAddress, BYTE commandOptions )
{
    BYTE                i;
    BYTE                maxTableEntry;
    BOOL                result;

    result = FALSE;

    // Check for capacity in the route discovery table.
    for( i=0; i<ROUTE_DISCOVERY_TABLE_SIZE; i++)
    {
        if ((routeDiscoveryTablePointer[i] == NULL) ||
            ( validID &&   //(routeSourceAddress.Val != macPIB.macShortAddress.Val) &&
             (routeDiscoveryTablePointer[i]->routeRequestID == routeRequestID) &&
             (routeDiscoveryTablePointer[i]->srcAddress.Val == routeSourceAddress.Val)))
        {
            result = TRUE;
        }
    }

    // If we have capacity in the route discovery table, make sure
    // we also have capacity in the routing table.  Note that we cannot
    // use the reserved routing table entries unless we are doing a
    // route repair.
    if (commandOptions & ROUTE_REPAIR)
    {
        maxTableEntry = ROUTING_TABLE_SIZE;
    }
    else
    {
        maxTableEntry = ROUTING_TABLE_SIZE-RESERVED_ROUTING_TABLE_ENTRIES;
    }

    if (result)
    {
        result = FALSE;
        #ifdef USE_EXTERNAL_NVM
        for( i=0, pCurrentRoutingEntry = routingTable;
            i<maxTableEntry;
            i++, pCurrentRoutingEntry+=(WORD)sizeof(ROUTING_ENTRY) )
        #else
        for( i=0, pCurrentRoutingEntry = routingTable;
            i<maxTableEntry;
            i++, pCurrentRoutingEntry++ )
        #endif
        {
            GetRoutingEntry( &currentRoutingEntry, pCurrentRoutingEntry );
            if ((currentRoutingEntry.destAddress.Val == routeDestAddress.Val) ||
                (currentRoutingEntry.status == ROUTE_INACTIVE))
            {
                result = TRUE;
            }
        }
    }

    return result;
}
#endif

/*********************************************************************
 * Function:        MESSAGE_ROUTING_STATUS InitiateRouteDiscovery( BOOL fromUpperLayers, BYTE discoverRoute )
 *
 * PreCondition:    None
 *
 * Input:           fromUpperLayers - if we are currently in a NLDE_DATA_request or
 *                          MCPS_DATA_indication
 *                  discoverRoute - Route Discovery indication
 *
 * Output:          None
 *
 * Side Effects:    routedMessage.message is allocated
 *
 * Overview:        If no message is currently awaiting route discovery,
 *                  this routine buffers the current message and
 *                  initiates route discovery.  If we already have a
 *                  message buffered, this message will be discarded.
 *
 * Note:            if fromUpperLayers is TRUE, we are servicing an
 *                      NLDE_DATA_request
 *                  if fromUpperLayers is FALSE, we are servicing an
 *                      MCPS_DATA_indication
 ********************************************************************/
#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
MESSAGE_ROUTING_STATUS InitiateRouteDiscovery( BOOL fromUpperLayers, BYTE discoverRoute )
{
    BYTE        i;
    BYTE        messageIndex;
    BYTE        *ptr;
    BYTE        radius;
    BYTE        rdIndex;
    BYTE        rtIndex;
    TICK        tempTime;

    // Find an empty place to store the buffered message
    for (messageIndex=0; (messageIndex<NUM_BUFFERED_ROUTING_MESSAGES) && (nwkStatus.routingMessages[messageIndex]); messageIndex++) {}

    // If there are no empty places, we cannot route this message.
    if (messageIndex==NUM_BUFFERED_ROUTING_MESSAGES)
    {
        return ROUTE_FAILURE_NO_CAPACITY;
    }

    // Start to buffer the message
    if ((nwkStatus.routingMessages[messageIndex] = (BUFFERED_MESSAGE_INFO *)SRAMalloc( sizeof(BUFFERED_MESSAGE_INFO) )) == NULL)
    {
        return ROUTE_FAILURE_NO_CAPACITY;
    }

    if (fromUpperLayers)
    {
        nwkStatus.routingMessages[messageIndex]->sourceAddress.Val      = macPIB.macShortAddress.Val;
        nwkStatus.routingMessages[messageIndex]->destinationAddress.Val = params.NLDE_DATA_request.DstAddr.Val;
        nwkStatus.routingMessages[messageIndex]->dataLength             = (TX_HEADER_START - TxHeader) + 8 + TxData;

        if ((ptr = (BYTE *)SRAMalloc(nwkStatus.routingMessages[messageIndex]->dataLength)) == NULL)
        {
            free( nwkStatus.routingMessages[messageIndex] );
            return ROUTE_FAILURE_NO_CAPACITY;
        }

        nwkStatus.routingMessages[messageIndex]->dataPtr = ptr;

        // Buffer the current message.  Create the NWK header first.
        *ptr++ = NWK_FRAME_DATA | (nwkProtocolVersion<<2) | (discoverRoute<<6); // Frame Control byte LSB
        *ptr++ = (params.NLDE_DATA_request.SecurityEnable<<1); // Frame Control byte MSB
        *ptr++ = params.NLDE_DATA_request.DstAddr.byte.LSB;
        *ptr++ = params.NLDE_DATA_request.DstAddr.byte.MSB;
        *ptr++ = macPIB.macShortAddress.byte.LSB;
        *ptr++ = macPIB.macShortAddress.byte.MSB;
        radius = params.NLDE_DATA_request.BroadcastRadius;
        *ptr++ = radius;
        *ptr++ = params.NLDE_DATA_request.NsduHandle;

        // Buffer any headers from upper layers
        while (TxHeader < TX_HEADER_START)
        {
            *ptr++ = TxBuffer[++TxHeader];
        }

        // Buffer the NWK payload.
        i = 0;
        while (TxData--)
        {
            *ptr++ = TxBuffer[i++];
        }
    }
    else
    {
        // Back up the packet to point to the beginning of the NWK header.  Pull the final route destination out
        // of the NWK header while we're doing it.
        params.MCPS_DATA_indication.msdu        -= 4;
        nwkStatus.routingMessages[messageIndex]->sourceAddress.Val           = params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val;
        nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.MSB = *(--params.MCPS_DATA_indication.msdu);
        nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.LSB = *(--params.MCPS_DATA_indication.msdu);
        params.MCPS_DATA_indication.msdu        -= 2;
        params.MCPS_DATA_indication.msduLength  += 8;
        nwkStatus.routingMessages[messageIndex]->dataLength                  = params.MCPS_DATA_indication.msduLength;

        if ((ptr = (BYTE *)SRAMalloc(nwkStatus.routingMessages[messageIndex]->dataLength)) == NULL)
        {
            free( nwkStatus.routingMessages[messageIndex] );
            return ROUTE_FAILURE_NO_CAPACITY;
        }

        nwkStatus.routingMessages[messageIndex]->dataPtr = ptr;

        // Buffer the current message.  Create the NWK header first.
        *ptr++ = (NWKGet() & 0x3F) | (discoverRoute << 6); // Put in new route discovery field
        *ptr++ = NWKGet();      // frameCONMSB.Val;
        *ptr++ = NWKGet();      // destAddr.byte.LSB;
        *ptr++ = NWKGet();      // destAddr.byte.MSB;
        *ptr++ = NWKGet();      // srcAddr.byte.LSB;
        *ptr++ = NWKGet();      // srcAddr.byte.MSB;
        radius = NWKGet()-1;
        *ptr++ = radius;        // broadcastRadius;
        *ptr++ = NWKGet();      // broadcastSequence;

        // Buffer the NWK payload.
        while (params.MCPS_DATA_indication.msduLength)
        {
            *ptr++ = NWKGet();
        }

        // We will discard the message when we return from this function. - NWKDiscardRx();
    }
    // Record when we started route discovery so we can purge this
    // message in case route discovery fails.
    nwkStatus.routingMessages[messageIndex]->timeStamp = TickGet();

    // Create the Routing and Route Discovery table entries.
    if (!CreateRoutingTableEntries( nwkStatus.routingMessages[messageIndex]->destinationAddress, &rdIndex, &rtIndex ))
    {
        // Destroy the buffered message.
        SRAMfree( nwkStatus.routingMessages[messageIndex]->dataPtr );
        free( nwkStatus.routingMessages[messageIndex] );
        return ROUTE_FAILURE_NO_CAPACITY;
    }

    // We successfully buffered the current message and created routing table entries.  Since
    // route discovery broadcasts must be triggered in the background, unblock the transmit path.
    ZigBeeUnblockTx();

    if (!(ptr = routeDiscoveryTablePointer[rdIndex]->forwardRREQ = SRAMalloc(sizeof(ROUTE_REQUEST_COMMAND)+8)))
    {
        // Destroy the Routing Table entry.
        currentRoutingEntry.status = ROUTE_DISCOVERY_UNDERWAY;
        PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );

        // Destroy the Route Discovery Table entry.
        free( routeDiscoveryTablePointer[rdIndex] );

        // Destroy the buffered message.
        SRAMfree( nwkStatus.routingMessages[messageIndex]->dataPtr );
        free( nwkStatus.routingMessages[messageIndex] );
        return ROUTE_FAILURE_NO_CAPACITY;
    }

    // Populate the remainder of the Route Discovery table entry.
    routeDiscoveryTablePointer[rdIndex]->routeRequestID             = nwkStatus.routeRequestID;
    routeDiscoveryTablePointer[rdIndex]->srcAddress                 = macPIB.macShortAddress;
    //  routeDiscoveryTablePointer[rdIndex]->senderAddress          = unneeded
    routeDiscoveryTablePointer[rdIndex]->forwardCost                = 0;
    routeDiscoveryTablePointer[rdIndex]->status.transmitCounter     = nwkcInitialRREQRetries + 1;
    routeDiscoveryTablePointer[rdIndex]->status.initialRebroadcast  = 0;     // We are initiating, so we use only nwkcRREQRetryInterval

    // Buffer the RREQ so we can transmit (and retransmit...) from the background.  Watch the order that it's loaded.

    // Load up the NWK payload - the route request command frame.
    *ptr++ = NWK_COMMAND_ROUTE_REQUEST;
    *ptr++ = ROUTE_REPAIR;                  // Route options; we will always use Route Repair
    *ptr++ = nwkStatus.routeRequestID++;
    *ptr++ = nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.LSB;
    *ptr++ = nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.MSB;
    *ptr++ = 0x00; // path cost

    // Load up the NWK header (backwards).
    *ptr++ = NLME_GET_nwkBCSN();
    *ptr++ = radius;
    *ptr++ = macPIB.macShortAddress.byte.MSB;
    *ptr++ = macPIB.macShortAddress.byte.LSB;
    *ptr++ = 0xFF; //nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.MSB;
    *ptr++ = 0xFF; //nwkStatus.routingMessages[messageIndex]->destinationAddress.byte.LSB;
#ifdef I_SUPPORT_SECURITY
    *ptr++ = 2;
#else
    *ptr++ = 0; // nwkFrameControlMSB TODO check security setting for this
#endif
    *ptr++ = NWK_FRAME_CMD | (nwkProtocolVersion<<2);    // nwkFrameControlLSB

    // Set the timer for the first transmission, so it goes off immediately.
    tempTime = TickGet();
    routeDiscoveryTablePointer[rdIndex]->rebroadcastTimer.Val = tempTime.Val - (DWORD)(ONE_SECOND*nwkcRREQRetryInterval/1000);

    return ROUTE_MESSAGE_BUFFERED;
}
#endif


/*********************************************************************
 * Function:        BOOL IsDescendant( SHORT_ADDR parentAddr,
 *                                     SHORT_ADDR childAddr,
 *                                     BYTE parentDepth )
 *
 * PreCondition:    None
 *
 * Input:           parentAddr - short address of the parent device
 *                  childAddr - short address of the child device
 *                  parentDepth - depth of the parent
 *
 * Output:          TRUE - child is a descendant of the parent
 *                  FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function determines if the child device is a
 *                  descendant of the parent device.  If the
 *                  If I am the coordinator, the device is always my
 *                  descendant.  Otherwise, the device is my descendant
 *                  if its address is within the address range that I
 *                  can allocate.
 *
 * Note:            None
 ********************************************************************/
#ifdef I_SUPPORT_ROUTING
BOOL IsDescendant( SHORT_ADDR parentAddr, SHORT_ADDR childAddr, BYTE parentDepth )
{
    if (parentDepth == 0)
    {
        return TRUE;
    }
    else
    {
        if ((parentAddr.Val < childAddr.Val) &&
            (childAddr.Val < (parentAddr.Val + GetCSkipVal( parentDepth-1 ))))
            return TRUE;
        else
            return FALSE;
    }
}
#endif

/*********************************************************************
 * Function:        void MarkNeighborAsPasssiveACKed( BYTE BTTindex )
 *
 * PreCondition:    This function must be called while MCPS_DATA_indication
 *                  parameters are valid.
 *
 * Input:           BTTindex
 *
 * Output:          None
 *
 * Side Effects:    The indicated BTR will be updated to reflect that
 *                  the source of the current message has rebroadcast
 *                  the message.
 *
 * Overview:        This routine marks indicated BTR as having been
 *                  passive ACK'd by the source of the current message.
 *
 * Note:            None
 ********************************************************************/
#ifndef I_AM_RFD
void MarkNeighborAsPasssiveACKed( BYTE BTTindex )
{
    BYTE    j;

    j = 0;

    while (j<MAX_NEIGHBORS)
    {
        #ifdef USE_EXTERNAL_NVM
            pCurrentNeighborRecord = neighborTable + (WORD)j * (WORD)sizeof(NEIGHBOR_RECORD);
        #else
            pCurrentNeighborRecord = &neighborTable[j];
        #endif
        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
        if (currentNeighborRecord.shortAddr.Val == params.MCPS_DATA_indication.SrcAddr.ShortAddr.Val)
        {
            nwkStatus.BTT[BTTindex]->flags[j].bits.bMessageNotRelayed = 0;
            break;
        }
        j++;
    }
}
#endif

/*******************************************************************************
 * Function:        BYTE NLME_GET_nwkBCSN( void )
 *
 * PreCondition:    none
 *
 * Input:           none
 *
 * Output:          a valid sequence number to use in the NWK header
 *
 * Side Effects:    None
 *
 * Overview:        This function provides the caller with a valid sequence
 *                  number to use in the NWK header.
 *
 * Note:            INVALID_NWK_HANDLE is reserved
 ******************************************************************************/
BYTE NLME_GET_nwkBCSN( void )
{
    NIB.nwkBCSN++;
    if (NIB.nwkBCSN == INVALID_NWK_HANDLE) NIB.nwkBCSN++;
    return NIB.nwkBCSN;
}

/*********************************************************************
 * Function:        void NWKClearNeighborTable( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears entire neighbor table.
 *
 * Note:            None
 ********************************************************************/
void NWKClearNeighborTable( void )
{
    BYTE i;

    // We are clearing the neighbor table, so we will lose all parent information.
    #ifndef I_AM_COORDINATOR
        ZigBeeStatus.flags.bits.bTryOrphanJoin = 0;
    #endif

    //memset( &currentNeighborRecord, 0x00, sizeof(NEIGHBOR_RECORD) );
    currentNeighborRecord.deviceInfo.Val = 0;

    #ifdef USE_EXTERNAL_NVM
    for (i=0, pCurrentNeighborRecord = neighborTable; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD))
    #else
    for (i=0, pCurrentNeighborRecord = neighborTable; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord++)
    #endif
    {
        PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );
    }

    // Reset count fields, indicate that the address info is invalid, clear
    // the parent record index, and mark the table as valid.
    //memset( &currentNeighborTableInfo, 0x00, sizeof(NEIGHBOR_TABLE_INFO) );
    currentNeighborTableInfo.neighborTableSize  = 0;
    #ifndef I_AM_COORDINATOR
        currentNeighborTableInfo.parentNeighborTableIndex = INVALID_NEIGHBOR_KEY;
    #endif
    #ifndef I_AM_END_DEVICE
        currentNeighborTableInfo.numChildren                        = 0;
        currentNeighborTableInfo.numChildRouters                    = 0;
        currentNeighborTableInfo.flags.bits.bChildAddressInfoValid  = 0;
    #endif
    currentNeighborTableInfo.validityKey = NEIGHBOR_TABLE_VALID;

    PutNeighborTableInfo();
}

/*********************************************************************
 * Function:        void NWKClearRoutingTable( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function clears the routing table.
 *
 * Note:            None
 ********************************************************************/
#if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
void NWKClearRoutingTable( void )
{
    BYTE    i;

    currentRoutingEntry.status = ROUTE_INACTIVE;
    currentRoutingEntry.destAddress.Val = 0xFFFF;
    pCurrentRoutingEntry = routingTable;

    #ifdef USE_EXTERNAL_NVM
    for (i=0; i<ROUTING_TABLE_SIZE; i++, pCurrentRoutingEntry+=(WORD)sizeof(ROUTING_ENTRY))
    #else
    for (i=0; i<ROUTING_TABLE_SIZE; i++, pCurrentRoutingEntry++)
    #endif
    {
        PutRoutingEntry( pCurrentRoutingEntry, &currentRoutingEntry );
    }
}
#endif

/*********************************************************************
 * Function:        BYTE NWKGet( void )
 *
 * PreCondition:    Must be called from the MCPS_DATA_indication
 *                  primitive.
 *
 * Input:           none
 *
 * Output:          One byte from the msdu if the length is greater
 *                  than 0; otherwise, 0.
 *
 * Side Effects:    The msdu pointer is incremented to point to the
 *                  next byte, and msduLength is decremented.
 *
 * Overview:        This function returns the next byte from the
 *                  msdu.
 *
 * Note:            None
 ********************************************************************/

BYTE NWKGet( void )
{
    if (params.MCPS_DATA_indication.msduLength == 0)
    {
        return 0;
    }
    else
    {
        params.MCPS_DATA_indication.msduLength--;
        return *params.MCPS_DATA_indication.msdu++;
    }
}

/*********************************************************************
 * Function:        NEIGHBOR_KEY NWKLookupNodeByLongAddr(LONG_ADDR *longAddr)
 *
 * PreCondition:    None
 *
 * Input:           longAddr        - Long address of node that is
 *                                    to be looked up in table.
 *
 * Output:          Handle to matching node.
 *
 * Side Effects:    If found, the matching neighbor table entry is in
 *                  currentNeighborRecord and pCurrentNeighborRecord
 *                  points to its location in the table.
 *
 * Overview:        Searches neighbor table for matching short address
 *
 * Note:            If the neighbor table is enlarged, it may be
 *                  necessary to read only the address and device info
 *                  fields until the record is found.
 ********************************************************************/
NEIGHBOR_KEY NWKLookupNodeByLongAddr( LONG_ADDR *longAddr )
{
    BYTE i;

    pCurrentNeighborRecord = neighborTable;

    #ifdef USE_EXTERNAL_NVM
    for ( i=0; i<MAX_NEIGHBORS; i++, pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD))
    #else
    for ( i=0; i<MAX_NEIGHBORS; i++, pCurrentNeighborRecord++)
    #endif
    {
        // Read the address into RAM.
        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

        // Make sure that this record is in use
        if ( currentNeighborRecord.deviceInfo.bits.bInUse )
        {

            if ( !memcmp((void*)longAddr,
                        (void*)&currentNeighborRecord.longAddr,
                        (size_t)(sizeof(LONG_ADDR))) )
            {
                return i;
            }
        }
    }

    return INVALID_NEIGHBOR_KEY;
}

/*********************************************************************
 * Function:        NEIGHBOR_KEY NWKLookupNodeByShortAddrVal( WORD shortAddrVal )
 *
 * PreCondition:    None
 *
 * Input:           shortAddrVal   - Short address word value of node that is
 *                                    to be looked up in table.
 *
 * Output:          Handle to matching node or INVALID_NEIGHBOR_KEY
 *
 * Side Effects:    If found, the matching neighbor table entry is in
 *                  currentNeighborRecord and pCurrentNeighborRecord
 *                  points to its location in the table.
 *
 * Overview:        Searches neighbor table for matching short address
 *
 * Note:            If the neighbor table is enlarged, it may be
 *                  necessary to read only the address and device info
 *                  fields until the record is found.
 ********************************************************************/
NEIGHBOR_KEY NWKLookupNodeByShortAddrVal( WORD shortAddrVal )
{
    BYTE i;

    pCurrentNeighborRecord = neighborTable;

    #ifdef USE_EXTERNAL_NVM
    for ( i = 0; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD))
    #else
    for ( i = 0; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord++)
    #endif
    {
        // Read the record into RAM.
        GetNeighborRecord(&currentNeighborRecord, pCurrentNeighborRecord );

        // Make sure that this record is in use
        if ( currentNeighborRecord.deviceInfo.bits.bInUse )
        {
            if ( currentNeighborRecord.shortAddr.Val == shortAddrVal )
            {
                return i;
            }
        }
    }

    return INVALID_NEIGHBOR_KEY;
}

/*********************************************************************
 * Function:        BOOL NWKThisIsMyLongAddress( LONG_ADDR *address )
 *
 * PreCondition:    None
 *
 * Input:           address - pointer to a long address
 *
 * Output:          TRUE - values at address match my long address
 *                  FALSE - no match
 *
 * Side Effects:    None
 *
 * Overview:        This routine determines if a long address is my
 *                  long address.
 *
 * Note:            None
 ********************************************************************/

BOOL NWKThisIsMyLongAddress( LONG_ADDR *address )
{
    LONG_ADDR   tempMACAddr;

    GetMACAddress( &tempMACAddr );

    if (!memcmp( (void *)address, (void *)&tempMACAddr, 8 ))
        return TRUE;
    else
        return FALSE;
}

/*********************************************************************
 * Function:        void NWKTidyNeighborTable( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Removes non-child nodes from the neighbor table.
 *
 * Note:            None
 ********************************************************************/
void NWKTidyNeighborTable( void )
{
    BYTE i;

    #ifdef USE_EXTERNAL_NVM
    for (i=0, pCurrentNeighborRecord = neighborTable; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD))
    #else
    for (i=0, pCurrentNeighborRecord = neighborTable; i < MAX_NEIGHBORS; i++, pCurrentNeighborRecord++)
    #endif
    {
        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
        if (currentNeighborRecord.deviceInfo.bits.bInUse &&
            ((currentNeighborRecord.deviceInfo.bits.Relationship != NEIGHBOR_IS_CHILD) &&
             (currentNeighborRecord.deviceInfo.bits.Relationship != NEIGHBOR_IS_PARENT)))
        {
            // This record isn't a child or parent, so remove it.
            currentNeighborRecord.deviceInfo.Val = 0;
            currentNeighborTableInfo.neighborTableSize--;
            PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );
        }
    }

    PutNeighborTableInfo();
}

/*********************************************************************
 * Function:        void Prepare_MCPS_DATA_request( WORD macDestAddressVal, BYTE *msduHandle )
 *
 * PreCondition:    None
 *
 * Input:           macDestAddressVal - the MAC destination address of
 *                          the message as a WORD to allow for constants
 *
 * Output:          MCPS_DATA_request parameters are loaded.
 *
 * Side Effects:    None
 *
 * Overview:        This routine loads all of the MCPS_DATA_request
 *                  parameters based on the input MAC address.
 *
 * Note:            None
 ********************************************************************/
void Prepare_MCPS_DATA_request( WORD macDestAddressVal, BYTE *msduHandle )
{
    // Block the transmit path
    ZigBeeBlockTx();

    params.MCPS_DATA_request.SrcAddrMode = 0x02;
    params.MCPS_DATA_request.SrcPANId = macPIB.macPANId;
    params.MCPS_DATA_request.SrcAddr.ShortAddr.Val = macPIB.macShortAddress.Val;
    params.MCPS_DATA_request.DstAddrMode = 0x02;
    params.MCPS_DATA_request.DstPANId = macPIB.macPANId;
    params.MCPS_DATA_request.DstAddr.ShortAddr.Val = macDestAddressVal;
    params.MCPS_DATA_request.msduHandle = *msduHandle = MLME_GET_macDSN();
    params.MCPS_DATA_request.TxOptions.Val = 0x00;
    if (macDestAddressVal != 0xFFFF)
    {
        params.MCPS_DATA_request.TxOptions.bits.acknowledged_transmission = 1;
        if (NWKLookupNodeByShortAddrVal( macDestAddressVal ) != INVALID_NEIGHBOR_KEY)
        {
            if (!currentNeighborRecord.deviceInfo.bits.RxOnWhenIdle)
            {
                params.MCPS_DATA_request.TxOptions.bits.indirect_transmission = 1;
            }
        }
#ifdef I_SUPPORT_SECURITY
     // this request is not originated from MAC layer, so no security requirement
     params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = FALSE;
#endif
        // TODO params.MCPS_DATA_request.TxOptions.bits.security_enabled_transmission = ???
    }
    params.MCPS_DATA_request.frameType = FRAME_DATA;
}

/*********************************************************************
 * Function:        void RemoveNeighborTableEntry( void )
 *
 * PreCondition:    currentNeighborRecord and pCurrentNeighborRecord
 *                  are set to the record to be removed, and the
 *                  record is in use (bInUse == 1)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Beacon payload is updated.
 *
 * Overview:        Removes the current node from the neighbor table.
 *
 * Note:            None
 ********************************************************************/
void RemoveNeighborTableEntry( void )
{
    #ifndef I_AM_END_DEVICE
    if (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD)
    {
        currentNeighborTableInfo.numChildren--;
        if (currentNeighborRecord.deviceInfo.bits.deviceType == DEVICE_ZIGBEE_ROUTER)
        {
            currentNeighborTableInfo.numChildRouters--;
        }
        SetBeaconPayload();
    }
    else
    #endif
    #ifndef I_AM_COORDINATOR
        if (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_PARENT)
        {
            // My parent is no longer on the network.
            // TODO how much should really be done here??? errata isn't very complete.
            currentNeighborTableInfo.parentNeighborTableIndex = INVALID_NEIGHBOR_KEY;
        }
    #endif

    // Remove the neighbor record by clearing the bInUse flag.
    currentNeighborRecord.deviceInfo.bits.bInUse = 0;
    currentNeighborRecord.deviceInfo.Val = 0;
    PutNeighborRecord( pCurrentNeighborRecord, &currentNeighborRecord );

    // Update the table size.
    currentNeighborTableInfo.neighborTableSize--;

    // Commit all neighbor table info changes, from here and from the caller.
    PutNeighborTableInfo();
}

/*********************************************************************
 * Function:        BOOL RequestedPANIDFound( BYTE channel )
 *
 * PreCondition:    params.MLME_START_request.PANId must have the PAN ID
 *                  being checked for a conflict
 *
 * Input:           channel - the channel to check for PAN ID conflicts
 *
 * Output:          TRUE - conflict found
 *                  FALSE - no conflict found
 *
 * Side Effects:    None
 *
 * Overview:        This function determines if a node in our neighbor
 *                  table on the specified channel has the same PAN ID
 *                  as the one requested.
 *
 * Note:            This routine may not work if the destination
 *                  is our child.
 ********************************************************************/
#ifdef I_AM_COORDINATOR
BOOL RequestedPANIDFound( BYTE channel )
{
    BYTE    i;

    pCurrentNeighborRecord = neighborTable;

    #ifdef USE_EXTERNAL_NVM
    for ( i=0; i<MAX_NEIGHBORS; i++, pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD))
    #else
    for ( i=0; i<MAX_NEIGHBORS; i++, pCurrentNeighborRecord++)
    #endif
    {
        // Read the address into RAM.
        GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );

        // Make sure that this record is in use
        if (currentNeighborRecord.deviceInfo.bits.bInUse)
        {
            if ((currentNeighborRecord.LogicalChannel == channel) &&
                (currentNeighborRecord.panID.Val == params.MLME_START_request.PANId.Val))
            {
                return TRUE;
            }
        }
    }
    return FALSE;
}
#endif

/*********************************************************************
 * Function:        BOOL RouteAlongTree( SHORT_ADDR destTarget, SHORT_ADDR *dest )
 *
 * PreCondition:    The destination must not be our child (?).
 *
 * Input:           destTarget - final target destination
 *                  dest - pointer to calculated destination address
 *
 * Output:          TRUE - valid destination address found
 *                  FALSE - could not find valid destination address
 *
 * Side Effects:    None
 *
 * Overview:        This function determines the address of the
 *                  next node to send a routing message to based on
 *                  tree hierarchy.  We do not check the path of the
 *                  message, only the destination address.  If the
 *                  message is going to one of our descendants, pass
 *                  it to the next level parent.  Otherwise, pass it
 *                  to our parent.  If we are a coordinator and we
 *                  cannot find the next level parent, return FALSE.
 *
 * Note:            This routine may not work if the destination
 *                  is our child.
 ********************************************************************/
#ifdef I_SUPPORT_ROUTING
BOOL RouteAlongTree( SHORT_ADDR destTarget, SHORT_ADDR *destNextHop )
{
    #ifdef I_AM_END_DEVICE

        // If I am an FFD end device, I have no children, so send the message to my parent.
        *destNextHop = macPIB.macCoordShortAddress;
        return TRUE;

    #else

        BOOL found;
        BYTE i;

        // Find which of my child nodes is the parent of the destination.
        found = FALSE;

        // Get ROM address of neighborTable in RAM.
        pCurrentNeighborRecord = neighborTable;

        for ( i=0; i < MAX_NEIGHBORS; i++ )
        {
            GetNeighborRecord( &currentNeighborRecord, pCurrentNeighborRecord );
            if ((currentNeighborRecord.deviceInfo.bits.bInUse) &&
                (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD))
            {
                if ((destTarget.Val == currentNeighborRecord.shortAddr.Val) ||
                     IsDescendant( currentNeighborRecord.shortAddr, destTarget, currentNeighborTableInfo.depth+1 ))
                {
                    // The destination address is either my child or a
                    // descendant of one of my children.  Pass the message
                    // to that child.
                    *destNextHop = currentNeighborRecord.shortAddr;
                    found = TRUE;
                    break;
                }
            }
            #ifdef USE_EXTERNAL_NVM
                pCurrentNeighborRecord += (WORD)sizeof(NEIGHBOR_RECORD);
            #else
                pCurrentNeighborRecord++;
            #endif
        }

        #ifndef I_AM_COORDINATOR
            // If none of my child nodes is the parent and the destination
            // should not be one of my descendants, so send the message to
            // my parent.
            if (!found && !IsDescendant( macPIB.macShortAddress, destTarget, currentNeighborTableInfo.depth ))
            {
                *destNextHop = macPIB.macCoordShortAddress;
                found = TRUE;
            }
        #endif

        return found;

    #endif
}
#endif

/*********************************************************************
 * Function:        void SetBeaconPayload( void )
 *
 * PreCondition:    None
 *
 * Input:           none
 *
 * Output:          none
 *
 * Side Effects:    Beacon Payload in MAC PIB is updated.
 *
 * Overview:        Sets the beacon payload, particularly the indications
 *                  as to whether or not end devices and routers can
 *                  join to this node.
 *
 * Note:            None
 ********************************************************************/
#ifndef I_AM_END_DEVICE
void SetBeaconPayload( void )
{
    macPIB.macBeaconPayload[0] = ZIGBEE_PROTOCOL_ID;
    macPIB.macBeaconPayload[1] = (nwkcProtocolVersion  << 4) | MY_STACK_PROFILE_ID;
    macPIB.macBeaconPayload[2] = currentNeighborTableInfo.depth << 3;

    if (macPIB.macAssociationPermit &&
        (currentNeighborTableInfo.numChildren < NIB_nwkMaxChildren) &&
        (currentNeighborTableInfo.neighborTableSize < MAX_NEIGHBORS))
    {
        if (!nwkStatus.flags.bits.bAllEndDeviceAddressesUsed)
        {
            macPIB.macBeaconPayload[2] |= 0x80; // End Devices can join
        }
        if ((currentNeighborTableInfo.depth < (NIB_nwkMaxDepth-1)) &&
            (currentNeighborTableInfo.numChildRouters < NIB_nwkMaxRouters) &&
            !nwkStatus.flags.bits.bAllRouterAddressesUsed)
        {
            macPIB.macBeaconPayload[2] |= 0x04; // Routers can join
        }
    }
}
#endif

/*********************************************************************
 * Function:        void VariableAndTableInitialization( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    All NWK layer variables and tables are
 *                  initialized except the neighbor table
 *
 * Overview:        This routine initializes all variables and constants
 *                  except the neighbor table.
 *
 * Note:            No pointers are initialized here.
 ********************************************************************/
void VariableAndTableInitialization( BOOL force )
{
    BYTE    i;

    // Reset simple variables
    NIB.nwkBCSN = RANDOM_LSB;

    ZigBeeStatus.flags.Val = 0;
    nwkStatus.flags.Val = 0;

    #ifndef NIB_STATIC_IMPLEMENTATION
        NIB.nwkPassiveAckTimeout                = DEFAULT_nwkPassiveAckTimeout;
        NIB.nwkMaxBroadcastRetries              = DEFAULT_nwkMaxBroadcastRetries;
        NIB.nwkMaxChildren                      = PROFILE_nwkMaxChildren;
        NIB.nwkMaxDepth                         = PROFILE_nwkMaxDepth;
        NIB.nwkMaxRouters                       = PROFILE_nwkMaxRouters;
        NIB.nwkNeighborTable                    = DEFAULT_nwkNeighborTable;
        NIB.nwkNetworkBroadcastDeliveryTime     = DEFAULT_nwkNetworkBroadcastDeliveryTime;
        NIB.nwkReportConstantCost               = DEFAULT_nwkReportConstantCost;
        NIB.nwkRouteDiscoveryRetriesPermitted   = DEFAULT_nwkRouteDiscoveryRetriesPermitted;
        NIB.nwkRouteTable                       = DEFAULT_nwkRouteTable;
        NIB.nwkSymLink                          = DEFAULT_nwkSymLink;
        NIB.nwkCapabilityInformation            = DEFAULT_nwkCapabilityInformation;
        NIB.nwkUseTreeAddrAlloc                 = DEFAULT_nwkUseTreeAddrAlloc;
        NIB.nwkUseTreeRouting                   = DEFAULT_nwkUseTreeRouting;
    #endif

	#ifdef I_SUPPORT_SECURITY
    	nwkSecurityLevel = PROFILE_nwkSecurityLevel;
    	securityStatus.flags.bits.nwkSecureAllFrames = PROFILE_nwkSecureAllFrames;
    	securityStatus.flags.bits.nwkAllFresh = DEFAULT_nwkAllFresh;
	#endif

    // Initialize NWK frame handles awaiting confirmation.
    for (i=0; i<MAX_NWK_FRAMES; i++)
    {
        nwkConfirmationHandles[i].nsduHandle = INVALID_NWK_HANDLE;
    }

    if (!force)
    {
        if (nwkStatus.discoveryInfo.channelList)
        {
            SRAMfree( (unsigned char *)nwkStatus.discoveryInfo.channelList );
        }
    }
    nwkStatus.discoveryInfo.channelList = NULL;

    #ifndef I_AM_RFD
        // Initialize the Broadcast Transaction Table pointers.
        for (i=0; i<NUM_BUFFERED_BROADCAST_MESSAGES; i++)
        {
            if (!force)
            {
                if (nwkStatus.BTT[i])
                {
                    SRAMfree( nwkStatus.BTT[i]->dataPtr );
                    SRAMfree( (unsigned char *)nwkStatus.BTT[i] );
                }
            }
            nwkStatus.BTT[i] = NULL;
        }
    #endif

    #if defined(I_SUPPORT_ROUTING) && !defined(USE_TREE_ROUTING_ONLY)
    {
        // Initialize the buffered messages awaiting routing.
        for (i=0; i<NUM_BUFFERED_ROUTING_MESSAGES; i++)
        {
            if (!force)
            {
                if (nwkStatus.routingMessages[i])
                {
                    SRAMfree( nwkStatus.routingMessages[i]->dataPtr );
                    SRAMfree( (unsigned char *)nwkStatus.routingMessages[i] );
                }
            }
            nwkStatus.routingMessages[i] = NULL;
        }

        // Initialize the Route Discovery Table.
        for (i=0; i<ROUTE_DISCOVERY_TABLE_SIZE; i++)
        {
            if (!force)
            {
                if (routeDiscoveryTablePointer[i])
                {
                    SRAMfree( (unsigned char *)routeDiscoveryTablePointer[i] );
                }
            }
            routeDiscoveryTablePointer[i] = NULL;
        }

        // Clear the Routing Table.
        NWKClearRoutingTable();

    }
    #endif
    
	#ifdef I_SUPPORT_SECURITY

  		#ifdef I_SUPPORT_RES_SECURITY
    	{
        	BYTE ActiveKeyIndex;

        	GetNwkActiveKeyNumber(&ActiveKeyIndex);

        	if( ActiveKeyIndex == 0x01 || ActiveKeyIndex == 0x02 )
        	{
            	#ifdef USE_EXTERNAL_NVM
                	GetNwkKeyInfo( &currentNetworkKeyInfo, networkKeyInfo + (ActiveKeyIndex-1) * sizeof(NETWORK_KEY_INFO) );
            	#else
                	GetNwkKeyInfo( &currentNetworkKeyInfo, &(networkKeyInfo[ActiveKeyIndex-1]) );
            	#endif
            	if( currentNetworkKeyInfo.SeqNumber.v[1] == nwkMAGICResSeq )
            	{
	            	#if defined(USE_EXTERNAL_NVM)
		            	InitSecurityKey();
		        	#endif
                	goto InitializeFrame;
            	}
            	else
            	{
                	ActiveKeyIndex = nwkMAGICResSeq;
                	PutNwkActiveKeyNumber(&ActiveKeyIndex);
            	}
        	}

        	#ifdef NETWORK_KEY_BYTE00
        	else
        	{
            	BYTE key[16] = { NETWORK_KEY_BYTE00, NETWORK_KEY_BYTE01, NETWORK_KEY_BYTE02, NETWORK_KEY_BYTE03,
                    NETWORK_KEY_BYTE04, NETWORK_KEY_BYTE05, NETWORK_KEY_BYTE06, NETWORK_KEY_BYTE07, NETWORK_KEY_BYTE08,
                    NETWORK_KEY_BYTE09, NETWORK_KEY_BYTE10, NETWORK_KEY_BYTE11, NETWORK_KEY_BYTE12, NETWORK_KEY_BYTE13,
                    NETWORK_KEY_BYTE14, NETWORK_KEY_BYTE15};
            	memcpy((void *)&(currentNetworkKeyInfo.NetKey), (void *)key, 16);
            	currentNetworkKeyInfo.SeqNumber.v[0] = NETWORK_KEY_SEQ;
            	currentNetworkKeyInfo.SeqNumber.v[1] = nwkMAGICResSeq;
            	#ifdef USE_EXTERNAL_NVM
        			plainSecurityKey[0] = currentNetworkKeyInfo;
        			SetSecurityKey(0, currentNetworkKeyInfo);
        			plainSecurityKey[1].SeqNumber.v[1] = 0xFF; // disable key 2
            	#else
            		PutNwkKeyInfo( &networkKeyInfo , &currentNetworkKeyInfo );
            	#endif
            	i = 0x01;
            	PutNwkActiveKeyNumber(&i);
        	}
        	#endif

InitializeFrame:
        	OutgoingFrameCount[0].Val = 0;
        	OutgoingFrameCount[1].Val = 0;
        	for(i = 0; i < MAX_NEIGHBORS; i++)
        	{
            	IncomingFrameCount[0][i].Val = 0;
            	IncomingFrameCount[1][i].Val = 0;
        	}

    	}
  		#endif

	#endif
}
