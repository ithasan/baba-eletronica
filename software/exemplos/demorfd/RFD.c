 /*
    Microchip ZigBee Stack

    Demo RFD

    This demonstration shows how a ZigBee RFD can be set up.  This demo allows
    the PICDEM Z Demostration Board to act as either a "Switching Load Controller"
    (e.g. a light) or a "Switching Remote Control" (e.g. a switch) as defined by
    the Home Controls, Lighting profile.  It is designed to interact with a
    second PICDEM Z programmed with the Demo Coordinator project.

    To give the PICDEM Z "switch" capability, uncomment the I_AM_SWITCH definition
    below.  To give the PICDEM Z "light" capability, uncomment the I_AM_LIGHT
    definition below.  The PICDEM Z may have both capabilities enabled.  Be sure
    that the corresponding Demo Coordinator device is programmed with complementary
    capabilities.  NOTE - for simplicity, the ZigBee simple descriptors for this
    demonstration are fixed.

    If this node is configured as a "switch", it can discover the network address
    of the "light" using two methods.  If the USE_BINDINGS definition is
    uncommented below, then End Device Binding must be performed between the
    "switch" and the "light" before messages can be sent and received successfully.
    If USE_BINDINGS is commented out, then the node will default to the probable
    network address of the other node, and messages may be able to be sent
    immediately.  However, the node will also be capable of performing Device
    Discovery to discover the actual network address of the other node, in case
    the network was formed with alternate short address assignments.  NOTE: The
    USE_BINDINGS definition must be the same in both the RFD and the ZigBee
    Coordinator nodes.

    Switch functionality is as follows:
        RB4, I_AM_SWITCH defined, sends a "toggle" message to the other node's "light"
        RB4, I_AM_SWITCH not defined, no effect
        RB5, USE_BINDINGS defined, sends an End Device Bind request
        RB5, USE_BINDINGS undefined, sends a NWK_ADDR_req for the MAC address specified

    End Device Binding
    ------------------
    If the USE_BINDINGS definition is uncommented, the "switch" will send an
    APS indirect message to toggle the "light".  In order for the message to
    reach its final destination, a binding must be created between the "switch"
    and the "light".  To do this, press RB5 on one PICDEM Z, and then press RB5
    on the other PICDEM Z within 5 seconds.  A message will be displayed indicating
    if binding was successful or not.  Note that End Device Binding is a toggle
    function.  Performing the operation again will unbind the nodes, and messages
    will not reach their final destination.

    Device Discovery
    ----------------
    If the USE_BINDINGS definition is not uncommented, pressing RB5 will send a
    broadcast NWK_ADDR_req message.  The NWK_ADDR_req message contains the MAC
    address of the desired node.  Be sure this address matches the address
    contained in the other node's zigbee.def file.

    NOTE: To speed network formation, ALLOWED_CHANNELS has been set to
    channel 12 only.

 *********************************************************************
 * FileName:        RFD.c
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
 * DF/KO/YY				11/27/06 Microchip ZigBee Stack v1.0-3.7
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/


//******************************************************************************
// Header Files
//******************************************************************************

// Include the main ZigBee header file.
#include "zAPL.h"

// If you are going to send data to a terminal, include this file.
#include "console.h"


//******************************************************************************
// Configuration Bits
//******************************************************************************

#if defined(MCHP_C18) && defined(__18F4620)
    #pragma romdata CONFIG1H = 0x300001
    const rom unsigned char config1H = 0b00000110;      // HSPLL oscillator

    #pragma romdata CONFIG2L = 0x300002
    const rom unsigned char config2L = 0b00011111;      // Brown-out Reset Enabled in hardware @ 2.0V, PWRTEN disabled

    #pragma romdata CONFIG2H = 0x300003
    const rom unsigned char config2H = 0b00010010;      // HW WD disabled, 1:512 prescaler

    #pragma romdata CONFIG3H = 0x300005
    const rom unsigned char config3H = 0b10000000;      // PORTB digital on RESET

    #pragma romdata CONFIG4L = 0x300006
    const rom unsigned char config4L = 0b10000001;      // DEBUG disabled,
                                                        // XINST disabled
                                                        // LVP disabled
                                                        // STVREN enabled
    #pragma romdata
#elif defined(HITECH_C18) && defined(_18F4620)
    // Set configuration fuses for HITECH compiler.
    __CONFIG(1, 0x0600);    // HSPLL oscillator
    __CONFIG(2, 0x101F);    // PWRTEN disabled, BOR enabled @ 2.0V, HW WD disabled, 1:128 prescaler
    __CONFIG(3, 0x8000);    // PORTB digital on RESET
    __CONFIG(4, 0x0081);    // DEBUG disabled,
                            // XINST disabled
                            // LVP disabled
                            // STVREN enabled
#endif

//******************************************************************************
// Constants
//******************************************************************************

// Switches and LEDs locations.
#define LEVEL_SENSOR            PORTBbits.RB5	// botão utilizado para simular o sensor de nível
#define MOVE_SENSOR             PORTBbits.RB4	// botão utilizado para simular o sensor de movimento


#define MOVE_SENSOR_LED         LATAbits.LATA0	// led ligado no RA0
#define LEVEL_SENSOR_LED        LATAbits.LATA1	// led ligado no RA1

#define BIND_STATE_BOUND            0
#define BIND_STATE_TOGGLE           1
#define BIND_STATE_UNBOUND          1
#define BIND_WAIT_DURATION          (6*ONE_SECOND)

#define ON							1
#define OFF							0

#define LIGHT_OFF                   0x00
#define LIGHT_ON                    0xFF
#define LIGHT_TOGGLE                0xF0

#define DELAY() {int i,j; for(i=0; i < 100; i++) for(j=0; j < 1000;j++);}

#define BLINK_LED(led) { led = OFF; DELAY(); led = ON; DELAY(); led = OFF; }


//******************************************************************************
// Application Variables
//******************************************************************************

static union
{
    struct
    {
        BYTE    bLevelSensorButtonPressed    : 1;
        BYTE    bMoveSensorButtonPressed     : 1;
        BYTE    bTryingToBind              	 : 1;
        BYTE    bDestinationAddressKnown     : 1;
    } bits;
    BYTE Val;
} myStatusFlags;
#define STATUS_FLAGS_INIT       0x00
#define TOGGLE_BOUND_FLAG       0x08

NETWORK_DESCRIPTOR  *currentNetworkDescriptor;
ZIGBEE_PRIMITIVE    currentPrimitive;
SHORT_ADDR          destinationAddress;
NETWORK_DESCRIPTOR  *NetworkDescriptor;
BYTE                orphanTries;


//******************************************************************************
// Function Prototypes
//******************************************************************************

void TransceiverInit(void);
void HardwareInit( void );
BOOL myProcessesAreDone( void );
void RFDSendMessage(unsigned short attributeID, unsigned char value);

//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************

void main(void)
{
    CLRWDT();
    ENABLE_WDT();

    currentPrimitive = NO_PRIMITIVE;
    NetworkDescriptor = NULL;
    orphanTries = 3;

    // If you are going to send data to a terminal, initialize the UART.
    ConsoleInit();

	ConsolePutROMString( (ROM char *)"Universidade Paulista - UNIP\r\n" );
	ConsolePutROMString( (ROM char *)"Daniel Gonçalves\r\n" );
	ConsolePutROMString( (ROM char *)"Projeto: Baba Eletronica\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"\r\n\r\n\r\n*************************************\r\n" );
    ConsolePutROMString( (ROM char *)"Microchip ZigBee(TM) Stack - v1.0-3.8\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"ZigBee RFD\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"Transceiver-MRF24J40\r\n\r\n" );

    // Inicializa o Hardware
    HardwareInit();

    // Inicializa a pilha ZigBee
    ZigBeeInit();

    // *************************************************************************
    // Outras Inicializações
    // *************************************************************************

    myStatusFlags.Val = STATUS_FLAGS_INIT;

    // Endereço padrão do Coordenador
    destinationAddress.Val = 0x0000;

    // Inicializa os LEDS
    MOVE_SENSOR_LED = ON;
    LEVEL_SENSOR_LED = ON;

    // Habilita as interrupções 
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;

    while (1)
    {
        CLRWDT();
        ZigBeeTasks( &currentPrimitive );

        switch (currentPrimitive)
        {
            case NLME_NETWORK_DISCOVERY_confirm:
                currentPrimitive = NO_PRIMITIVE;
                if (!params.NLME_NETWORK_DISCOVERY_confirm.Status)
                {
                    if (!params.NLME_NETWORK_DISCOVERY_confirm.NetworkCount)
                    {
                        ConsolePutROMString( (ROM char *)"No networks found.  Trying again...\r\n" );
                    }
                    else
                    {
                        // Save the descriptor list pointer so we can destroy it later.
                        NetworkDescriptor = params.NLME_NETWORK_DISCOVERY_confirm.NetworkDescriptor;

                        // Select a network to try to join.  We're not going to be picky right now...
                        currentNetworkDescriptor = NetworkDescriptor;

SubmitJoinRequest:
                        // not needed for new join params.NLME_JOIN_request.ScanDuration = ;
                        // not needed for new join params.NLME_JOIN_request.ScanChannels = ;
                        params.NLME_JOIN_request.PANId          = currentNetworkDescriptor->PanID;
                        ConsolePutROMString( (ROM char *)"Network(s) found. Trying to join " );
                        PrintChar( params.NLME_JOIN_request.PANId.byte.MSB );
                        PrintChar( params.NLME_JOIN_request.PANId.byte.LSB );
                        ConsolePutROMString( (ROM char *)".\r\n" );
                        params.NLME_JOIN_request.JoinAsRouter   = FALSE;
                        params.NLME_JOIN_request.RejoinNetwork  = FALSE;
                        params.NLME_JOIN_request.PowerSource    = NOT_MAINS_POWERED;
                        params.NLME_JOIN_request.RxOnWhenIdle   = FALSE;
                        params.NLME_JOIN_request.MACSecurity    = FALSE;
                        currentPrimitive = NLME_JOIN_request;
                    }
                }
                else
                {
                    PrintChar( params.NLME_NETWORK_DISCOVERY_confirm.Status );
                    ConsolePutROMString( (ROM char *)" Error finding network.  Trying again...\r\n" );
                }
                break;

            case NLME_JOIN_confirm:
                currentPrimitive = NO_PRIMITIVE;
                if (!params.NLME_JOIN_confirm.Status)
                {
                    ConsolePutROMString( (ROM char *)"Join successful!\r\n" );

                    // Free the network descriptor list, if it exists. If we joined as an orphan, it will be NULL.
                    while (NetworkDescriptor)
                    {
                        currentNetworkDescriptor = NetworkDescriptor->next;
                        free( NetworkDescriptor );
                        NetworkDescriptor = currentNetworkDescriptor;
                    }
                }
                else
                {
                    PrintChar( params.NLME_JOIN_confirm.Status );

                    // If we were trying as an orphan, see if we have some more orphan attempts.
                    if (ZigBeeStatus.flags.bits.bTryOrphanJoin)
                    {
                        // If we tried to join as an orphan, we do not have NetworkDescriptor, so we do
                        // not have to free it.

                        ConsolePutROMString( (ROM char *)" Could not join as orphan. " );
                        orphanTries--;
                        if (orphanTries == 0)
                        {
                            ConsolePutROMString( (ROM char *)"Must try as new node...\r\n" );
                            ZigBeeStatus.flags.bits.bTryOrphanJoin = 0;
                        }
                        else
                        {
                            ConsolePutROMString( (ROM char *)"Trying again...\r\n" );
                        }
                    }
                    else
                    {
                        ConsolePutROMString( (ROM char *)" Could not join selected network. " );
                        currentNetworkDescriptor = currentNetworkDescriptor->next;
                        if (currentNetworkDescriptor)
                        {
                            ConsolePutROMString( (ROM char *)"Trying next discovered network...\r\n" );
                            goto SubmitJoinRequest;
                        }
                        else
                        {
                            // We ran out of descriptors.  Free the network descriptor list, and fall
                            // through to try discovery again.
                            ConsolePutROMString( (ROM char *)"Cleaning up and retrying discovery...\r\n" );
                            while (NetworkDescriptor)
                            {
                                currentNetworkDescriptor = NetworkDescriptor->next;
                                free( NetworkDescriptor );
                                NetworkDescriptor = currentNetworkDescriptor;
                            }
                        }
                    }
                }
                break;

            case NLME_LEAVE_indication:
                if (!memcmppgm2ram( &params.NLME_LEAVE_indication.DeviceAddress, (ROM void *)&macLongAddr, 8 ))
                {
                    ConsolePutROMString( (ROM char *)"We have left the network.\r\n" );
                }
                else
                {
                    ConsolePutROMString( (ROM char *)"Another node has left the network.\r\n" );
                }
                currentPrimitive = NO_PRIMITIVE;
                break;

            case NLME_RESET_confirm:
                ConsolePutROMString( (ROM char *)"ZigBee Stack has been reset.\r\n" );
                currentPrimitive = NO_PRIMITIVE;
                break;

            case NLME_SYNC_confirm:
                switch (params.NLME_SYNC_confirm.Status)
                {
                    case SUCCESS:
                        // I have heard from my parent, but it has no data for me.  Note that
                        // if my parent has data for me, I will get an APSDE_DATA_indication.
                        ConsolePutROMString( (ROM char *)"No data available.\r\n" );
                        break;

                    case NWK_SYNC_FAILURE:
                        // I cannot communicate with my parent.
                        ConsolePutROMString( (ROM char *)"I cannot communicate with my parent.\r\n" );
                        break;

                    case NWK_INVALID_PARAMETER:
                        // If we call NLME_SYNC_request correctly, this doesn't occur.
                        ConsolePutROMString( (ROM char *)"Invalid sync parameter.\r\n" );
                        break;
                }
                currentPrimitive = NO_PRIMITIVE;
                break;

            case APSDE_DATA_indication:
                {
                    WORD_VAL    attributeId;
                    BYTE        command;
                    BYTE        data;
                    BYTE        dataLength;
                    //BYTE        dataType;
                    BYTE        frameHeader;
                    BYTE        sequenceNumber;
                    BYTE        transaction;
                    BYTE        transByte;

                    currentPrimitive = NO_PRIMITIVE;
                    frameHeader = APLGet();

                    switch (params.APSDE_DATA_indication.DstEndpoint)
                    {
                        case EP_ZDO:
                            ConsolePutROMString( (ROM char *)"  Receiving ZDO cluster " );
                            PrintChar( params.APSDE_DATA_indication.ClusterId );
                            ConsolePutROMString( (ROM char *)"\r\n" );

                            // Put code here to handle any ZDO responses that we requested
                            if ((frameHeader & APL_FRAME_TYPE_MASK) == APL_FRAME_TYPE_MSG)
                            {
                                frameHeader &= APL_FRAME_COUNT_MASK;
                                for (transaction=0; transaction<frameHeader; transaction++)
                                {
                                    sequenceNumber          = APLGet();
                                    dataLength              = APLGet();
                                    transByte               = 1;    // Account for status byte

                                    switch( params.APSDE_DATA_indication.ClusterId )
                                    {

                                        // ********************************************************
                                        // Put a case here to handle each ZDO response that we requested.
                                        // ********************************************************

                                        case NWK_ADDR_rsp:
                                            if (APLGet() == SUCCESS)
                                            {
                                                ConsolePutROMString( (ROM char *)"  Receiving NWK_ADDR_rsp.\r\n" );

                                                // Skip over the IEEE address of the responder.
                                                for (data=0; data<8; data++)
                                                {
                                                    APLGet();
                                                    transByte++;
                                                }
                                                destinationAddress.byte.LSB = APLGet();
                                                destinationAddress.byte.MSB = APLGet();
                                                transByte += 2;
                                                myStatusFlags.bits.bDestinationAddressKnown = 1;
                                            }
                                            break;
                                        default:
                                            break;
                                    }

                                    // Read out the rest of the MSG in case there is another transaction.
                                    for (; transByte<dataLength; transByte++)
                                    {
                                        APLGet();
                                    }
                                }
                            }
                            break;

                        // ************************************************************************
                        // Place a case for each user defined endpoint.
                        // ************************************************************************
                        case EP_LIGHT:
                            if ((frameHeader & APL_FRAME_TYPE_MASK) == APL_FRAME_TYPE_KVP)
                            {
                                frameHeader &= APL_FRAME_COUNT_MASK;
                                for (transaction=0; transaction<frameHeader; transaction++)
                                {
                                    sequenceNumber          = APLGet();
                                    command                 = APLGet();
                                    attributeId.byte.LSB    = APLGet();
                                    attributeId.byte.MSB    = APLGet();

                                    //dataType = command & APL_FRAME_DATA_TYPE_MASK;
                                    command &= APL_FRAME_COMMAND_MASK;

                                    if ((params.APSDE_DATA_indication.ClusterId == OnOffSRC_CLUSTER) &&
                                        (attributeId.Val == OnOffSRC_OnOff))
                                    {
                                        if ((command == APL_FRAME_COMMAND_SET) ||
                                            (command == APL_FRAME_COMMAND_SETACK))
                                        {
                                            // Prepare a response in case it is needed.
                                            TxBuffer[TxData++] = APL_FRAME_TYPE_KVP | 1;    // KVP, 1 transaction
                                            TxBuffer[TxData++] = sequenceNumber;
                                            TxBuffer[TxData++] = APL_FRAME_COMMAND_SET_RES | (APL_FRAME_DATA_TYPE_UINT8 << 4);
                                            TxBuffer[TxData++] = attributeId.byte.LSB;
                                            TxBuffer[TxData++] = attributeId.byte.MSB;

                                            // Data type for this attibute must be APL_FRAME_DATA_TYPE_UINT8
                                            data = APLGet();
                                            switch (data)
                                            {
                                                case LIGHT_OFF:
                                                    ConsolePutROMString( (ROM char *)" Turning light off.\r\n" );
                                                    LEVEL_SENSOR_LED = 0;
                                                    TxBuffer[TxData++] = SUCCESS;
                                                    break;
                                                case LIGHT_ON:
                                                    ConsolePutROMString( (ROM char *)" Turning light on.\r\n" );
                                                    LEVEL_SENSOR_LED = 1;
                                                    TxBuffer[TxData++] = SUCCESS;
                                                    break;
                                                case LIGHT_TOGGLE:
                                                    ConsolePutROMString( (ROM char *)" Toggling light.\r\n" );
                                                    LEVEL_SENSOR_LED ^= 1;
                                                    TxBuffer[TxData++] = SUCCESS;
                                                    break;
                                                default:
                                                    PrintChar( data );
                                                    ConsolePutROMString( (ROM char *)" Invalid light message.\r\n" );
                                                    TxBuffer[TxData++] = KVP_INVALID_ATTRIBUTE_DATA;
                                                    break;
                                            }
                                        }
                                        if (command == APL_FRAME_COMMAND_SETACK)
                                        {
                                            // Send back an application level acknowledge.
                                            ZigBeeBlockTx();

                                            // Take care here that parameters are not overwritten before they are used.
                                            // We can use the data byte as a temporary variable.
                                            params.APSDE_DATA_request.DstAddrMode = params.APSDE_DATA_indication.SrcAddrMode;
                                            params.APSDE_DATA_request.DstEndpoint = params.APSDE_DATA_indication.SrcEndpoint;
                                            params.APSDE_DATA_request.DstAddress.ShortAddr = params.APSDE_DATA_indication.SrcAddress.ShortAddr;

                                            //params.APSDE_DATA_request.asduLength; TxData
                                            //params.APSDE_DATA_request.ProfileId; unchanged
                                            params.APSDE_DATA_request.RadiusCounter = DEFAULT_RADIUS;
                                            params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_ENABLE;
											#ifdef I_SUPPORT_SECURITY
												params.APSDE_DATA_request.TxOptions.Val = 1;
											#else											                                            
                                            	params.APSDE_DATA_request.TxOptions.Val = 0;
											#endif                                            
                                            params.APSDE_DATA_request.SrcEndpoint = EP_LIGHT;
                                            //params.APSDE_DATA_request.ClusterId; unchanged

                                            currentPrimitive = APSDE_DATA_request;
                                        }
                                        else
                                        {
                                            // We are not sending an acknowledge, so reset the transmit message pointer.
                                            TxData = TX_DATA_START;
                                        }
                                    }
                                    // TODO read to the end of the transaction.
                                } // each transaction
                            } // frame type
                            break;
                          
                        default:               
                			break;
                	}

                    APLDiscardRx();
                }
                break;

            case APSDE_DATA_confirm:
                if (params.APSDE_DATA_confirm.Status)
                {
                    ConsolePutROMString( (ROM char *)"Error " );
                    PrintChar( params.APSDE_DATA_confirm.Status );
                    ConsolePutROMString( (ROM char *)" sending message.\r\n" );
                }
                else
                {
                    ConsolePutROMString( (ROM char *)" Message sent successfully.\r\n" );
                }
                currentPrimitive = NO_PRIMITIVE;
                break;

            case NO_PRIMITIVE:
                if (!ZigBeeStatus.flags.bits.bNetworkJoined)
                {
                    if (!ZigBeeStatus.flags.bits.bTryingToJoinNetwork)
                    {
                        if (ZigBeeStatus.flags.bits.bTryOrphanJoin)
                        {
                            ConsolePutROMString( (ROM char *)"Trying to join network as an orphan...\r\n" );
                            params.NLME_JOIN_request.JoinAsRouter           = FALSE;
                            params.NLME_JOIN_request.RejoinNetwork          = TRUE;
                            params.NLME_JOIN_request.PowerSource            = NOT_MAINS_POWERED;
                            params.NLME_JOIN_request.RxOnWhenIdle           = FALSE;
                            params.NLME_JOIN_request.MACSecurity            = FALSE;
                            params.NLME_JOIN_request.ScanDuration           = 8;
                            params.NLME_JOIN_request.ScanChannels.Val       = ALLOWED_CHANNELS;
                            currentPrimitive = NLME_JOIN_request;
                        }
                        else
                        {
                            ConsolePutROMString( (ROM char *)"Trying to join network as a new device...\r\n" );
                            params.NLME_NETWORK_DISCOVERY_request.ScanDuration          = 6;
                            params.NLME_NETWORK_DISCOVERY_request.ScanChannels.Val      = ALLOWED_CHANNELS;
                            currentPrimitive = NLME_NETWORK_DISCOVERY_request;
                        }
                    }
                }
                else
                {
                    // See if I can do my own internal tasks.  We don't want to try to send a message
                    // if we just asked for one.
                    if (ZigBeeStatus.flags.bits.bDataRequestComplete && ZigBeeReady())
                    {

                        // ************************************************************************
                        // Place all processes that can send messages here.  Be sure to call
                        // ZigBeeBlockTx() when currentPrimitive is set to APSDE_DATA_request.
                        // ************************************************************************
                        if ( myStatusFlags.bits.bMoveSensorButtonPressed)
                        {
                            // Send a light toggle message to the other node.
                            myStatusFlags.bits.bMoveSensorButtonPressed = FALSE;

							BLINK_LED(MOVE_SENSOR_LED);							

							// envia a mensagem para ligar/desligar o led
							RFDSendMessage(MoveSensor_Activated, 0x00);                
                        }
                        else if (myStatusFlags.bits.bLevelSensorButtonPressed)
                        {
                        	// Envia mensagem indicando que o sensor de nível foi acionado

                            myStatusFlags.bits.bLevelSensorButtonPressed = FALSE;

							BLINK_LED(LEVEL_SENSOR_LED);

							RFDSendMessage(LevelSensor_Activated, 0x00);                                
                        }

                        // We've processed any key press, so re-enable interrupts.
                        INTCONbits.RBIE = 1;
                    }

                    // If we don't have to execute a primitive, see if we need to request data from
                    // our parent, or if we can go to sleep.
                    if (currentPrimitive == NO_PRIMITIVE)
                    {
                        if (!ZigBeeStatus.flags.bits.bDataRequestComplete)
                        {
                            // We have not received all data from our parent.  If we are not waiting
                            // for an answer from a data request, send a data request.
                            if (!ZigBeeStatus.flags.bits.bRequestingData)
                            {
                                if (ZigBeeReady())
                                {
                                    // Our parent still may have data for us.
                                    params.NLME_SYNC_request.Track = FALSE;
                                    currentPrimitive = NLME_SYNC_request;
                                    ConsolePutROMString( (ROM char *)"Requesting data...\r\n" );
                                }
                            }
                        }
                        else
                        {
                            if (!ZigBeeStatus.flags.bits.bHasBackgroundTasks && myProcessesAreDone())
                            {
                                // We do not have a primitive to execute, we've extracted all messages
                                // that our parent has for us, the stack has no background tasks,
                                // and all application-specific processes are complete.  Now we can
                                // go to sleep.  Make sure that the UART is finished, turn off the transceiver,
                                // and make sure that we wakeup from key press.
                                if(APLDisable() == TRUE)
                                {
	                                ConsolePutROMString( (ROM char *)"Going to sleep...\r\n" );
    	                            while (!ConsoleIsPutReady());
                                	APLDisable();
    	                            INTCONbits.RBIE = 1;
	                                SLEEP();
        	                        NOP();

	                                // We just woke up from sleep. Turn on the transceiver and
	                                // request data from our parent.
	                                APLEnable();
	                                params.NLME_SYNC_request.Track = FALSE;
	                                currentPrimitive = NLME_SYNC_request;
	                                ConsolePutROMString( (ROM char *)"Requesting data...\r\n" );
								}
                            }
                        }
                    }
                }
                break;

            default:
                PrintChar( currentPrimitive );
                ConsolePutROMString( (ROM char *)" Unhandled primitive.\r\n" );
                currentPrimitive = NO_PRIMITIVE;
        }

        // *********************************************************************
        // Place any non-ZigBee related processing here.  Be sure that the code
        // will loop back and execute ZigBeeTasks() in a timely manner.
        // *********************************************************************

    }
}

/*******************************************************************************
myProcessesAreDone

This routine should contain any tests that are required by the application to
confirm that it can go to sleep.  If the application can go to sleep, this
routine should return TRUE.  If the application is still busy, this routine
should return FALSE.
*******************************************************************************/

BOOL myProcessesAreDone( void )
{
    return (myStatusFlags.bits.bLevelSensorButtonPressed==FALSE) && (myStatusFlags.bits.bMoveSensorButtonPressed==FALSE);
}

/*******************************************************************************
HardwareInit

All port directioning and SPI must be initialized before calling ZigBeeInit().

For demonstration purposes, required signals are configured individually.
*******************************************************************************/
void HardwareInit(void)
{

    //-------------------------------------------------------------------------
    // This section is required to initialize the PICDEM Z for the CC2420
    // and the ZigBee Stack.
    //-------------------------------------------------------------------------

    #ifdef USE_EXTERNAL_NVM
        EEPROM_nCS          = 1;
        EEPROM_nCS_TRIS     = 0;
    #endif

    #if defined(USE_EXTERNAL_NVM) && !defined(EE_AND_RF_SHARE_SPI)
        RF_SPIInit();
        EE_SPIInit();
    #else
        SPIInit();
    #endif

	// Inicializa o Transceiver
    TransceiverInit();

    #if defined(USE_EXTERNAL_NVM) && !defined(EE_AND_RF_SHARE_SPI)
        // Initialize the SPI1 pins and directions
        LATCbits.LATC3               = 0;    // SCK
        LATCbits.LATC5               = 1;    // SDO
        TRISCbits.TRISC3             = 0;    // SCK
        TRISCbits.TRISC4             = 1;    // SDI
        TRISCbits.TRISC5             = 0;    // SDO
    
        // Initialize the SPI2 pins and directions
        LATDbits.LATD6               = 0;    // SCK
        LATDbits.LATD4               = 1;    // SDO
        TRISDbits.TRISD6             = 0;    // SCK
        TRISDbits.TRISD5             = 1;    // SDI
        TRISDbits.TRISD4             = 0;    // SDO
    
        RF_SSPSTAT_REG = 0x40;
        RF_SSPCON1_REG = 0x21;
        EE_SSPSTAT_REG = 0x40;
        EE_SSPCON1_REG = 0x21;
    #else
        // Initialize the SPI pins and directions
        LATCbits.LATC3               = 0;    // SCK
        LATCbits.LATC5               = 1;    // SDO
        TRISCbits.TRISC3             = 0;    // SCK
        TRISCbits.TRISC4             = 1;    // SDI
        TRISCbits.TRISC5             = 0;    // SDO
    
        SSPSTAT_REG = 0x40;
        SSPCON1_REG = 0x20;
    #endif

    //-------------------------------------------------------------------------
    // This section is required for application-specific hardware
    // initialization.
    //-------------------------------------------------------------------------

    // D1 and D2 are on RA0 and RA1 respectively, and CS of the TC77 is on RA2.
    // Make PORTA digital I/O.
    ADCON1 = 0x0F;
    
    // Deselect the TC77 temperature sensor (RA2)
    LATA = 0x04;
    
    // Make RA0, RA1, RA2 and RA4 outputs.
    TRISA = 0xE0;

    // Clear the RBIF flag (INTCONbits.RBIF)
    INTCONbits.RBIF = 0;

    // Enable PORTB pull-ups (INTCON2bits.RBPU)
    INTCON2bits.RBPU = 0;

    // Make the PORTB switch connections inputs.

    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
}

void TransceiverInit()
{
	// Start with MRF24J40 disabled and not selected
    PHY_CS              = 1;
    PHY_RESETn          = 1;

    // Set the directioning for the MRF24J40 pin connections.
    PHY_CS_TRIS         = 0;
    PHY_RESETn_TRIS     = 0;

    // Initialize the interrupt.
    INTCON2bits.INTEDG0 = 0;
}

/*******************************************************************************
User Interrupt Handler

The stack uses some interrupts for its internal processing.  Once it is done
checking for its interrupts, the stack calls this function to allow for any
additional interrupt processing.
*******************************************************************************/

void UserInterruptHandler(void)
{

    // *************************************************************************
    // Place any application-specific interrupt processing here
    // *************************************************************************

    // Is this a interrupt-on-change interrupt?
    if ( INTCONbits.RBIF == 1 )
    {
        // Record which button was pressed so the main() loop can
        // handle it
        if (LEVEL_SENSOR == 0)
		{
            myStatusFlags.bits.bLevelSensorButtonPressed = TRUE;
			ConsolePutROMString( (ROM char *)"O botao LEVEL_SENSOR foi pressionado!\r\n" );
		}

        if (MOVE_SENSOR == 0)
		{
            myStatusFlags.bits.bMoveSensorButtonPressed = TRUE;
			ConsolePutROMString( (ROM char *)"O botao MOVE_SENSOR foi pressionado!\r\n" );
		}

        // Disable further RBIF until we process it
        INTCONbits.RBIE = 0;

        // Clear mis-match condition and reset the interrupt flag
        LATB = PORTB;

        INTCONbits.RBIF = 0;
    }
}


/* Função para Envio de uma mensagem */

void RFDSendMessage(unsigned short attributeID, unsigned char value)
{
	// Send a light toggle message to the other node.
    ZigBeeBlockTx();

    TxBuffer[TxData++] = APL_FRAME_TYPE_KVP | 1;    // KVP, 1 transaction
    TxBuffer[TxData++] = APLGetTransId();
    TxBuffer[TxData++] = APL_FRAME_COMMAND_SET | (APL_FRAME_DATA_TYPE_UINT8 << 4);
    TxBuffer[TxData++] = ATTR_ID_LSB(attributeID);	// Attribute ID LSB
    TxBuffer[TxData++] = ATTR_ID_MSB(attributeID);  		// Attribute ID MSB
    TxBuffer[TxData++] = value;

    params.APSDE_DATA_request.DstAddrMode = APS_ADDRESS_16_BIT;
    params.APSDE_DATA_request.DstEndpoint = EP_LIGHT;
    params.APSDE_DATA_request.DstAddress.ShortAddr = destinationAddress;

    //params.APSDE_DATA_request.asduLength; TxData
    params.APSDE_DATA_request.ProfileId.Val = BABY_PROFILE_ID;
    params.APSDE_DATA_request.RadiusCounter = DEFAULT_RADIUS;
    params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_ENABLE;
						    
	// Sem suporte a segurança                        
    params.APSDE_DATA_request.TxOptions.Val = 0; 
                          
    params.APSDE_DATA_request.SrcEndpoint = EP_LIGHT;
    params.APSDE_DATA_request.ClusterId = BabyControl_CLUSTER;

    ConsolePutROMString( (ROM char *)" Trying to send light switch message.\r\n" );

    currentPrimitive = APSDE_DATA_request;
}
