/*
    Microchip ZigBee Stack

    Demo Coordinator

    This demonstration shows how a ZigBee coordinator can be set up.  This demo allows
    the PICDEM Z Demostration Board to act as either a "Switching Load Controller"
    (e.g. a light) or a "Switching Remote Control" (e.g. a switch) as defined by
    the Home Controls, Lighting profile.  It is designed to interact with a
    second PICDEM Z programmed with the Demo RFD project.

    To give the PICDEM Z "switch" capability, uncomment the I_AM_SWITCH definition
    below.  To give the PICDEM Z "light" capability, uncomment the I_AM_LIGHT
    definition below.  The PICDEM Z may have both capabilities enabled.  Be sure
    that the corresponding Demo RFD device is programmed with complementary
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
 * FileName:        Coordinator.c
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

//******************************************************************************
// Header Files
//******************************************************************************

// Include the main ZigBee header file.
#include "zAPL.h"
#ifdef I_SUPPORT_SECURITY
#include "zSecurity.h"
#endif

// If you are going to send data to a terminal, include this file.
#include "console.h"


#ifdef I_SUPPORT_SECURITY
	#ifdef USE_EXTERNAL_NVM
		extern WORD trustCenterLongAddr;
		extern NETWORK_KEY_INFO plainSecurityKey[2];
	#else
		extern ROM LONG_ADDR trustCenterLongAddr;
	#endif
#endif

//******************************************************************************
// Configuration Bits
//******************************************************************************

#if defined(MCHP_C18)
    #if defined(__18F4620)
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
    #elif defined(__18F87J10)
        #pragma romdata CONFIG1L = 0x1FFF8
        const rom unsigned char config1L = 0b10111110;      
    
        #pragma romdata CONFIG1H = 0x1FFF9
        const rom unsigned char config1H = 0b11111111;      
    
        #pragma romdata CONFIG2L = 0x1FFFA
        const rom unsigned char config2L = 0b11111101;      
    
        #pragma romdata CONFIG2H = 0x1FFFB
        const rom unsigned char config2H = 0b11111010;      
    
        #pragma romdata CONFIG3L = 0x1FFFC
        const rom unsigned char config3L = 0b11111111;

        #pragma romdata CONFIG3H = 0x1FFFD
        const rom unsigned char config3H = 0b11111111;
        
    #endif
    #pragma romdata

#else
    #error Other compilers are not yet supported.
#endif


//******************************************************************************
// Compilation Configuration
//******************************************************************************

//#define USE_BINDINGS
#define I_AM_LIGHT
#define I_AM_SWITCH

//******************************************************************************
// Constants
//******************************************************************************

#if defined (__18F4620)
    #define SWITCH_PRESSED      0
#else
    #define SWITCH_PRESSED      0
#endif

#define BIND_SWITCH                 PORTBbits.RB5
#define LIGHT_SWITCH                PORTBbits.RB4

#define MOVE_SENSOR_LED             LATAbits.LATA0
#define LEVEL_SENSOR_LED         	LATAbits.LATA1

#define BIND_STATE_BOUND            0
#define BIND_STATE_TOGGLE           1
#define BIND_STATE_UNBOUND          1
#define BIND_WAIT_DURATION          (5*ONE_SECOND)

#define ON							1
#define OFF							0

#define LIGHT_OFF                   0x00
#define LIGHT_ON                    0xFF
#define LIGHT_TOGGLE                0xF0

#define DELAY() {int i,j; for(i=0; i < 100; i++) for(j=0; j < 1000;j++);}

#define BLINK_LED(led) { led = OFF; DELAY(); led = ON; DELAY(); led = OFF; }


//******************************************************************************
// Function Prototypes
//******************************************************************************

void HardwareInit( void );

//******************************************************************************
// Application Variables
//******************************************************************************

ZIGBEE_PRIMITIVE    currentPrimitive;
SHORT_ADDR          destinationAddress;
#ifdef I_SUPPORT_SECURITY
	extern KEY_VAL	KeyVal;
#endif

static union
{
    struct
    {
        BYTE    bBroadcastSwitchToggled    : 1;
        BYTE    bLightSwitchToggled        : 1;
        BYTE    bTryingToBind              : 1;
        BYTE    bIsBound                   : 1;
        BYTE    bDestinationAddressKnown   : 1;
    } bits;
    BYTE Val;
} myStatusFlags;
#define STATUS_FLAGS_INIT       0x00
#define TOGGLE_BOUND_FLAG       0x08
#define bBindSwitchToggled      bBroadcastSwitchToggled

//******************************************************************************
//******************************************************************************
//******************************************************************************

void main(void)
{    
    CLRWDT();
    ENABLE_WDT();

    currentPrimitive = NO_PRIMITIVE;

    // If you are going to send data to a terminal, initialize the UART.
    ConsoleInit();

	ConsolePutROMString( (ROM char *)"Universidade Paulista - UNIP\r\n" );
	ConsolePutROMString( (ROM char *)"Daniel Gonçalves\r\n" );
	ConsolePutROMString( (ROM char *)"Projeto: Baba Eletronica\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"\r\n\r\n\r\n*************************************\r\n" );
    ConsolePutROMString( (ROM char *)"Microchip ZigBee(TM) Stack - v1.0-3.8\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"ZigBee Coordinator\r\n\r\n" );
    ConsolePutROMString( (ROM char *)"Transceiver-MRF24J40\r\n\r\n" );

    // Initialize the hardware - must be done before initializing ZigBee.
    HardwareInit();
    ConsolePutROMString( (ROM char *)"Hardware initialized\r\n" );
   
    // Initialize the ZigBee Stack.
    ZigBeeInit();
    ConsolePutROMString( (ROM char *)"ZigBee initialized\r\n\r\n" );

    if (PORTBbits.RB4 == 0)
    {
        NWKClearNeighborTable();
        ConsolePutROMString( (ROM char *)"Neighbor table \r\n" ); 
    }
    
    // *************************************************************************
    // Perform any other initialization here
    // *************************************************************************
    myStatusFlags.Val = STATUS_FLAGS_INIT;
    destinationAddress.Val = 0x796F;    // Default to first RFD

    MOVE_SENSOR_LED = !myStatusFlags.bits.bIsBound;
    LATAbits.LATA4 = 0;

    // Enable interrupts to get everything going.
    INTCONbits.RBIE = 1;
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;

    while (1)
    {
        CLRWDT();
        ZigBeeTasks( &currentPrimitive );

        switch (currentPrimitive)
        {
            case NLME_NETWORK_FORMATION_confirm:
                if (!params.NLME_NETWORK_FORMATION_confirm.Status)
                {
                    ConsolePutROMString( (ROM char *)"PAN " );
                    PrintChar( macPIB.macPANId.byte.MSB );
                    PrintChar( macPIB.macPANId.byte.LSB );
                    ConsolePutROMString( (ROM char *)" started successfully.\r\n" );
                    ConsolePutROMString( (ROM char *)"Turning on joining...\r\n" );
                    params.NLME_PERMIT_JOINING_request.PermitDuration = 0xFF;   // No Timeout
                    currentPrimitive = NLME_PERMIT_JOINING_request;
                }
                else
                {
                    PrintChar( params.NLME_NETWORK_FORMATION_confirm.Status );
                    ConsolePutROMString( (ROM char *)" Error forming network.  Trying again...\r\n" );
                    currentPrimitive = NO_PRIMITIVE;
                }
                break;

            case NLME_PERMIT_JOINING_confirm:
                if (!params.NLME_PERMIT_JOINING_confirm.Status)
                {
                    ConsolePutROMString( (ROM char *)"Joining status changed.\r\n" );
                    currentPrimitive = NO_PRIMITIVE;
                }
                else
                {
                    PrintChar( params.NLME_PERMIT_JOINING_confirm.Status );
                    ConsolePutROMString( (ROM char *)" Join status unchanged.\r\n" );
                    currentPrimitive = NO_PRIMITIVE;
                }
                break;

            case NLME_JOIN_indication:
                ConsolePutROMString( (ROM char *)"Node " );
                PrintChar( params.NLME_JOIN_indication.ShortAddress.byte.MSB );
                PrintChar( params.NLME_JOIN_indication.ShortAddress.byte.LSB );
                ConsolePutROMString( (ROM char *)" just joined.\r\n" );
                
				#ifdef I_SUPPORT_SECURITY

					#ifdef I_AM_TRUST_CENTER
  		 			{
			  		  	BOOL AllowJoin = TRUE;
						// decide if you allow this device to join
			  		  	if( !AllowJoin )
  		  				{
			  		  		// no need to set deviceAddress, since it is overlap with NLME_JOIN_indication
		  					//params.NLME_LEAVE_request.DeviceAddress = params.NLME_JOIN_indication.ExtendedAddress;
					  		params.NLME_LEAVE_request.RemoveChildren = TRUE;
  		  					currentPrimitive = NLME_LEAVE_request;
							break;
  		  				}
  		  	
						#ifdef I_SUPPORT_SECURITY_SPEC 	
						if( params.NLME_JOIN_indication.secureJoin )
						{
							BYTE i;
			//				ConsolePutROMString((ROM char *)"secure join\r\n");
							for(i = 0; i < 16; i++)
							{
								KeyVal.v[i] = 0;
							}
							params.APSME_TRANSPORT_KEY_request.Key = &KeyVal;
							params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = 0;
				
						} 
						else 
						{
							BYTE i;
							GetNwkActiveKeyNumber(&i);
							#ifdef USE_EXTERNAL_NVM
								currentNetworkKeyInfo = plainSecurityKey[i-1];
							#else
								GetNwkKeyInfo(&currentNetworkKeyInfo, (ROM void *)&(networkKeyInfo[i-1]));
							#endif
							params.APSME_TRANSPORT_KEY_request.Key = &(currentNetworkKeyInfo.NetKey);
							params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = currentNetworkKeyInfo.SeqNumber.v[0];

						}
						#else
							#ifdef PRECONFIGURE_KEY
							{
								BYTE i;
			//					ConsolePutROMString((ROM char *)"secure join\r\n");
								for(i = 0; i < 16; i++)
								{
									KeyVal.v[i] = 0;
								}
								params.APSME_TRANSPORT_KEY_request.Key = &KeyVal;
								params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = 0;
								params.APSME_TRANSPORT_KEY_request._UseSecurity = TRUE;
							}
							#else
							{
								BYTE i;
								GetNwkActiveKeyNumber(&i);
								#ifdef USE_EXTERNAL_NVM
									currentNetworkKeyInfo = plainSecurityKey[i-1];
								#else
									GetNwkKeyInfo(&currentNetworkKeyInfo, (ROM void *)&(networkKeyInfo[i-1]));
								#endif
								params.APSME_TRANSPORT_KEY_request.Key = &(currentNetworkKeyInfo.NetKey);
								params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.KeySeqNumber = currentNetworkKeyInfo.SeqNumber.v[0];
								params.APSME_TRANSPORT_KEY_request._UseSecurity = FALSE;
							}
							#endif
		
						#endif
						params.APSME_TRANSPORT_KEY_request.KeyType = ID_NetworkKey;
						params.APSME_TRANSPORT_KEY_request.DestinationAddress = params.NLME_JOIN_indication.ExtendedAddress;
						params.APSME_TRANSPORT_KEY_request.TransportKeyData.NetworkKey.UseParent = FALSE;
						currentPrimitive = APSME_TRANSPORT_KEY_request;
					}
					#else
						params.APSME_UPDATE_DEVICE_request.Status = (params.NLME_JOIN_indication.secureJoin ) ? 0x00 : 0x01;
					  	params.APSME_UPDATE_DEVICE_request.DeviceShortAddress = params.NLME_JOIN_indication.ShortAddress;
					  	params.APSME_UPDATE_DEVICE_request.DeviceAddress = params.NLME_JOIN_indication.ExtendedAddress;
			  			GetTrustCenterAddress(&params.APSME_UPDATE_DEVICE_request.DestAddress);
					  	currentPrimitive = APSME_UPDATE_DEVICE_request;
					#endif
				#else
        			currentPrimitive = NO_PRIMITIVE;
				#endif
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
				#ifdef I_SUPPORT_SECURITY
		 			{
						SHORT_ADDR		LeavingChildShortAddress;
						if( !APSFromLongToShort(&params.NLME_LEAVE_indication.DeviceAddress) ) 
						{
							currentPrimitive = NO_PRIMITIVE;
							break;
						}
						LeavingChildShortAddress = currentAPSAddress.shortAddr;

						#ifdef I_AM_TRUST_CENTER
							params.APSME_UPDATE_DEVICE_indication.Status = 0x02;
							params.APSME_UPDATE_DEVICE_indication.DeviceAddress = params.NLME_LEAVE_indication.DeviceAddress;
							GetMACAddress(&params.APSME_UPDATE_DEVICE_indication.SrcAddress);
							params.APSME_UPDATE_DEVICE_indication.DeviceShortAddress = LeavingChildShortAddress;
							currentPrimitive = APSME_UPDATE_DEVICE_indication;
							break;
						#else
							params.APSME_UPDATE_DEVICE_request.Status = 0x02;
							// omit to do this since the address overlap
							//params.APSME_UPDATE_DEVICE_request.DeviceAddress = params.NLME_LEAVE_indication.DeviceAddress;
							GetTrustCenterAddress(&params.APSME_UPDATE_DEVICE_request.DestAddress);
							params.APSME_UPDATE_DEVICE_request.DeviceShortAddress = LeavingChildShortAddress;
							currentPrimitive = APSME_UPDATE_DEVICE_request;
							break;
						#endif
					}
				#else			
                	currentPrimitive = NO_PRIMITIVE;
				#endif
				
                break;

            case NLME_RESET_confirm:
                ConsolePutROMString( (ROM char *)"ZigBee Stack has been reset.\r\n" );
                currentPrimitive = NO_PRIMITIVE;
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

									if(params.APSDE_DATA_indication.ClusterId == BabyControl_CLUSTER)
									{
										if(command == APL_FRAME_COMMAND_SET || command == APL_FRAME_COMMAND_SETACK)
										{
											switch(attributeId.Val)
											{
												case LevelSensor_Activated:
													{
														BLINK_LED(LEVEL_SENSOR_LED);
														ConsolePutROMString((ROM char *) "A crianca levantou!!\r\n");
													}break;
												case MoveSensor_Activated:
													{
														BLINK_LED(MOVE_SENSOR_LED)
														ConsolePutROMString((ROM char *) "A crianca se mexeu!!\r\n");
													}break;
											}//switch
										}// if
									}// if

									/*
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
                                                    ConsolePutROMString( (ROM char *)"Turning light off.\r\n" );
                                                    LEVEL_SENSOR_LED = 0;
                                                    TxBuffer[TxData++] = SUCCESS;
                                                    break;
                                                case LIGHT_ON:
                                                    ConsolePutROMString( (ROM char *)"Turning light on.\r\n" );
                                                    LEVEL_SENSOR_LED = 1;
                                                    TxBuffer[TxData++] = SUCCESS;
                                                    break;
                                                case LIGHT_TOGGLE:
                                                    ConsolePutROMString( (ROM char *)"Toggling light.\r\n" );
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
                                    }*/
                                    // TODO read to the end of the transaction.
                                } // each transaction
                            } // frame type
                            break;
                        default:
                            // If the command type was something that requested an acknowledge, we could send back
                            // KVP_INVALID_ENDPOINT here.
                            break;
                    }
                    APLDiscardRx();
                }
                break;

            case NO_PRIMITIVE:
                if (!ZigBeeStatus.flags.bits.bNetworkFormed)
                {
                    if (!ZigBeeStatus.flags.bits.bTryingToFormNetwork)
                    {
                        ConsolePutROMString( (ROM char *)"Trying to start network...\r\n" );
                        params.NLME_NETWORK_FORMATION_request.ScanDuration          = 8;
                        params.NLME_NETWORK_FORMATION_request.ScanChannels.Val      = ALLOWED_CHANNELS;
                        params.NLME_NETWORK_FORMATION_request.PANId.Val             = 0xFFFF;
                        params.NLME_NETWORK_FORMATION_request.BeaconOrder           = MAC_PIB_macBeaconOrder;
                        params.NLME_NETWORK_FORMATION_request.SuperframeOrder       = MAC_PIB_macSuperframeOrder;
                        params.NLME_NETWORK_FORMATION_request.BatteryLifeExtension  = MAC_PIB_macBattLifeExt;
                        currentPrimitive = NLME_NETWORK_FORMATION_request;
                    }
                }
                else
                {
 
                    if (ZigBeeReady())
                    {

                        // ************************************************************************
                        // Place all processes that can send messages here.  Be sure to call
                        // ZigBeeBlockTx() when currentPrimitive is set to APSDE_DATA_request.
                        // ************************************************************************

                        #ifdef I_AM_SWITCH
                        if ( myStatusFlags.bits.bLightSwitchToggled )
                        {
                            // Send a light toggle message to the other node.
                            myStatusFlags.bits.bLightSwitchToggled = FALSE;
                            ZigBeeBlockTx();

                            TxBuffer[TxData++] = APL_FRAME_TYPE_KVP | 1;    // KVP, 1 transaction
                            TxBuffer[TxData++] = APLGetTransId();
                            TxBuffer[TxData++] = APL_FRAME_COMMAND_SET | (APL_FRAME_DATA_TYPE_UINT8 << 4);
                            TxBuffer[TxData++] = OnOffSRC_OnOff & 0xFF;         // Attribute ID LSB
                            TxBuffer[TxData++] = (OnOffSRC_OnOff >> 8) & 0xFF;  // Attribute ID MSB
                            TxBuffer[TxData++] = LIGHT_TOGGLE;

                            #ifdef USE_BINDINGS
                                // We are sending indirect
                                params.APSDE_DATA_request.DstAddrMode = APS_ADDRESS_NOT_PRESENT;
                            #else
                                // We are sending direct
                                params.APSDE_DATA_request.DstAddrMode = APS_ADDRESS_16_BIT;
                                params.APSDE_DATA_request.DstEndpoint = EP_LIGHT;
                                params.APSDE_DATA_request.DstAddress.ShortAddr = destinationAddress;
                            #endif

                            //params.APSDE_DATA_request.asduLength; TxData
                            params.APSDE_DATA_request.ProfileId.Val = MY_PROFILE_ID;
                            params.APSDE_DATA_request.RadiusCounter = DEFAULT_RADIUS;
                            params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_ENABLE;
							#ifdef I_SUPPORT_SECURITY
								params.APSDE_DATA_request.TxOptions.Val = 1;
							#else							                            
                            	params.APSDE_DATA_request.TxOptions.Val = 0;
							#endif                            
                            params.APSDE_DATA_request.SrcEndpoint = EP_LIGHT;
                            params.APSDE_DATA_request.ClusterId = OnOffSRC_CLUSTER;

                            ConsolePutROMString( (ROM char *)" Trying to send light switch message.\r\n" );

                            currentPrimitive = APSDE_DATA_request;
                        }
                        else
                        #endif
                        #ifdef USE_BINDINGS
                        if (myStatusFlags.bits.bBindSwitchToggled)
                        {
                            myStatusFlags.bits.bBindSwitchToggled = FALSE;
                            if (myStatusFlags.bits.bTryingToBind)
                            {
                                ConsolePutROMString( (ROM char *)" End Device Binding already in progress.\r\n" );
                            }
                            else
                            {
                                myStatusFlags.bits.bTryingToBind = 1;

                                // Send down an end device bind primitive.
                                ConsolePutROMString( (ROM char *)" Trying to perform end device binding.\r\n" );
                                currentPrimitive = ZDO_END_DEVICE_BIND_req;

                                params.ZDO_END_DEVICE_BIND_req.ProfileID.Val = MY_PROFILE_ID;
                                #ifdef I_AM_LIGHT
                                    params.ZDO_END_DEVICE_BIND_req.NumInClusters = 1;
                                    if ((params.ZDO_END_DEVICE_BIND_req.InClusterList=SRAMalloc(1)) != NULL)
                                    {
                                        *params.ZDO_END_DEVICE_BIND_req.InClusterList = OnOffSRC_CLUSTER;
                                    }
                                    else
                                    {
                                        myStatusFlags.bits.bTryingToBind = 0;
                                        ConsolePutROMString( (ROM char *)" Could not send down ZDO_END_DEVICE_BIND_req.\r\n" );
                                        currentPrimitive = NO_PRIMITIVE;
                                    }
                                #else
                                    params.ZDO_END_DEVICE_BIND_req.NumInClusters = 0;
                                    params.ZDO_END_DEVICE_BIND_req.InClusterList = NULL;
                                #endif
                                #ifdef I_AM_SWITCH
                                    params.ZDO_END_DEVICE_BIND_req.NumOutClusters = 1;
                                    if ((params.ZDO_END_DEVICE_BIND_req.OutClusterList=SRAMalloc(1)) != NULL)
                                    {
                                        *params.ZDO_END_DEVICE_BIND_req.OutClusterList = OnOffSRC_CLUSTER;
                                    }
                                    else
                                    {
                                        myStatusFlags.bits.bTryingToBind = 0;
                                        ConsolePutROMString( (ROM char *)" Could not send down ZDO_END_DEVICE_BIND_req.\r\n" );
                                        currentPrimitive = NO_PRIMITIVE;
                                    }
                                #else
                                    params.ZDO_END_DEVICE_BIND_req.NumOutClusters = 0;
                                    params.ZDO_END_DEVICE_BIND_req.OutClusterList = NULL;
                                #endif
                                params.ZDO_END_DEVICE_BIND_req.LocalCoordinator = macPIB.macShortAddress;
                                params.ZDO_END_DEVICE_BIND_req.endpoint = EP_LIGHT;
                            }
                        }
                        #else
                        if (myStatusFlags.bits.bBroadcastSwitchToggled)
                        {
                            myStatusFlags.bits.bBroadcastSwitchToggled = FALSE;

                            // Send NWK_ADDR_req message
                            ZigBeeBlockTx();

                            TxBuffer[TxData++] = APL_FRAME_TYPE_MSG | 1;    // KVP, 1 transaction
                            TxBuffer[TxData++] = APLGetTransId();
                            TxBuffer[TxData++] = 10; // Transaction Length

                            // IEEEAddr
                            TxBuffer[TxData++] = 0x65;
                            TxBuffer[TxData++] = 0x00;
                            TxBuffer[TxData++] = 0x00;
                            TxBuffer[TxData++] = 0x00;
                            TxBuffer[TxData++] = 0x00;
                            TxBuffer[TxData++] = 0xa3;
                            TxBuffer[TxData++] = 0x04;
                            TxBuffer[TxData++] = 0x00;

                            // RequestType
                            TxBuffer[TxData++] = 0x00;

                            // StartIndex
                            TxBuffer[TxData++] = 0x00;

                            params.APSDE_DATA_request.DstAddrMode = APS_ADDRESS_16_BIT;
                            params.APSDE_DATA_request.DstEndpoint = EP_ZDO;
                            params.APSDE_DATA_request.DstAddress.ShortAddr.Val = 0xFFFF;

                            //params.APSDE_DATA_request.asduLength; TxData
                            params.APSDE_DATA_request.ProfileId.Val = ZDO_PROFILE_ID;
                            params.APSDE_DATA_request.RadiusCounter = DEFAULT_RADIUS;
                            params.APSDE_DATA_request.DiscoverRoute = ROUTE_DISCOVERY_ENABLE;
							#ifdef I_SUPPORT_SECURITY
								params.APSDE_DATA_request.TxOptions.Val = 1;
							#else							                            
                            	params.APSDE_DATA_request.TxOptions.Val = 0;
							#endif                            
                            params.APSDE_DATA_request.SrcEndpoint = EP_ZDO;
                            params.APSDE_DATA_request.ClusterId = NWK_ADDR_req;

                            ConsolePutROMString( (ROM char *)" Trying to send NWK_ADDR_req.\r\n" );

                            currentPrimitive = APSDE_DATA_request;
                        }
                        #endif

                        // Re-enable interrupts.
                        INTCONbits.RBIE = 1;
                    }
                }
                break;

            default:
                PrintChar( currentPrimitive );
                ConsolePutROMString( (ROM char *)" Unhandled primitive.\r\n" );
                currentPrimitive = NO_PRIMITIVE;
                break;
        }

        // *********************************************************************
        // Place any non-ZigBee related processing here.  Be sure that the code
        // will loop back and execute ZigBeeTasks() in a timely manner.
        // *********************************************************************

        #if !defined (__18F4620)
            if (PORTBbits.RB3 == SWITCH_PRESSED)
            {
                LATDbits.LATD7 = 1;
            }
            else
            {
                LATDbits.LATD7 = 0;
            }
        #endif
        
    }
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

    #if (RF_CHIP == MRF24J40)
        // Start with MRF24J40 disabled and not selected
        PHY_CS              = 1;
        PHY_RESETn          = 1;

        // Set the directioning for the MRF24J40 pin connections.
        PHY_CS_TRIS         = 0;
        PHY_RESETn_TRIS     = 0;

        // Initialize the interrupt.
        INTCON2bits.INTEDG0 = 0;
    #elif (RF_CHIP==UZ2400)
        // Start with UZ2400 disabled and not selected
        PHY_SEN             = 1;
        PHY_RESETn          = 1;

        // Set the directioning for the UZ2400 pin connections.
        PHY_SEN_TRIS        = 0;
        PHY_RESETn_TRIS     = 0;

        // Initialize the interrupt.
        INTCON2bits.INTEDG0 = 0;
    #elif (RF_CHIP==CC2420)
        // CC2420 I/O assignments with respect to PIC:
        //NOTE: User must make sure that pin is capable of correct digital operation.
        //      This may require modificaiton of which pins are digital and analog.
        //NOTE: The stack requires that the SPI interface be located on LATC3 (SCK),
        //      RC4 (SO), and LATC5 (SI).
        //NOTE: The appropriate config bit must be set such that FIFOP is the CCP2
        //      input pin. The stack uses the CCP2 interrupt.

        // Start with CC2420 disabled and not selected
        PHY_CSn             = 1;
        PHY_VREG_EN         = 0;
        PHY_RESETn          = 1;

        // Set the directioning for the CC2420 pin connections.
        PHY_FIFO_TRIS       = 1;    // FIFO      (Input)
        PHY_SFD_TRIS        = 1;    // SFD       (Input - Generates interrupt on falling edge)
        PHY_FIFOP_TRIS      = 1;    // FIFOP     (Input - Used to detect overflow, CCP2 interrupt)
        PHY_CSn_TRIS        = 0;    // CSn       (Output - to select CC2420 SPI slave)
        PHY_VREG_EN_TRIS    = 0;    // VREG_EN   (Output - to enable CC2420 voltage regulator)
        PHY_RESETn_TRIS     = 0;    // RESETn    (Output - to reset CC2420)
    #else
        #error Unknown transceiver selected
    #endif

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

    #if defined (__18F4620)
        // D1 and D2 are on RA0 and RA1 respectively, and CS of the TC77 is on RA2.
        // Make PORTA digital I/O.
        ADCON1 = 0x0F;
    
        // Deselect the TC77 temperature sensor (RA2)
        LATA = 0x04;
    
        // Make RA0, RA1, RA2 and RA4 outputs.
        TRISA = 0xE0;
    #endif

    // Clear the RBIF flag (INTCONbits.RBIF)
    INTCONbits.RBIF = 0;

    // Enable PORTB pull-ups (INTCON2bits.RBPU)
    INTCON2bits.RBPU = 0;

    // Make the PORTB switch connections inputs.
    #if !defined(__18F4620)
        TRISDbits.TRISD7 = 0;
        TRISBbits.TRISB3 = 1;
        TRISDbits.TRISD0 = 0;
        TRISDbits.TRISD1 = 0;
    #endif
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
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

    // Is this an interrupt-on-change interrupt?
    if ( INTCONbits.RBIF == 1 )
    {
        // Record which button was pressed so the main() loop can
        // handle it
        if (BIND_SWITCH == SWITCH_PRESSED)
            myStatusFlags.bits.bBindSwitchToggled = TRUE;

        #ifdef I_AM_SWITCH
            if (LIGHT_SWITCH == SWITCH_PRESSED)
                myStatusFlags.bits.bLightSwitchToggled = TRUE;
        #endif

        // Disable further RBIF until we process it
        INTCONbits.RBIE = 0;

        // Clear mis-match condition and reset the interrupt flag
        LATB = PORTB;

        INTCONbits.RBIF = 0;
    }
}
