/*

Home Control, Lighting Profile Definition File

This ZigBee Profile header file provides all of the constants needed to
implement a device that is conforms to the ZigBee Home Control, Lighting
profile.

Mandatory Clusters shall be used in all devices and require all attributes
to be full implemented with the designated data type.

Optional Clusters may be unsupported.  However, if a cluster is supported,
then all of that cluster's attributes shall be supported.

Refer to the ZigBee Specification for the Mandatory and Optional clusters
for each device.


NOTES:
(1) Simple Bind must be supported. See [APS] End Device Bind sections for more details.
(2) Simple Binding required on Light Sensor Monochromatic and Occupancy Sensor Inputs

 *********************************************************************
 * FileName:        zHCLighting.h
 * Dependencies:
 * Processor:       PIC18
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
 * DF/KO                04/29/05 Microchip ZigBee Stack v1.0-2.0
 * DF/KO                07/18/05 Microchip ZigBee Stack v1.0-3.0
 * DF/KO                07/27/05 Microchip ZigBee Stack v1.0-3.1
 * DF/KO                08/19/05 Microchip ZigBee Stack v1.0-3.2
 * DF/KO                01/09/06 Microchip ZigBee Stack v1.0-3.5
 * DF/KO                08/31/06 Microchip ZigBee Stack v1.0-3.6
 * DF/KO/YY				11/27/06 Microchip ZigBee Stack v1.0-3.7 
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
*/

#ifndef _ZHCLIGHTING_H_
#define _ZHCLIGHTING_H_

#define MY_PROFILE_ID                       0x0100  //Home Control, Lighting
#define MY_PROFILE_ID_MSB                   0x01
#define MY_PROFILE_ID_LSB                   0x00

#define DM_LOAD_CONTROLLER_DEV_ID      0xfffa   //Dimming Load Controller
#define DM_LOAD_CONTROLLER_DEV_ID_MSB  0xff
#define DM_LOAD_CONTROLLER_DEV_ID_LSB  0xfa
#define DM_LOAD_CONTROLLER_DEV_VER     0x00

#define SW_LOAD_CONTROLLER_DEV_ID      0xfffb   //Switch Load Controller
#define SW_LOAD_CONTROLLER_DEV_ID_MSB  0xff
#define SW_LOAD_CONTROLLER_DEV_ID_LSB  0xfb
#define SW_LOAD_CONTROLLER_DEV_VER     0x00

#define OCCUPANCY_SENSOR_DEV_ID        0xfffc   //Occupancy Sensor
#define OCCUPANCY_SENSOR_DEV_ID_MSB    0xff
#define OCCUPANCY_SENSOR_DEV_ID_LSB    0xfc
#define OCCUPANCY_SENSOR_DEV_VER       0x00

#define DM_REMOTE_CONTROL_DEV_ID       0xfffd   //Dimming Remote Control
#define DM_REMOTE_CONTROL_DEV_ID_MSB   0xff
#define DM_REMOTE_CONTROL_DEV_ID_LSB   0xfd
#define DM_REMOTE_CONTROL_DEV_VER      0x00

#define SW_REMOTE_CONTROL_DEV_ID       0xfffe   //Switch Remote Control
#define SW_REMOTE_CONTROL_DEV_ID_MSB   0xff
#define SW_REMOTE_CONTROL_DEV_ID_LSB   0xfe
#define SW_REMOTE_CONTROL_DEV_VER      0x00

#define LIGHT_SENSOR_MONO_DEV_ID       0xffff   //Light Sensor Monochromatic
#define LIGHT_SENSOR_MONO_DEV_ID_MSB   0xff
#define LIGHT_SENSOR_MONO_DEV_ID_LSB   0xff
#define LIGHT_SENSOR_MONO_DEV_VER      0x00


#define MY_STACK_PROFILE_ID                 0x01
#define PROFILE_nwkSecurityLevel            0x05        // TODO: should be 5 when security supported
#define PROFILE_nwkSecureAllFrames			0x01

#define PROFILE_nwkMaxChildren              20
#define PROFILE_nwkMaxDepth                 5
#define PROFILE_nwkMaxRouters               6


#define PROFILE_MinBindings                     100
#define PROFILE_MinCoordinatorNeighbors         24
#define PROFILE_MinRouterNeighbors              25
#define PROFILE_MinEndDeviceNeighbors           1
#define PROFILE_MinRouteDiscoveryTableSize      4
#define PROFILE_MinRoutingTableSize             8
#define PROFILE_MinReservedRoutingTableEntries  8

//******************************************************************************
// Distributed Address Assignment Constants
//
// These should be calculated manually and placed here.  They are calculated by
// the following formulae.  CSKIP values should be generated until the max
// depth is reached or until CSKIP equals 0.
//
//  Cskip(d) =  if PROFILE_nwkMaxRouters is 1 =
//                  1 + Cm * (Lm - d - 1)
//              otherwise =
//                  1 + Cm - Rm - (Cm * Rm^(Lm - d - 1))
//                  ------------------------------------
//                                1 - Rm
//  where
//      Cm = PROFILE_nwkMaxChildren
//      Lm = PROFILE_nwkMaxDepth
//      Rm = PROFILE_nwkMaxRouters
//      d  = depth of node in the network

#define CSKIP_DEPTH_0                       0x143D
#define CSKIP_DEPTH_1                       0x035D
#define CSKIP_DEPTH_2                       0x008D
#define CSKIP_DEPTH_3                       0x0015
#define CSKIP_DEPTH_4                       0x0001
#define CSKIP_DEPTH_5                       0x0000


//******************************************************************************
// Mandatory Clusters

#define OnOffSRC_CLUSTER        0x13    //ON or OFF commands for Switch Remote Control (1)
#define OnOffDRC_CLUSTER        0x01    //ON or OFF commands for Dimmer Remote Control (1)
#define DimBrightDRC_CLUSTER    0x02    //DIM or BRIGHT commands for Dimmer Remote Control (1)
#define PresetDRC_CLUSTER       0x03    //Presets lighting level for Dimmer Remote Control (1)


//******************************************************************************
// Optional Clusters

#define LightLevelLSM_CLUSTER   0x06    //Current light level reading for Light Sensor Monochromatic (2)
#define OccupancyOS_CLUSTER     0x08    //Current occupancy state for Occupancy Sensor (2)
#define ProgramLSM_CLUSTER      0x07    //Program input to set up key parameters for the Light Sensor Monochromatic
#define ProgramSRC_CLUSTER      0x14    //Program input to set up key parameters for the Switch Remote Control
#define AdjDRC_CLUSTER          0x05    //Output setup and additional dimmer commands to external device
#define ProgramDRC_CLUSTER      0x04    //Program input to set up key parameters for the Dimmer Remote Control
#define ProgramOS_CLUSTER       0x09    //Program input to set up key parameters for the Occupancy Sensor
#define ProgramSLC_CLUSTER      0x16    //Program input to set up key parameters for the Switching Load Controller
#define StatusSLC_CLUSTER       0x15    //Load status and energy consumption for Switching Load Controller
#define ProgramDLC_CLUSTER      0x12    //Program input to set up key parameters for the Dimming Load Controller
#define StatusDLC_CLUSTER       0x11    //Load status and energy consumption for Dimming Load Controller


//******************************************************************************
// Mandatory Cluster Attributes

//------------------------------------------------------------------------------
// Cluster OnOffSRC     : (1) ON or OFF commands for Switch Remote Control

#define OnOffSRC_OnOff                          0x0000  //Outputs ON and OFF command to external device
#define OnOffSRC_OnOff_DATATYPE                 TRANS_UINT8


//------------------------------------------------------------------------------
// Cluster OnOffDRC     : (1) ON or OFF commands for Dimmer Remote Control

#define OnOffDRC_OnOff                          0x0000  //Outputs ON and OFF command to external device
#define OnOffDRC_OnOff_DATATYPE                 TRANS_UINT8


//------------------------------------------------------------------------------
// Cluster DimBrightDRC : (1) DIM or BRIGHT commands for Dimmer Remote Control

#define DimBrightDRC_DimBright                  0x0000  //Changes the load controller output up or down in increments of 0.4% from a CurrentLevel or Preset.
#define DimBrightDRC_DimBright_DATATYPE         TRANS_UINT8


//------------------------------------------------------------------------------
// Cluster PresetDRC    : (1) Presets lighting level for Dimmer Remote Control

#define PresetDRC_StorePreset                   0x0000  //Store the current lighting level. This value becomes the Preset.
#define PresetDRC_StorePreset_DATATYPE          TRANS_NO_DATA

#define PresetDRC_Preset                        0x0100  //Recalls the stored preset level at a lighting load with or without the load being on.
#define PresetDRC_Preset_DATATYPE               TRANS_NO_DATA


//******************************************************************************
// Optional Cluster Attributes


//------------------------------------------------------------------------------
// Cluster LightLevelLSM    : (2) Current light level reading for Light Sensor Monochromatic

#define LightLevelLSM_CurrentLevel              0x0000  //Current light level reading
#define LightLevelLSM_CurrentLevel_DATATYPE     TRANS_SEMI_PRECISE


//------------------------------------------------------------------------------
// Cluster OccupancyOS      : (2) Current occupancy state for Occupancy Sensor

#define OccupancyOS_CurrentState                0x0000  //Current occupancy state of area being monitored (see Table 4-2 for details)
#define OccupancyOS_CurrentState_DATATYPE       TRANS_UINT8


//------------------------------------------------------------------------------
// Cluster ProgramLSM       : Program input to set up key parameters for the Light Sensor Monochromatic

#define ProgramLSM_ReportTime                   0x0000  //Maximum time permitted from the last time CurrentLevel was sent to output new CurrentLevel
#define ProgramLSM_ReportTime_DATATYPE          TRANS_UINT16

#define ProgramLSM_MinLevelChange               0x0001  //Minimum light level change in % of the CurrentLevel required to output new CurrentLevel
#define ProgramLSM_MinLevelChange_DATATYPE      TRANS_UINT16

#define ProgramLSM_MaxThreshold                 0x0002  //Light level threshold that must be exceeded to output new CurrentLevel.
#define ProgramLSM_MaxThreshold_DATATYPE        TRANS_SEMI_PRECISE

#define ProgramLSM_MinThreshold                 0x0003  //Light level below which a new CurrentLevel is to be output.
#define ProgramLSM_MinThreshold_DATATYPE        TRANS_SEMI_PRECISE

#define ProgramLSM_Offset                       0s0004  //Adds or subtracts offset to the measured light level to normalize with those at another location
#define ProgramLSM_Offset_DATATYPE              TRANS_SEMI_PRECISE

#define ProgramLSM_Override                     0x0005  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramLSM_Override_DATATYPE            TRANS_NO_DATA

#define ProgramLSM_Auto                         0x0006  //Override is disabled, all Clusters are enabled
#define ProgramLSM_Auto_DATATYPE                TRANS_NO_DATA

#define ProgramLSM_FactoryDefault               0x0007  //Resets all Attributes to factory defaults
#define ProgramLSM_FactoryDefault_DATATYPE      TRANS_NO_DATA


//------------------------------------------------------------------------------
// Cluster ProgramSRC       : Program input to set up key parameters for the Switch Remote Control

#define ProgramSRC_Override                     0x0000  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramSRC_Override_DATATYPE            TRANS_NO_DATA

#define ProgramSRC_Auto                         0x0001  //Override is disabled, all Clusters are enabled
#define ProgramSRC_Auto_DATATYPE                TRANS_NO_DATA

#define ProgramSRC_FactoryDefault               0x0002  //Resets all Attributes to factory defaults
#define ProgramSRC_FactoryDefault_DATATYPE      TRANS_NO_DATA


//------------------------------------------------------------------------------
// Cluster AdjDRC           : Output setup and additional dimmer commands to external device

#define AdjDRC_CurrentLevel                     0x0000  //Sets the desired lighting level in % of full on.
#define AdjDRC_CurrentLevel_DATATYPE            TRANS_UINT8

#define AdjDRC_PreviousLevel                    0x0100  //Recalls the last lighting level. A fade time can be applied.
#define AdjDRC_PreviousLevel_DATATYPE           TRANS_NO_DATA

#define AdjDRC_Stop                             0x0500  //Stops the dimming and freezes the current light level.
#define AdjDRC_Stop_DATATYPE                    TRANS_NO_DATA

#define AdjDRC_MinDimLevel                      0x0600  //The minimum dim level supported in % of full on
#define AdjDRC_MinDimLevel_DATATYPE             TRANS_UINT8

#define AdjDRC_MaxBrightLevel                   0x0700  //The maximum bright level supported in % of full on
#define AdjDRC_MaxBrightLevel_DATATYPE          TRANS_UINT8

//------------------------------------------------------------------------------
// Cluster ProgramDRC       : Program input to set up key parameters for the Dimmer Remote Control

#define ProgramDRC_Override                     0x0000  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramDRC_Override_DATATYPE            TRANS_NO_DATA

#define ProgramDRC_Auto                         0x0001  //Override is disabled, all Clusters are enabled
#define ProgramDRC_Auto_DATATYPE                TRANS_NO_DATA

#define ProgramDRC_FactoryDefault               0x0002  //Resets all Attributes to factory defaults
#define ProgramDRC_FactoryDefault_DATATYPE      TRANS_NO_DATA


//------------------------------------------------------------------------------
// Cluster ProgramOS        : Program input to set up key parameters for the Occupancy Sensor

#define ProgramOS_ReportTime                    0x0000  //Maximum time permitted from the last time CurrentState was sent to output new CurrentState
#define ProgramOS_ReportTime_DATATYPE           TRANS_UINT16

#define ProgramOS_TimeOut                       0x0001  //The minimum period of time with no occupancy detected before an "unoccupied" CurrentState is sent.
#define ProgramOS_TimeOut_DATATYPE              TRANS_UINT16

#define ProgramOS_CurrentStateOn                0x0002  //Forces CurrentState to report area occupied 0xFF
#define ProgramOS_CurrentStateOn_DATATYPE       TRANS_NO_DATA

#define ProgramOS_CurrentStateOff               0x0003  //Forces CurrentState to report area unoccupied 0x00
#define ProgramOS_CurrentStateOff_DATATYPE      TRANS_NO_DATA

#define ProgramOS_Override                      0x0004  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramOS_Override_DATATYPE             TRANS_NO_DATA

#define ProgramOS_Auto                          0x0005  //Override is disabled, all Clusters are enabled
#define ProgramOS_Auto_DATATYPE                 TRANS_NO_DATA

#define ProgramOS_FactoryDefault                0x0006  //Resets all Attributes to factory defaults
#define ProgramOS_FactoryDefault_DATATYPE       TRANS_NO_DATA


//------------------------------------------------------------------------------
// Cluster ProgramSLC       : Program input to set up key parameters for the Switching Load Controller

#define ProgramSLC_Override                     0x0000  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramSLC_Override_DATATYPE            TRANS_NO_DATA

#define ProgramSLC_Auto                         0x0001  //Override is disabled, all Clusters are enabled
#define ProgramSLC_Auto_DATATYPE                TRANS_NO_DATA

#define ProgramSLC_FactoryDefault               0x0002  //Resets all Attributes to factory defaults
#define ProgramSLC_FactoryDefault_DATATYPE      TRANS_NO_DATA

#define ProgramSLC_ResetO_LSSLC                 0x0007  //Reset Output Load Status to zero
#define ProgramSLC_ResetO_LSSLC_DATATYPE        TRANS_NO_DATA

#define ProgramSLC_PresetO_LSSLC                0x0008  //Preset Output Load Status
#define ProgramSLC_PresetO_LSSLC_DATATYPE       TRANS_NO_DATA

#define ProgramSLC_BrownOutMinVolt              0x0009  //Minimum voltage level for brown out detection
#define ProgramSLC_BrownOutMinVolt_DATATYPE     TRANS_UINT16

#define ProgramSLC_ShutDownPkCurrent            0x000a  //Peak Current for shut down
#define ProgramSLC_ShutDownPkCurrent_DATATYPE   TRANS_UINT16

#define ProgramSLC_MeteringPeriod               0x000b  //Length of Metering period in minutes for StatusSLC Attributes
#define ProgramSLC_MeteringPeriod_DATATYPE      TRANS_UINT16


//------------------------------------------------------------------------------
// Cluster StatusSLC        : Load status and energy consumption for Switching Load Controller

#define StatusSLC_OnOff                         0x0000  //Load ON contains data 0xFF, load OFF contains data 0x00
#define StatusSLC_OnOff_DATATYPE                TRANS_UINT8

#define StatusSLC_RunTime                       0x0001  //Total time load is ON
#define StatusSLC_RunTime_DATATYPE              TRANS_UINT16

#define StatusSLC_Watts                         0x0002  //Real power consumed by load in watts
#define StatusSLC_Watts_DATATYPE                TRANS_UINT16

#define StatusSLC_TotalPower                    0x0003  //Total Real and Reactive Power consumed by load in watts
#define StatusSLC_TotalPower_DATATYPE           TRANS_UINT16

#define StatusSLC_Energy                        0x0004  //Real power * time = Watts * runtime
#define StatusSLC_Energy_DATATYPE               TRANS_UINT16

#define StatusSLC_Vars                          0x0005  //Vars (Reactive Power)
#define StatusSLC_Vars_DATATYPE                 TRANS_UINT16

#define StatusSLC_Voltage                       0x0006  //Instantaneous Voltage
#define StatusSLC_Voltage_DATATYPE              TRANS_UINT16

#define StatusSLC_Current                       0x0007  //Instantaneous RMS Current
#define StatusSLC_Current_DATATYPE              TRANS_UINT16

#define StatusSLC_Ripple                        0x0008  //Ripple voltage
#define StatusSLC_Ripple_DATATYPE               TRANS_UINT16

#define StatusSLC_Frequency                     0x0009  //power line frequency
#define StatusSLC_Frequency_DATATYPE            TRANS_UINT16

#define StatusSLC_Phase                         0x000a  //A , B, C or N
#define StatusSLC_Phase_DATATYPE                TRANS_UINT16

#define StatusSLC_PhaseAngle                    0x000b  //Phase angle between voltage and current
#define StatusSLC_PhaseAngle_DATATYPE           TRANS_UINT16

#define StatusSLC_PF                            0x000c  //Power factor
#define StatusSLC_PF_DATATYPE                   TRANS_UINT16

#define StatusSLC_PeakWatts                     0x000d  //Peak power in Watts
#define StatusSLC_PeakWatts_DATATYPE            TRANS_UINT16

#define StatusSLC_PeakTotalPower                0x000e  //Peak Total Power
#define StatusSLC_PeakTotalPower_DATATYPE       TRANS_UINT16

#define StatusSLC_PeakVoltage                   0x000f  //Peak Voltage
#define StatusSLC_PeakVoltage_DATATYPE          TRANS_UINT16

#define StatusSLC_PeakCurrent                   0x0010  //Peak Current
#define StatusSLC_PeakCurrent_DATATYPE          TRANS_UINT16

#define StatusSLC_LoadType                      0x0011  //Resistive, Capacitive, Inductive
#define StatusSLC_LoadType_DATATYPE             TRANS_UINT16

//------------------------------------------------------------------------------
// Cluster ProgramDLC       : Program input to set up key parameters for the Dimming Load Controller

#define ProgramDLC_Override                     0x0000  //Outputs no longer report data or send control commands until placed in Auto Mode
#define ProgramDLC_Override_DATATYPE            TRANS_NO_DATA

#define ProgramDLC_Auto                         0x0001  //Override is disabled, all Clusters are enabled
#define ProgramDLC_Auto_DATATYPE                TRANS_NO_DATA

#define ProgramDLC_FactoryDefault               0x0002  //Resets all Attributes to factory defaults
#define ProgramDLC_FactoryDefault_DATATYPE      TRANS_NO_DATA

#define ProgramDLC_ResetO_LSDLC                 0x0007  //Reset Output Load Status to zero
#define ProgramDLC_ResetO_LSDLC_DATATYPE        TRANS_NO_DATA

#define ProgramDLC_PresetO_LSDLC                0x0008  //Preset Output Load Status
#define ProgramDLC_PresetO_LSDLC_DATATYPE       TRANS_NO_DATA

#define ProgramDLC_BrownOutMinVolt              0x0009  //Minimum voltage level for brown out detection
#define ProgramDLC_BrownOutMinVolt_DATATYPE     TRANS_UINT16

#define ProgramDLC_ShutDownPkCurrent            0x000a  //Peak Current for shut down
#define ProgramDLC_ShutDownPkCurrent_DATATYPE   TRANS_UINT16

#define ProgramDLC_MeteringPeriod               0x000b  //Length of Metering period in minutes for StatusDLC Attributes
#define ProgramDLC_MeteringPeriod_DATATYPE      TRANS_UINT16

//------------------------------------------------------------------------------
// Cluster StatusDLC        : Load status and energy consumption for Dimming Load Controller

#define StatusDLC_OnOff                         0x0000  //Load ON contains data 0xFF, load OFF contains data 0x00
#define StatusDLC_OnOff_DATATYPE                TRANS_UINT8

#define StatusDLC_RunTime                       0x0001  //Total time load is ON
#define StatusDLC_RunTime_DATATYPE              TRANS_UINT16

#define StatusDLC_Watts                         0x0002  //Real power consumed by load in watts
#define StatusDLC_Watts_DATATYPE                TRANS_UINT16

#define StatusDLC_TotalPower                    0x0003  //Total Real and Reactive Power consumed by load in watts
#define StatusDLC_TotalPower_DATATYPE           TRANS_UINT16

#define StatusDLC_Energy                        0x0004  //Real power * time = Watts * runtime
#define StatusDLC_Energy_DATATYPE               TRANS_UINT16

#define StatusDLC_Vars                          0x0005  //Vars (Reactive Power)
#define StatusDLC_Vars_DATATYPE                 TRANS_UINT16

#define StatusDLC_Voltage                       0x0006  //Instantaneous Voltage
#define StatusDLC_Voltage_DATATYPE              TRANS_UINT16

#define StatusDLC_Current                       0x0007  //Instantaneous RMS Current
#define StatusDLC_Current_DATATYPE              TRANS_UINT16

#define StatusDLC_Ripple                        0x0008  //Ripple voltage
#define StatusDLC_Ripple_DATATYPE               TRANS_UINT16

#define StatusDLC_Frequency                     0x0009  //Power line frequency
#define StatusDLC_Frequency_DATATYPE            TRANS_UINT16

#define StatusDLC_Phase                         0x000a  //A , B, C or N
#define StatusDLC_Phase_DATATYPE                TRANS_UINT16

#define StatusDLC_PhaseAngle                    0x000b  //Phase angle between voltage and current
#define StatusDLC_PhaseAngle_DATATYPE           TRANS_UINT16

#define StatusDLC_PF                            0x000c  //Power factor
#define StatusDLC_PF_DATATYPE                   TRANS_UINT16

#define StatusDLC_PeakWatts                     0x000d  //Peak power in Watts
#define StatusDLC_PeakWatts_DATATYPE            TRANS_UINT16

#define StatusDLC_PeakTotalPower                0x000e  //Peak Total Power
#define StatusDLC_PeakTotalPower_DATATYPE       TRANS_UINT16

#define StatusDLC_PeakVoltage                   0x000f  //Peak Voltage
#define StatusDLC_PeakVoltage_DATATYPE          TRANS_UINT16

#define StatusDLC_PeakCurrent                   0x0010  //Peak Current
#define StatusDLC_PeakCurrent_DATATYPE          TRANS_UINT16

#define StatusDLC_LoadType                      0x0011  //Resistive, Capacitive, Inductive
#define StatusDLC_LoadType_DATATYPE             TRANS_UINT16


//******************************************************************************
// Check User Assignments

// Check size of routing table
#if defined(I_SUPPORT_ROUTING)
    #if ROUTING_TABLE_SIZE < PROFILE_MinRoutingTableSize
        #error Routing Table size too small.  Check Profile description.
    #endif
    #if RESERVED_ROUTING_TABLE_ENTRIES < PROFILE_MinReservedRoutingTableEntries
        #error Reserved Routing Table entries too small.  Check Profile description.
    #endif
#endif

// Check size of neighbor table
#if defined(I_AM_COORDINATOR)
    #if MAX_NEIGHBORS < PROFILE_MinCoordinatorNeighbors
        #error Neighbor Table size too small.  Check Profile description.
    #endif
#elif defined (I_AM_ROUTER)
    #if MAX_NEIGHBORS < PROFILE_MinRouterNeighbors
        #error Neighbor Table size too small.  Check Profile description.
    #endif
#else
    #if MAX_NEIGHBORS < PROFILE_MinEndDeviceNeighbors
        #error Neighbor Table size too small.  Check Profile description.
    #endif
#endif

// Check size of route discovery table
#if defined(I_AM_ROUTER) || defined(I_AM_COORDINATOR)
    #if ROUTE_DISCOVERY_TABLE_SIZE < PROFILE_MinRouteDiscoveryTableSize
        #error Route Discovery Table size too small.  Check Profile description.
    #endif
#endif

// Check size of binding table
#if defined(I_AM_COORDINATOR)
    #if MAX_BINDINGS < PROFILE_MinBindings
        #error Binding Table size too small.  Check Profile description.
    #endif
#endif

// If coordinator, make sure that end device binding is supported.
#if defined(I_AM_COORDINATOR)
    #ifndef SUPPORT_END_DEVICE_BINDING
        #error This Profile requires that ZigBee coordinators support End Device Binding.
    #endif
#endif


#endif
