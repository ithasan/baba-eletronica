/*********************************************************************
 *
 *                  Compiler specific defs.
 *

 This file was originally intended to help compatibility between different
 C compilers.  However, these definitions have the effect that standard
 Microchip syntax, such as PORTAbits.RA2, is illegal.

 This file has been modified such that standard Microchip syntax is now
 used for all file register access.  Translations for other compilers
 may be added in the future.

 *********************************************************************
 * FileName:        Compiler.h
 * Dependencies:    None
 * Processor:       PIC18
 * Complier:        MCC18 v1.00.50 or higher
 *                  HITECH PICC-18 V8.10PL1 or higher
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
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Nilesh Rajbharti     7/12/04     Rel 0.9
 * Nilesh Rajbharti     11/1/04     Pre-release version
 * DF/KO                04/29/05 Microchip ZigBee Stack v1.0-2.0
 * DF/KO                07/18/05 Microchip ZigBee Stack v1.0-3.0
 * DF/KO                07/27/05 Microchip ZigBee Stack v1.0-3.1
 * DF/KO                01/09/06 Microchip ZigBee Stack v1.0-3.5
 * DF/KO                08/31/06 Microchip ZigBee Stack v1.0-3.6
 * DF/KO/YY				11/27/06 Microchip ZigBee Stack v1.0-3.7 
 * DF/KO/YY				01/12/07 Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/
#ifndef COMPILER_H
#define COMPILER_H

#if defined(HI_TECH_C)
    #if defined(_MPC_)
        #define HITECH_C18
    #else
        #error "Unknown compiler is selected."
    #endif
#else
    #if !defined(_WIN32)
        #define MCHP_C18
    #endif
#endif

#if defined(MCHP_C18) && defined(HITECH_C18)
    #error "Invalid Compiler selection."
#endif

//#if !defined(MCHP_C18) && !defined(HITECH_C18) && !defined(_WIN32)
#if !defined(MCHP_C18)
    #error "Compiler not supported."
#endif

#if defined(MCHP_C18)
    #include <p18cxxx.h>    // p18cxxx.h must have current processor
                            // defined.
#endif

#if defined(HITECH_C18)
    #include <pic18.h>
    #include <stdio.h>
#endif

#include <stdlib.h>


#if defined(WIN32)
    #define ROM
#endif

#if defined(MCHP_C18)
    #define ROM                 rom

    #define NOP()               Nop()
    #define CLRWDT()            ClrWdt()
    #define RESET()             Reset()
    #define SLEEP()             Sleep()
    #define DISABLE_WDT()       (WDTCONbits.SWDTEN = 0)
    #define ENABLE_WDT()        (WDTCONbits.SWDTEN = 1)
    #define TBLWTPOSTINC()      _asm tblwtpostinc _endasm

#endif


#if defined(HITECH_C18)
    #define ROM                 const

    #define memcmppgm2ram(a, b, c)      memcmp(a, b, c)
    #define memcpypgm2ram(a, b, c)      mymemcpy(a, b, c)
    #define itoa(val, string)           sprintf(string, "%u", val)

    extern void *mymemcpy(void * d1, const void * s1, unsigned char n);
    extern char *strupr(char*);

    // Here's where HI-TECH equivalents of the register and bit names
    // need to be added.

    #define TBLWTPOSTINC()      asm(" tblwt*+")

#endif

#if defined(WIN32)
    #define memcpypgm2ram(d, s, c)  memcpy(d, s, c)
    #define TBLWTPOSTINC()
#endif

#endif
