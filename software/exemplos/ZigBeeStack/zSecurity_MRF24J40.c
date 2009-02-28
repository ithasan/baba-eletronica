/*********************************************************************
 *
 *                  ZigBee Security Module for MRF24J40
 *
 *********************************************************************
 * FileName:        zSecurity_MRF24J40
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
 * DF/KO/YY				11/27/06    Microchip ZigBee Stack v1.0-3.7
 * DF/KO/YY				01/12/07	Microchip ZigBee Stack v1.0-3.8
 ********************************************************************/
#include "zigbee.h"

#ifdef I_SUPPORT_SECURITY
#include "ZigbeeTasks.h"
#include "Zigbee.def"
#include "zPHY.h"
#include "zMAC.h"
#include "MSPI.h"
#include "sralloc.h"
#include "generic.h"
#include "zSecurity.h"
#include "zPHY_MRF24J40.h"
#include "zNWK.h"
#include "zNVM.h"

#ifdef ZCP_DEBUG
    #include "Console.h"
#else
    #define ConsolePutROMString(x)
    #define printf(x)
    #define PrintChar(x)
#endif
#define CIPHER_RETRY 5

#if (RF_CHIP == MRF24J40) || (RF_CHIP == UZ2400)

typedef enum _CIPHER_MODE
{
    MODE_ENCRYPTION,
    MODE_DECRYPTION
} CIPHER_MODE;

typedef enum _CIPHER_STATUS
{
    CIPHER_SUCCESS = 0,
    CIPHER_ERROR,
    CIPHER_MIC_ERROR
} CIPHER_STATUS;

extern volatile TX_STAT TxStat;
extern BYTE nwkSecurityLevel;
extern volatile PHY_PENDING_TASKS  PHYTasksPending;

DWORD_VAL   OutgoingFrameCount[2];
DWORD_VAL IncomingFrameCount[2][MAX_NEIGHBORS];
SECURITY_STATUS	securityStatus;


CIPHER_STATUS PHYCipher(INPUT CIPHER_MODE CipherMode, INPUT SECURITY_INPUT SecurityInput, OUTPUT BYTE *OutData, OUTPUT BYTE *OutDataLen);

#ifdef USE_EXTERNAL_NVM
const BYTE defaultMasterKey[16] = {NVM_KEY_BYTE00, NVM_KEY_BYTE01,NVM_KEY_BYTE02,NVM_KEY_BYTE03,NVM_KEY_BYTE04,NVM_KEY_BYTE05,NVM_KEY_BYTE06,NVM_KEY_BYTE07,NVM_KEY_BYTE08,NVM_KEY_BYTE09,NVM_KEY_BYTE10,NVM_KEY_BYTE11,NVM_KEY_BYTE12,NVM_KEY_BYTE13,NVM_KEY_BYTE14,NVM_KEY_BYTE15};
NETWORK_KEY_INFO plainSecurityKey[2];

/*********************************************************************
 * Function:        BOOL SetSecurityKey(INPUT BYTE index, INPUT NETWORK_KEY_INFO newSecurityKey)
 *
 * PreCondition:    None
 *
 * Input:           BYTE 	index: the index to the security key, usually either 0 or 1
 *					NETWORK_KEY_INFO: the network key information to be saved in external NVM
 *
 * Output:          BOOL to indicate if operation is successful
 *
 * Side Effects:    The security keys will be encrypted and stored in the external NVM
 *
 * Overview:        This function is used to encrypt the security key and store it in the external
 * 					NVM.
 ********************************************************************/
BOOL SetSecurityKey(INPUT BYTE index, INPUT NETWORK_KEY_INFO newSecurityKey)
{
	SECURITY_INPUT SecurityInput;
	BYTE Counter;
	BYTE i;
	BYTE EncryptedLen;
	BYTE tmpBuf[16];
	LONG_ADDR myAddr;

	plainSecurityKey[index] = newSecurityKey;

	SecurityInput.cipherMode = 0x01;
	SecurityInput.FrameCounter.Val = 0;
	SecurityInput.SecurityControl.Val = nwkSecurityLevel;
	SecurityInput.SecurityKey = (BYTE *)defaultMasterKey;
	SecurityInput.KeySeq = 0;
	GetMACAddress(&myAddr);
	SecurityInput.SourceAddress = &myAddr;
	SecurityInput.Header = NULL;
	SecurityInput.HeaderLen = 0;
	
	Counter = CIPHER_RETRY;
	while(Counter)
	{
		SecurityInput.InputText = newSecurityKey.NetKey.v;
		SecurityInput.TextLen = 16;
		PHYCipher(MODE_ENCRYPTION, SecurityInput, tmpBuf, &EncryptedLen);
		
		SecurityInput.InputText = tmpBuf;
		SecurityInput.TextLen = EncryptedLen;
		if( PHYCipher(MODE_DECRYPTION, SecurityInput, newSecurityKey.NetKey.v, &i) == CIPHER_SUCCESS )
		{
			break;
		}
		Counter--;
	}
	
	if( Counter )
	{
		for(i = 0; i < 16; i++)
			currentNetworkKeyInfo.NetKey.v[i] = tmpBuf[i];
		
		currentNetworkKeyInfo.SeqNumber.v[0] = newSecurityKey.SeqNumber.v[0];
		currentNetworkKeyInfo.SeqNumber.v[1] = nwkMAGICResSeq;
	
		PutNwkKeyInfo((networkKeyInfo+index*sizeof(NETWORK_KEY_INFO)), &currentNetworkKeyInfo);
		
		return TRUE;
	}
	
	return FALSE;
}

/*********************************************************************
 * Function:        BOOL InitSecurityKey(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          BOOL to indicate if operation is successful
 *
 * Side Effects:    The security keys stored in external NVM encrypted get decrypted and 
 *					stored in the RAM
 *
 * Overview:        This function is used in the system start up to retrieve security key
 *					stored in external NVM encrypted and decrypt them and store in the
 *					RAM for future security operation use
 ********************************************************************/
BOOL InitSecurityKey(void)
{
	SECURITY_INPUT SecurityInput;
	BYTE i,j;
	BYTE DataLen;
	LONG_ADDR myAddr;
	
	for(i = 0; i < 2; i++)
	{
		GetNwkKeyInfo(&currentNetworkKeyInfo, (networkKeyInfo+i*sizeof(NETWORK_KEY_INFO)));
		//printf("\r\nStored key:");
		//for(j=0;j<16;j++) PrintChar(currentNetworkKeyInfo.NetKey.v[j]);
		plainSecurityKey[i].SeqNumber.Val = currentNetworkKeyInfo.SeqNumber.Val;
		if( currentNetworkKeyInfo.SeqNumber.v[1] != nwkMAGICResSeq )
		{
			plainSecurityKey[i].SeqNumber = currentNetworkKeyInfo.SeqNumber;
			continue;
		}
		
		SecurityInput.cipherMode = 0x01;
		SecurityInput.FrameCounter.Val = 0;
		SecurityInput.SecurityControl.Val = nwkSecurityLevel;
		SecurityInput.SecurityKey = (BYTE *)defaultMasterKey;
		SecurityInput.KeySeq = 0;
		GetMACAddress(&myAddr);
		SecurityInput.SourceAddress = &myAddr;
		SecurityInput.Header = NULL;
		SecurityInput.HeaderLen = 0;
		SecurityInput.InputText = currentNetworkKeyInfo.NetKey.v;
		SecurityInput.TextLen = 16;	
		
		if( PHYCipher(MODE_DECRYPTION, SecurityInput, plainSecurityKey[i].NetKey.v, &DataLen) != CIPHER_SUCCESS )
		{
			return FALSE;
		}
		plainSecurityKey[i].SeqNumber = currentNetworkKeyInfo.SeqNumber;
		//{
		//	BYTE j;
		//	printf("\r\nRecovered Key: ");
		//	for(j = 0; j < DataLen; j++)
		//		PrintChar(plainSecurityKey[i].NetKey.v[j]);
		//}	
		
	}
	
	return TRUE;
}


#endif



/*********************************************************************
 * Function:        void SetNonce(INPUT LONG_ADDR *SourceAddress, INPUT DWORD_VAL *FrameCounter, INPUT SECURITY_CONTROL SecurityControl)
 *
 * PreCondition:    Input valid data for nonce
 *
 * Input:           LONG_ADDR *SourceAddress            - Extended source address
 *                  DWORD_VAL *FrameCounter             - FrameCounter
 *                  SECURITY_CONTROL SecurityControl    - Security control byte in auxilary header
 *
 * Output:          None
 *
 * Side Effects:    Security nonce being set
 *
 * Overview:        This function set the security nonce for hardware cipher
 ********************************************************************/
void SetNonce(INPUT LONG_ADDR *SourceAddress, INPUT DWORD_VAL *FrameCounter, INPUT SECURITY_CONTROL SecurityControl)
{
    WORD loc = 0x24C;
    BYTE i;

    for(i = 0; i < 8; i++) {
        PHYSetLongRAMAddr(loc--, SourceAddress->v[i]);
    }

    for(i = 0; i < 4; i++)
    {
        PHYSetLongRAMAddr(loc--, FrameCounter->v[i]);
    }

    PHYSetLongRAMAddr(loc--, SecurityControl.Val | nwkSecurityLevel);
}


/*********************************************************************
 * Function:        CIPHER_STATUS PHYCipher(INPUT CIPHER_MODE CipherMode, INPUT SECURITY_INPUT SecurityInput, OUTPUT BYTE *OutData, OUTPUT BYTE *OutDataLen)
 *
 * PreCondition:    Called by DataEncrypt or DataDecrypt
 *
 * Input:           CIPHER_MODE CipherMode       - Either MODE_ENCRYPTION or MODE_DECRYPTION
 *                  SECURITY_INPUT SecurityInput - Cipher operation input. Filled by DataEncryption or DataDecryption
 *
 * Output:          BYTE *OutData                - Encrypted or decrypted data, including MIC
 *                  BYTE *OutDataLen             - Data length after cipher operation, including MIC bytes
 *                  CIPHER_STATUS                - Cipher operation result
 *
 * Side Effects:    Input data get encrypted or decrypted and put into output buffer
 *
 * Overview:        This is the function that invoke the hardware cipher to do encryption and decryption
 ********************************************************************/
CIPHER_STATUS PHYCipher(INPUT CIPHER_MODE CipherMode, INPUT SECURITY_INPUT SecurityInput, OUTPUT BYTE *OutData, OUTPUT BYTE *OutDataLen)
{
    BYTE CipherRetry = CIPHER_RETRY;
    BYTE i;
    WORD loc;

    // make sure that we are not in the process of sending out a packet
    loc = 0;
    while( !TxStat.finished )
    {
        loc++;
        if( loc > 0xfff )
        {
            PHY_RESETn = 0;
            MACEnable();
            TxStat.finished = 1;
        }

        if( PORTBbits.RB0 == 0 )
        {
            INTCONbits.INT0IF = 1;
        }
        Nop();
    }

CipherOperationStart:

    // step 1, set the normal FIFO
    // step 1a, fill the length of the header
    if( SecurityInput.cipherMode > 0x04 )
    {
        PHYSetLongRAMAddr(0x000, SecurityInput.HeaderLen+SecurityInput.TextLen+14);
    } else {
        PHYSetLongRAMAddr(0x000, SecurityInput.HeaderLen+14);
    }

    // step 1b, fill the length of the packet
    if( CipherMode == MODE_ENCRYPTION )
    {
        PHYSetLongRAMAddr(0x001, SecurityInput.TextLen+SecurityInput.HeaderLen+14);
    } else {
        PHYSetLongRAMAddr(0x001, SecurityInput.TextLen+SecurityInput.HeaderLen+16);// two additional bytes FCS
    }

    // step 1c, fill the header
    loc = 0x002;
    for(i = 0; i < SecurityInput.HeaderLen; i++)
    {
        PHYSetLongRAMAddr(loc++, SecurityInput.Header[i]);
    }

    // step 1d, fill the auxilary header
    PHYSetLongRAMAddr(loc++, SecurityInput.SecurityControl.Val | nwkSecurityLevel);
    for(i = 0; i < 4; i++)
    {
        PHYSetLongRAMAddr(loc++, SecurityInput.FrameCounter.v[i]);
    }
    for(i = 0; i < 8; i++)
    {
        PHYSetLongRAMAddr(loc++, SecurityInput.SourceAddress->v[i]);
    }
    PHYSetLongRAMAddr(loc++, SecurityInput.KeySeq);

    // step 1e, fill the data to be encrypted or decrypted
    for(i = 0; i < SecurityInput.TextLen; i++)
    {
        PHYSetLongRAMAddr(loc++, SecurityInput.InputText[i]);
    }

    // step 2, set nounce
    SetNonce(SecurityInput.SourceAddress, &(SecurityInput.FrameCounter), SecurityInput.SecurityControl);

    // step 3, set TXNFIFO security key
    loc = 0x280;
    for(i = 0; i < 16; i++)
    {
        PHYSetLongRAMAddr(loc++, SecurityInput.SecurityKey[i]);
    }

    // step 4, set cipher mode either encryption or decryption
    if( CipherMode == MODE_ENCRYPTION )
    {
        PHYSetShortRAMAddr(SECCR2, 0x40);
    } else {
        PHYSetShortRAMAddr(SECCR2, 0x80);
    }

    // step 5, fill the encryption mode
    PHYSetShortRAMAddr(SECCR0, SecurityInput.cipherMode);

    TxStat.cipher = 1;
    // step 6, trigger
    PHYSetShortRAMAddr(TXNMTRIG, 0x03);

    i = 0;
    while( TxStat.cipher )
    {
        if( PORTBbits.RB0 == 0 )
        {
            INTCONbits.INT0IF = 1;
        }
        i++;
        #if 1
        // in rare condition, the hardware cipher will stall. Handle such condition
        // here
        if(i > 0x1f)
        {
            // in certain rare cases, the RX and Upper Cipher will block each other
            // in case that happens, reset the RF chip to avoid total disfunction
            ConsolePutROMString((rom char*)"X");
            PHYTasksPending.Val = 0;
            PHY_RESETn = 0;
            MACEnable();
            break;
        }
        #endif
    }
    //PHYSetShortRAMAddr(0x0d, 0x01);

    // if MIC is generated, check MIC here
    if( (CipherMode == MODE_DECRYPTION) && (SecurityInput.cipherMode != 0x01))
    {
        BYTE MIC_check = PHYGetShortRAMAddr(0x30);
        if( MIC_check & 0x40 )
        {
            PHYSetShortRAMAddr(0x30, 0x40);
            // there is a small chance that the hardware cipher will not
            // decrypt for the first time, retry to solve this problem.
            // details documented in errata
            if( CipherRetry )
            {
                CipherRetry--;
                for(loc = 0; loc < 0x255; loc++);
                goto CipherOperationStart;
            }
            PHY_RESETn = 0;
            MACEnable();
            printf("MIC error");
            return CIPHER_MIC_ERROR;
        }
    }

    if( TxStat.success )
    {
        // get output data length
        *OutDataLen = PHYGetLongRAMAddr(0x001) - SecurityInput.HeaderLen - 14;
        // get the index of data encrypted or decrypted
        loc = 0x002 + SecurityInput.HeaderLen + 14;

        // if this is a decryption operation, get rid of the useless MIC and two bytes of FCS
        if( CipherMode == MODE_DECRYPTION )
        {
            switch( SecurityInput.cipherMode )
            {
                case 0x02:
                case 0x05:
                    *OutDataLen -= 18;
                    break;
                case 0x03:
                case 0x06:
                    *OutDataLen -= 10;
                    break;
                case 0x04:
                case 0x07:
                    *OutDataLen -= 6;
                    break;
				case 0x01:
					*OutDataLen-= 2;
					break;
            }
        }

        // copy the output data
        for(i = 0; i < *OutDataLen; i++)
        {
            OutData[i] = PHYGetLongRAMAddr(loc++);
        }
        return CIPHER_SUCCESS;
    }

    return CIPHER_ERROR;
}

/*********************************************************************
 * Function:        BYTE SecurityLevel_ZIGBEE_2_IEEE(INPUT BYTE SL_ZigBee)
 *
 * PreCondition:    Input a valid ZigBee security level
 *
 * Input:           BYTE SL_ZigBee - ZigBee security level
 *
 * Output:          BYTE           - IEEE 802.15.4 security level
 *
 * Side Effects:    None
 *
 * Overview:        This function transfers ZigBee security level to IEEE 802.15.4 security level
 ********************************************************************/
BYTE SecurityLevel_ZIGBEE_2_IEEE(INPUT BYTE SL_ZigBee)
{
    BYTE SwitchTable[8] = {0x00, 0x07, 0x06, 0x05, 0x01, 0x04, 0x03, 0x02};
    return SwitchTable[SL_ZigBee];
}

/*********************************************************************
 * Function:        BOOL DataEncrypt(IOPUT BYTE *Input, IOPUT BYTE *DataLen, INPUT BYTE *Header, INPUT BYTE HeaderLen, INPUT KEY_IDENTIFIER KeyIdentifier, BOOL bExtendedNonce)
 *
 * PreCondition:    Input and Header has been filled
 *
 * Input:           BYTE *Header                 - Point to MiWi header
 *                  BYTE HeaderLen               - MiWi header length
 *                  KEY_IDENTIFIER KeyIdentifier - Identifier to specify key type. reserved for commercial mode
 *                  BOOL bExtendedNonce          - if extended nonce being used.
 *
 * Output:          BOOL           - If data encryption successful
 *
 * Input/Output:    BYTE *Input    - Pointer to the data to be encrypted. The encrypted data will be write back to the pointer
 *                  BYTE *DataLen  - Input as the length of the data to be encrypted. The encrypted data length (including MICs) will be written back
 *
 * Side Effects:    Input data get encrypted and written back to the input pointer
 *
 * Overview:        This is the function that call the hardware cipher to encrypt input data
 ********************************************************************/
BOOL DataEncrypt(IOPUT BYTE *Input, IOPUT BYTE *DataLen, INPUT BYTE *Header, INPUT BYTE HeaderLen, INPUT KEY_IDENTIFIER KeyIdentifier, BOOL bExtendedNonce)
{
    SECURITY_INPUT SecurityInput;
    BYTE EncryptedLen;
    BYTE i;
    BYTE Counter;
    BYTE ActiveKeyIndex;
    LONG_ADDR myAddress;
    // reserve space for multiple try of encryption
    BYTE *tmpBuf = (BYTE *)SRAMalloc(*DataLen + 18);
    if( tmpBuf == NULL )
    {
        return FALSE;
    }

    // get the IEEE 802.15.4 security mode
    SecurityInput.cipherMode = SecurityLevel_ZIGBEE_2_IEEE(nwkSecurityLevel);

    // get security key
    GetNwkActiveKeyNumber(&ActiveKeyIndex);
    if( ActiveKeyIndex != 0x01 && ActiveKeyIndex != 0x02 ) // no valid key
    {
        free(tmpBuf);
        return FALSE;
    }

    // handle the frame counter
    SecurityInput.FrameCounter = (OutgoingFrameCount[ActiveKeyIndex-1]);
    OutgoingFrameCount[ActiveKeyIndex-1].Val++;

    // fill the secuirty input
    SecurityInput.SecurityControl.Val = nwkSecurityLevel | (KeyIdentifier << 3);
    if( bExtendedNonce )
    {
        SecurityInput.SecurityControl.Val |= 0x20;
    }
    #ifdef USE_EXTERNAL_NVM
		currentNetworkKeyInfo = plainSecurityKey[ActiveKeyIndex-1];
    #else
        GetNwkKeyInfo( &currentNetworkKeyInfo, &(networkKeyInfo[ActiveKeyIndex-1]) );
    #endif

    SecurityInput.SecurityKey = currentNetworkKeyInfo.NetKey.v;
    SecurityInput.KeySeq = currentNetworkKeyInfo.SeqNumber.v[0];
    GetMACAddress(&myAddress);
    SecurityInput.SourceAddress = &myAddress;
    SecurityInput.Header = Header;
    SecurityInput.HeaderLen = HeaderLen;

    // in rare cases, the hardware encryption engine may not suceed for the
    // first time. Retry a few times will solve the problem
    Counter = CIPHER_RETRY;
    while(Counter)
    {
        // fill the input data and data length
        SecurityInput.InputText = Input;
        SecurityInput.TextLen = *DataLen;
        // call hardware cipher and store the output to the temporary buffer
        PHYCipher(MODE_ENCRYPTION, SecurityInput, tmpBuf, &EncryptedLen);

        // try to decrypt the buffer to make sure that encryption is correct
        SecurityInput.InputText = tmpBuf;
        SecurityInput.TextLen = EncryptedLen;
        if( PHYCipher(MODE_DECRYPTION, SecurityInput, Input, &i) == CIPHER_SUCCESS )
        {
            break;
        }
        Counter--;
    }

    // fill the auxilary header
    Input[0] = SecurityInput.SecurityControl.Val & 0xF8; // set security level
    for(i = 0; i < 4; i++)
    {
        Input[i+1] = SecurityInput.FrameCounter.v[i];
    }
    Counter = i+1;
    if( bExtendedNonce )
    {
        for(i = 0; i < 8; i++)
        {
            Input[Counter++] = SecurityInput.SourceAddress->v[i];
        }
    }

    if( KeyIdentifier == ID_NetworkKey )
    {
        Input[Counter++] = currentNetworkKeyInfo.SeqNumber.v[0];
    }

    // fill the encrypted data
    for(i = 0; i < EncryptedLen; i++)
    {
        Input[Counter++] = tmpBuf[i];
    }

    free(tmpBuf);

    *DataLen = Counter;

    return TRUE;
}

/*********************************************************************
 * Function:        BOOL DataDecrypt(IOPUT BYTE *Input, IOPUT BYTE *DataLen, INPUT BYTE *Header, INPUT BYTE HeaderLen, INPUT KEY_IDENTIFIER KeyIdentifier, INPUT LONG_ADDR *longAddr)
 *
 * PreCondition:    Input and Header has been filled
 *
 * Input:           BYTE *Header                    - Point to MiWi header
 *                  BYTE HeaderLen                  - MiWi header length
 *                  KEY_IDENTIFIER KeyIdentifier    - Identifier to specify key type
 *                  LONG_ADDRESS *longAddress       - Extended source address if not use extended nonce
 *
 * Output:          BOOL           - If data encryption successful
 *
 * Input/Output:    BYTE *Input    - Pointer to the data to be decrypted. The decrypted data will be write back to the pointer
 *                  BYTE *DataLen  - Input as the length of the data to be decrypted. The encrypted data length (excluding MICs) will be written back
 *
 * Side Effects:    Input data get decrypted and written back to the input pointer
 *
 * Overview:        This is the function that call the hardware cipher to decrypt input data
 ********************************************************************/
BOOL DataDecrypt(IOPUT BYTE *Input, IOPUT BYTE *DataLen, INPUT BYTE *Header, INPUT BYTE HeaderLen, INPUT KEY_IDENTIFIER KeyIdentifier, INPUT LONG_ADDR *longAddr)
{
    SECURITY_INPUT SecurityInput;
    SECURITY_CONTROL SecurityControl;
    BYTE i;
    BYTE Counter;
    DWORD_VAL   FrameCounter;
    LONG_ADDR mySourceAddress;
    BYTE KeySeq;
    BYTE ActiveKeyIndex;

    // retrieve information from auxilary header
    SecurityControl.Val = Input[0];
    Counter = 1;
    for(i = 0; i < 4; i++)
    {
        FrameCounter.v[i] = Input[Counter++];
    }
    if( SecurityControl.bits.ExtendedNonce )
    {
        for(i = 0; i < 8; i++)
        {
            mySourceAddress.v[i] = Input[Counter++];
        }
    } else {
        mySourceAddress = *longAddr;
    }

    if( SecurityControl.bits.KeyIdentifier == ID_NetworkKey )
    {
        KeySeq = Input[Counter++];
        // get security key based on the key sequence number
        for(i = 0; i < 2; i++)
        {
            #ifdef USE_EXTERNAL_NVM
				currentNetworkKeyInfo = plainSecurityKey[i];
            #else
                GetNwkKeyInfo(&currentNetworkKeyInfo, &networkKeyInfo[i]);
            #endif
            if( KeySeq == currentNetworkKeyInfo.SeqNumber.v[0] && currentNetworkKeyInfo.SeqNumber.v[1] == nwkMAGICResSeq )
            {
                ActiveKeyIndex = i;
                break;
            }
        }
        if( i == 2 )
        {
            ConsolePutROMString( (ROM char *)"No Valid Key.\r\n" );
            return FALSE;
        }
    }

    // fill the security input
    SecurityInput.cipherMode = SecurityLevel_ZIGBEE_2_IEEE(nwkSecurityLevel);
    SecurityInput.FrameCounter = FrameCounter;
    SecurityInput.InputText = &(Input[Counter]);
    SecurityInput.SecurityControl = SecurityControl;
    SecurityInput.SecurityKey = currentNetworkKeyInfo.NetKey.v;
    SecurityInput.KeySeq = KeySeq;
    SecurityInput.SourceAddress = &mySourceAddress;
    SecurityInput.TextLen = *DataLen - Counter;
    SecurityInput.Header = Header;
    SecurityInput.HeaderLen = HeaderLen;

    // call hardware cipher
    if( PHYCipher(MODE_DECRYPTION, SecurityInput, Input, DataLen) != 0 )
    {
//      printf("decrpt wrong");
        return FALSE;
    }

    // check the frame counter. make sure that the frame counter increase always
    // we only check family members, because only family members know if a node
    // join or leave the network to reset the frame counter
    if( securityStatus.flags.bits.nwkAllFresh )
    {
        if( INVALID_NEIGHBOR_KEY != (i = NWKLookupNodeByLongAddr(&mySourceAddress)) )
        {
            if( (currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_CHILD ||
                    currentNeighborRecord.deviceInfo.bits.Relationship == NEIGHBOR_IS_PARENT ) &&
                FrameCounter.Val < IncomingFrameCount[ActiveKeyIndex][i].Val )
            {
                printf("frame counter");
                PrintChar(FrameCounter.v[3]);
                PrintChar(FrameCounter.v[2]);
                PrintChar(FrameCounter.v[1]);
                PrintChar(FrameCounter.v[0]);
                printf(" vs ");
                PrintChar(IncomingFrameCount[ActiveKeyIndex][i].v[3]);
                PrintChar(IncomingFrameCount[ActiveKeyIndex][i].v[2]);
                PrintChar(IncomingFrameCount[ActiveKeyIndex][i].v[1]);
                PrintChar(IncomingFrameCount[ActiveKeyIndex][i].v[0]);
                return FALSE;
            }
            IncomingFrameCount[ActiveKeyIndex][i].Val = FrameCounter.Val;
        }
    }
    return TRUE;
}

#else
    #error Please link the appropriate security file for the selected transceiver.
#endif      // RF_CHIP == MRF24J40

#endif
