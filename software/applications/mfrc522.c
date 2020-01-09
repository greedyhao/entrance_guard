#include "mfrc522.h"
#include<string.h>

static rt_base_t _chipSelectPin, _resetPowerDownPin;

#define PIN_NUM_MAX     255

#ifndef MFRC522_SPI_DEVICE_NAME
#define MFRC522_SPI_DEVICE_NAME "spi1"
#endif

static struct rt_spi_device *spi_dev_rc522;     /* SPI 设备句柄 */
static Uid uid;								// Used by PICC_ReadCardSerial().
static char *msg;
#define MSG_BUF_MAX 50U

//
// Functions for setting up the MFRC522
//

// static void gpio_init(void)
// {
//     rt_pin_mode(_chipSelectPin, PIN_MODE_OUTPUT);
//     rt_pin_mode(_resetPowerDownPin, PIN_MODE_OUTPUT);
// }

void MFRC522(rt_base_t chipSelectPin, rt_base_t resetPowerDownPin)
{
    _chipSelectPin = chipSelectPin;
    _resetPowerDownPin = resetPowerDownPin;
	spi_dev_rc522 = (struct rt_spi_device *)rt_device_find(MFRC522_SPI_DEVICE_NAME);
}

//
// Basic interface functions for communicating with the MFRC522
//

void PCD_WriteReg_Bit(enum PCD_Register reg, byte value)
{
	rt_pin_write(_chipSelectPin, PIN_LOW);
	rt_spi_send(spi_dev_rc522, (void *)&reg, 1);
	rt_spi_send(spi_dev_rc522, (void *)&value, 1);
	rt_pin_write(_chipSelectPin, PIN_HIGH);
}

void PCD_WriteRegister(enum PCD_Register reg, byte count, byte *values)
{
	byte tmp = 0;
	tmp = (byte)reg;
	rt_pin_write(_chipSelectPin, PIN_LOW);
	rt_spi_send(spi_dev_rc522, &tmp, 1);
	rt_spi_send(spi_dev_rc522, values, count);
	rt_pin_write(_chipSelectPin, PIN_HIGH);
}

byte PCD_ReadReg_Bit(enum PCD_Register reg)
{
	byte value = 0;
	rt_pin_write(_chipSelectPin, PIN_LOW);			// Select slave
	byte address = 0x80 | reg;			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	rt_spi_send(spi_dev_rc522, &address, 1);					
	rt_spi_recv(spi_dev_rc522, &value, 1);	// Read the value back. Send 0 to stop reading.
	rt_pin_write(_chipSelectPin, PIN_HIGH);			// Release slave again
	return value;
}

void PCD_ReadRegister(enum PCD_Register reg, byte count, byte *values, byte rxAlign)
{
	if (count == 0) {
		return;
	}
	byte address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	byte index = 0;							// Index in values array.
	rt_pin_write(_chipSelectPin, PIN_LOW);
	count--;								// One read is performed outside of the loop
	rt_spi_send(spi_dev_rc522, &address, 1);					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		byte mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		byte value = 0;
		rt_spi_recv(spi_dev_rc522, &value, 1);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		rt_spi_transfer(spi_dev_rc522, &address, &values[index], 1);	// Read value and tell that we want to read the same address again.
		index++;
	}
	address = 0;			
	rt_spi_transfer(spi_dev_rc522, &address, &values[index], 1); // Read the final byte. Send 0 to stop reading.
	rt_pin_write(_chipSelectPin, PIN_HIGH);
}

void PCD_SetRegisterBitMask(enum PCD_Register reg, byte mask)
{
	byte tmp = 0;
	tmp = PCD_ReadReg_Bit(reg);
	PCD_WriteReg_Bit(reg, tmp | mask);			// set bit mask
}

void PCD_ClearRegisterBitMask(enum PCD_Register reg, byte mask)
{
	byte tmp = 0;
	tmp = PCD_ReadReg_Bit(reg);
	PCD_WriteReg_Bit(reg, tmp & (~mask));		// clear bit mask
}

enum StatusCode PCD_CalculateCRC(byte *data, byte length, byte *result)
{
	PCD_WriteReg_Bit(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteReg_Bit(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteReg_Bit(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteReg_Bit(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		byte n = 0;
		n = PCD_ReadReg_Bit(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteReg_Bit(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadReg_Bit(CRCResultRegL);
			result[1] = PCD_ReadReg_Bit(CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()

//
// Functions for manipulating the MFRC522
//

void PCD_Init()
{
    bool hardReset = false;

	// RTT code
	if (msg == RT_NULL)
		msg = (char *)rt_malloc((MSG_BUF_MAX+1)*sizeof(char));

    // Set the chipSelectPin as digital output, do not select the slave yet
    rt_pin_mode(_chipSelectPin, PIN_MODE_OUTPUT);
    rt_pin_write(_chipSelectPin, PIN_HIGH);

    // If a valid pin number has been set, pull device out of power down / reset state.
	if (_resetPowerDownPin < PIN_NUM_MAX) {
		// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
		rt_pin_mode(_resetPowerDownPin, PIN_MODE_INPUT_PULLUP);

		if (rt_pin_read(_resetPowerDownPin) == PIN_LOW) {	// The MFRC522 chip is in power down mode.
			rt_pin_mode(_resetPowerDownPin, PIN_MODE_OUTPUT);		// Now set the resetPowerDownPin as digital output.
			rt_pin_write(_resetPowerDownPin, PIN_LOW);		// Make sure we have a clean LOW state.
			rt_thread_mdelay(2);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
			rt_pin_write(_resetPowerDownPin, PIN_HIGH);		// Exit power down mode. This triggers a hard reset.
			// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
			rt_thread_mdelay(50);
			hardReset = true;
		}
	}

    if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		PCD_Reset();
	}

    // Reset baud rates
	PCD_WriteReg_Bit(TxModeReg, 0x00);
	PCD_WriteReg_Bit(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteReg_Bit(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.	
	PCD_WriteReg_Bit(TModeReg, 0x80);		// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteReg_Bit(TPrescalerReg, 0xA9);	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.	
	PCD_WriteReg_Bit(TReloadRegH, 0x03);	// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteReg_Bit(TReloadRegL, 0xE8);

	PCD_WriteReg_Bit(TxASKReg, 0x40);	// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting	
	PCD_WriteReg_Bit(ModeReg, 0x3D);	// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

void PCD_Reset()
{
	PCD_WriteReg_Bit(CommandReg, PCD_SoftReset);// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;
	uint8_t value = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		rt_thread_mdelay(50);
		value = PCD_ReadReg_Bit(CommandReg);
	} while ((value & (1 << 4)) && (++count) < 3);
}

void PCD_AntennaOn()
{
    byte value = 0;
	value = PCD_ReadReg_Bit(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteReg_Bit(TxControlReg, value | 0x03);
	}
}

// void PCD_AntennaOff(void);
// byte PCD_GetAntennaGain(void);
// void PCD_SetAntennaGain(byte mask);
// bool PCD_PerformSelfTest(void);

/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_MIFARE_Transceive(	byte *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
										byte sendLen,		///< Number of bytes in sendData.
										bool acceptTimeout	///< True => A timeout is also success
									) {
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Returns a __FlashStringHelper pointer to a status code name.
 * 
 * @return const __FlashStringHelper *
 */
const char *GetStatusCodeName(enum StatusCode code	///< One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:				strncpy(msg, "Success.", MSG_BUF_MAX);
		case STATUS_ERROR:			strncpy(msg, "Error in communication.", MSG_BUF_MAX);
		case STATUS_COLLISION:		strncpy(msg, "Collission detected.", MSG_BUF_MAX);
		case STATUS_TIMEOUT:		strncpy(msg, "Timeout in communication.", MSG_BUF_MAX);
		case STATUS_NO_ROOM:		strncpy(msg, "A buffer is not big enough.", MSG_BUF_MAX);
		case STATUS_INTERNAL_ERROR:	strncpy(msg, "Internal error in the code. Should not happen.", MSG_BUF_MAX);
		case STATUS_INVALID:		strncpy(msg, "Invalid argument.", MSG_BUF_MAX);
		case STATUS_CRC_WRONG:		strncpy(msg, "The CRC_A does not match.", MSG_BUF_MAX);
		case STATUS_MIFARE_NACK:	strncpy(msg, "A MIFARE PICC responded with NAK.", MSG_BUF_MAX);
		default:					strncpy(msg, "Unknown error", MSG_BUF_MAX);
	}
	return msg;
} // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
enum PICC_Type PICC_GetType(byte sak		///< The SAK byte returned from PICC_Select().
							) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	// byte sak = uid.sak; ///< The SAK byte returned from PICC_Select().
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()

/**
 * Returns a __FlashStringHelper pointer to the PICC type name.
 * 
 * @return const __FlashStringHelper *
 */
const char *PICC_GetTypeName(enum PICC_Type piccType	///< One of the PICC_Type enums.
						) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		strncpy(msg, "PICC compliant with ISO/IEC 14443-4", MSG_BUF_MAX);
		case PICC_TYPE_ISO_18092:		strncpy(msg, "PICC compliant with ISO/IEC 18092 (NFC)", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_MINI:		strncpy(msg, "MIFARE Mini, 320 bytes", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_1K:		strncpy(msg, "MIFARE 1KB", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_4K:		strncpy(msg, "MIFARE 4KB", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_UL:		strncpy(msg, "MIFARE Ultralight or Ultralight C", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_PLUS:		strncpy(msg, "MIFARE Plus", MSG_BUF_MAX);
		case PICC_TYPE_MIFARE_DESFIRE:	strncpy(msg, "MIFARE DESFire", MSG_BUF_MAX);
		case PICC_TYPE_TNP3XXX:			strncpy(msg, "MIFARE TNP3XXX", MSG_BUF_MAX);
		case PICC_TYPE_NOT_COMPLETE:	strncpy(msg, "SAK indicates UID is not complete.", MSG_BUF_MAX);
		case PICC_TYPE_UNKNOWN:
		default:						strncpy(msg, "Unknown type", MSG_BUF_MAX);
	}
	return msg;
} // End PICC_GetTypeName()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
									byte sendLen,		///< Number of bytes to transfer to the FIFO.
									byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
									byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
									byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
											byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
											byte *sendData,		///< Pointer to the data to transfer to the FIFO.
											byte sendLen,		///< Number of bytes to transfer to the FIFO.
											byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
											byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
											byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
											byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
											bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteReg_Bit(CommandReg, PCD_Idle);				// Stop any active command.
	PCD_WriteReg_Bit(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteReg_Bit(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteReg_Bit(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteReg_Bit(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = 2000; i > 0; i--) {
		byte n = 0;
		n = PCD_ReadReg_Bit(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	byte errorRegValue = 0;
	errorRegValue = PCD_ReadReg_Bit(ErrorReg);	// ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	byte _validBits = 0;

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		byte n = 0;	
		n = PCD_ReadReg_Bit(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadReg_Bit(ControlReg);		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		_validBits = _validBits & 0x07;
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_RequestA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_WakeupA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum StatusCode PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
									byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
								) {
	byte validBits;
	enum StatusCode status;

	if (bufferATQA == RT_NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
								byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
								) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	enum StatusCode result = STATUS_OK;
	byte count;
	byte checkBit;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				// break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.s
			PCD_WriteReg_Bit(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = PCD_ReadReg_Bit(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum StatusCode PICC_HaltA(void) {
	enum StatusCode result;
	byte buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), RT_NULL, 0, false, 0, false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
enum StatusCode PCD_Authenticate(byte command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
								byte blockAddr, 	///< The block number. See numbering in the comments in the .h file.
								MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
								Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
								) {
	return STATUS_ERROR;
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteReg_Bit(TxModeReg, 0x00);
	PCD_WriteReg_Bit(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteReg_Bit(ModWidthReg, 0x26);

	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
bool PICC_ReadCardSerial() {
	enum StatusCode result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
} // End 

/////////////////////////////////////////////////////////////////////////////////////
// Other functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Get the uid object
 * 
 * @return Uid* 
 */
Uid *get_uid(void) {
	return &uid;
}

/**
 * @brief End of the program
 * 
 */
void PCD_End(void) {
	if (msg != RT_NULL)
		rt_free(msg);
}
