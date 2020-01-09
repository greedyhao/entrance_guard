/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read new NUID from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to the read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the type, and the NUID if a new card has been detected. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 */

#include "mfrc522.h"

#define MFRC522_SS_PIN 	GET_PIN(H, 3)
#define MFRC522_RST_PIN GET_PIN(H, 8)

MIFARE_Key key;
Uid *uid;

// Init array that will store new NUID 
byte nuidPICC[4];

void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);

void setup() {
	MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN); // Instance of the class
	PCD_Init(); // Init MFRC522
	uid = get_uid();

	for (byte i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
	}

	rt_kprintf("This code scan the MIFARE Classsic NUID.\n");
	rt_kprintf("Using the following key:");
	printHex(key.keyByte, MF_KEY_SIZE);

	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( ! PICC_IsNewCardPresent())
		return;

	// Verify if the NUID has been readed
	if ( ! PICC_ReadCardSerial())
		return;

	rt_kprintf("PICC type: ");
	enum PICC_Type piccType = PICC_GetType(uid->sak);
	rt_kprintf("%s\n", PICC_GetTypeName(piccType));

	// Check is the PICC of Classic MIFARE type
	if (piccType != PICC_TYPE_MIFARE_MINI &&  
		piccType != PICC_TYPE_MIFARE_1K &&
		piccType != PICC_TYPE_MIFARE_4K) {
		rt_kprintf("Your tag is not of type MIFARE Classic.\n");
		return;
	}

	if (uid->uidByte[0] != nuidPICC[0] || 
		uid->uidByte[1] != nuidPICC[1] || 
		uid->uidByte[2] != nuidPICC[2] || 
		uid->uidByte[3] != nuidPICC[3] ) {
		rt_kprintf("A new card has been detected.\n");

		// Store NUID into nuidPICC array
		for (byte i = 0; i < 4; i++) {
			nuidPICC[i] = uid->uidByte[i];
		}

		rt_kprintf("The NUID tag is:\n");
		rt_kprintf("In hex: ");
		printHex(uid->uidByte, uid->size);
		rt_kprintf("\n");
		rt_kprintf("In dec: ");
		printDec(uid->uidByte, uid->size);
		rt_kprintf("\n");
	}
	else rt_kprintf("Card read previously.\n");

	// Halt PICC
	PICC_HaltA();

	// Stop encryption on PCD
	PCD_StopCrypto1();

	rt_free(uid);
}


/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
	for (byte i = 0; i < bufferSize; i++) {
		rt_kprintf(buffer[i] < 0x10 ? " 0" : " ");
		rt_kprintf("%x", buffer[i]);
	}
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
	for (byte i = 0; i < bufferSize; i++) {
		rt_kprintf(buffer[i] < 0x10 ? " 0" : " ");
		rt_kprintf("%d", buffer[i]);
	}
}
