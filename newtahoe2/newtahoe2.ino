/*
  MCP2515 CAN Interface Using SPI
  
  Author: David Harding
  
  Created: 11/08/2010
  Modified: 6/26/12 by RechargeCar Inc.
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus

  The MCP2515 Library files also contain important information.
  
  This sketch is configured to work with the 'Macchina' Automotive Interface board 
  manufactured by RechargeCar Inc. CS_PIN and INT_PIN are specific to this board.
  
  This sketch shows the most basic of steps to send and receive CAN messages.
  
  NOTE!!!  If you use this sketch to test on a live system I suggest that you comment out the
  send messages lines unless you are certain that they will have no detrimental effect! 


  This example code is in the public domain.
  
*/

#include <SPI.h>
#include <MCP2515_sw_can.h>

// Pin definitions specific to how the MCP2515 is wired up.
#ifdef MACCHINA_M2
#define CS_PIN  SPI0_CS3
#define INT_PIN SWC_INT
#else
#define CS_PIN  34
#define INT_PIN 38
#endif

// Create CAN object with pins as defined
SWcan SCAN(CS_PIN, INT_PIN);

void CANHandler() {
	SCAN.intHandler();
}

CAN_FRAME wakeMessage;
CAN_FRAME initializeMessage;
CAN_FRAME startupMessage;
CAN_FRAME stopdownMessage;



void setup() {

  wakeMessage.id=0x100;
  wakeMessage.rtr=0;
  wakeMessage.extended=0;
  wakeMessage.length=8;
  wakeMessage.data.byte[0]=0x00;
  wakeMessage.data.byte[1]=0x00;
  wakeMessage.data.byte[2]=0x00;
  wakeMessage.data.byte[3]=0x00;
  wakeMessage.data.byte[4]=0x00;
  wakeMessage.data.byte[5]=0x00;
  wakeMessage.data.byte[6]=0x00;
  wakeMessage.data.byte[7]=0x00;
  
  initializeMessage.id=0x638;
  initializeMessage.rtr=0;
  initializeMessage.extended=0;
  initializeMessage.length=8;
  initializeMessage.data.byte[0]=0x01;
  initializeMessage.data.byte[1]=0x14;
  initializeMessage.data.byte[2]=0x00;
  initializeMessage.data.byte[3]=0x00;
  initializeMessage.data.byte[4]=0x23;
  initializeMessage.data.byte[5]=0x40;
  initializeMessage.data.byte[6]=0x05;
  initializeMessage.data.byte[7]=0x03;
  
  startupMessage.id=0x044097;
  startupMessage.rtr=0;
  startupMessage.extended=1;
  startupMessage.length=3;
  startupMessage.data.byte[0]=0x00;
  startupMessage.data.byte[1]=0xFF;
  startupMessage.data.byte[2]=0x0A;
  
  stopdownMessage.id=0x0080B0;
  stopdownMessage.rtr=0;
  stopdownMessage.extended=1;
  stopdownMessage.length=2;
  stopdownMessage.data.byte[0]=0x02;
  stopdownMessage.data.byte[1]=0x0C;
  
  delay(5000);
	SerialUSB.begin(115200);
	
	SerialUSB.println("Initializing ...");

	// Set up SPI Communication
	// dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
	
	// Initialize MCP2515 CAN controller at the specified speed and clock frequency
	// (Note:  This is the oscillator attached to the MCP2515, not the Arduino oscillator)
	//speed in KHz, clock in MHz
	SCAN.setupSW(33);
	
	attachInterrupt(INT_PIN, CANHandler, FALLING);
	SCAN.InitFilters(true);
    SCAN.mode(3); //3 = Normal, 2 = HV Wake up, 1 = High Speed, 0 = Sleep

	SerialUSB.println("Ready ...");

 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);

 delay(3000);
 startCar();

 digitalWrite(LED_BUILTIN, LOW);

 delay(10000);
 stopCar();

 digitalWrite(LED_BUILTIN, HIGH);
}

byte i=0;



void loop()
{
  
}

void startCar()
{
  SCAN.mode(2);
  delay(500);
  SCAN.EnqueueTX(wakeMessage);
  delay(500);
  SCAN.EnqueueTX(initializeMessage);
  delay(500);
  SCAN.mode(3);
  delay(500);
  SCAN.EnqueueTX(startupMessage);
  delay(500);
}

void stopCar()
{
  SCAN.mode(2);
  delay(500);
  SCAN.EnqueueTX(wakeMessage);
  delay(500);
  SCAN.EnqueueTX(initializeMessage);
  delay(500);
  SCAN.mode(3);
  delay(500);
  SCAN.EnqueueTX(stopdownMessage);
  delay(500);
}
