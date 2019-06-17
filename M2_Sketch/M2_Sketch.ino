//Macchina LLC
//Kenny Truong
//kenny@macchina.cc
//6-17-19

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

#include <pwm_defs.h>
#include <pwm_lib.h>

#include <Debounce.h>
#include <M2_12VIO.h>

M2_12VIO M2IO;

const byte RED_LED=14;
const byte GREEN_LED=18;

const unsigned int MIN_PERIOD=5000;//set a minimum time in ms between actions (an action is starting or stopping the car)
unsigned long lastAction=0;//record time last action was performed

const float ALLOW_DELTA=0.5;//minimum allowable difference between battery and idle voltage

float avgBat=0;
float avgIdle=0;

void setup()
{
  SerialUSB.begin(115200);
  
  pinMode(XBEE_MULT2, INPUT);//initialize input from SuperB
  
  pinMode(RED_LED, OUTPUT);//initialize LED outputs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(Button1, INPUT_PULLUP);//initialize button inputs
  pinMode(Button2, INPUT_PULLUP);

  digitalWrite(RED_LED, HIGH);//turn off LED outputs
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  SerialUSB.println("I/O initialized");

  M2IO.Init_12VIO();//initialize 12VIO object

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

  SerialUSB.println("Measuring battery voltage");//measure battery voltage and idle voltage
  avgBat=avgBatteryVoltage(3000);
  startCar(true);
  delay(3000);
  SerialUSB.println("Measuring idle voltage");
  avgIdle=avgBatteryVoltage(3000);

  SerialUSB.println("Verifying voltage data");
  if((avgIdle-avgBat)<ALLOW_DELTA)//verify voltage data
  {
    SerialUSB.println("Idle and battery voltage too close, freezing");
    freeze();
  }

  SerialUSB.println("Setup complete");
}

byte i=0;

void loop()
{
  while(!carRunning())//watch for command to start car while car is not running
  {
    if(digitalRead(XBEE_MULT2)==HIGH)
    {
      startCar();
    }
  }
  while(carRunning())//watch for command to stop car while car is running
  {
    if(digitalRead(XBEE_MULT2)==LOW)
    {
      stopCar();
    }
  }
}

CAN_FRAME message;

void startCarSpecific()//GM, chevy code
{
  SerialUSB.println("Calling startCarSpecific()");

  SCAN.mode(2);
  message.id = 0x100;//wake
  message.rtr = 0;
  message.extended = 0;
  message.length = 8;
  message.data.byte[0] = 0x00;
  message.data.byte[1] = 0x00;
  message.data.byte[2] = 0x00;
  message.data.byte[3] = 0x00;
  message.data.byte[4] = 0x00;
  message.data.byte[5] = 0x00;
  message.data.byte[6] = 0x00;
  message.data.byte[7] = 0x00;
  SCAN.EnqueueTX(message);
  delay(2000);
  
  message.id = 0x638;//initialize
  message.rtr = 0;
  message.extended = 0;
  message.length = 8;
  message.data.byte[0] = 0x01;
  message.data.byte[1] = 0x14;
  message.data.byte[2] = 0x00;
  message.data.byte[3] = 0x00;
  message.data.byte[4] = 0x23;
  message.data.byte[5] = 0x40;
  message.data.byte[6] = 0x05;
  message.data.byte[7] = 0x03;
  SCAN.EnqueueTX(message);
  delay(2000);

  SCAN.mode(3);
  message.id = 0x044097;//startup
  message.rtr = 0;
  message.extended = 1;
  message.length = 3;
  message.data.byte[0] = 0x00;
  message.data.byte[1] = 0xFF;
  message.data.byte[2] = 0x0A;
  SCAN.EnqueueTX(message); //send it
  delay(5); //wait a bit to make sure it was sent before ending HV Wake up
  SCAN.mode(3); //go back to normal mode
  delay(2000);
}

void stopCarSpecific()//GM, chevy code
{
  SerialUSB.println("Calling stopCarSpecific()");

  SCAN.mode(2);
  message.id = 0x100;//wake
  message.rtr = 0;
  message.extended = 0;
  message.length = 8;
  message.data.byte[0] = 0x00;
  message.data.byte[1] = 0x00;
  message.data.byte[2] = 0x00;
  message.data.byte[3] = 0x00;
  message.data.byte[4] = 0x00;
  message.data.byte[5] = 0x00;
  message.data.byte[6] = 0x00;
  message.data.byte[7] = 0x00;
  SCAN.EnqueueTX(message);
  delay(2000);
  
  message.id = 0x638;//initialize
  message.rtr = 0;
  message.extended = 0;
  message.length = 8;
  message.data.byte[0] = 0x01;
  message.data.byte[1] = 0x14;
  message.data.byte[2] = 0x00;
  message.data.byte[3] = 0x00;
  message.data.byte[4] = 0x23;
  message.data.byte[5] = 0x40;
  message.data.byte[6] = 0x05;
  message.data.byte[7] = 0x03;
  SCAN.EnqueueTX(message);
  delay(2000);

  SCAN.mode(3);
  message.id = 0x0080B0;//stopdown
  message.rtr = 0;
  message.extended = 1;
  message.length = 2;
  message.data.byte[0] = 0x02;
  message.data.byte[1] = 0x0C;
  SCAN.EnqueueTX(message); //send it
  delay(5); //wait a bit to make sure it was sent before ending HV Wake up
  SCAN.mode(3); //go back to normal mode
  delay(2000);
}

//attempt to start the car
//if forceTime=false, time since last action must be greater than MIN_PERIOD
//if forceCheck=false, program will freeze after starting car and detecting car did not start
//true values bypass checks
void startCar(bool forceTime, bool forceCheck)
{
  SerialUSB.println("Calling startCar() with forceTime="+String(forceTime)+", forceCheck="+String(forceCheck));
  
  if(forceTime || (millis()-lastAction)>MIN_PERIOD)//TIMECHECK: time since last action must be greater than MIN_PERIOD or forceTime must be true
  {
    SerialUSB.println("TIMECHECK passed");
    for(int i=0; i<3; i++)
    {
      digitalWrite(GREEN_LED, LOW);
      delay(100);
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
    }
  
    startCarSpecific();//run the start code specific to the car being used
    
    delay(3000);
    SerialUSB.println("startCarSpecific() called, checking if car is running");
    if(!forceCheck && !carRunning())//RUNCHECK: car must be running after calling startCarSpecific() or forceCheck must be true
    {
      SerialUSB.println("Car not running, freezing program");
      freeze();
    }
    SerialUSB.println("RUNCHECK passed");
    
    digitalWrite(LED_BUILTIN, LOW);
    lastAction=millis();
    SerialUSB.println("Exiting startCar()");
  }
  else
  {
    SerialUSB.println("Action frequency too high!");
  }
}

void startCar(bool force)
{
  startCar(force, force);
}

void startCar()
{
  startCar(false, false);
}

//attempt to stop the car
//if forceTime=false, time since last action must be greater than MIN_PERIOD
//if forceCheck=false, program will freeze after starting car and detecting car did not start
//true values bypass checks
void stopCar(bool forceTime, bool forceCheck)
{
  SerialUSB.println("Calling stopCar() with forceTime="+String(forceTime)+", forceCheck="+String(forceCheck));
  
  if(forceTime || (millis()-lastAction)>MIN_PERIOD)//TIMECHECK: time since last action must be greater than MIN_PERIOD or forceTime must be true
  {
    SerialUSB.println("TIMECHECK passed");
    for(int i=0; i<3; i++)
    {
      digitalWrite(RED_LED, LOW);
      delay(100);
      digitalWrite(RED_LED, HIGH);
      delay(100);
    }
  
    stopCarSpecific();//run the stop code specific to the car being used

    delay(3000);
    SerialUSB.println("stopCarSpecific() called, checking if car is stopped");
    if(!forceCheck && carRunning())//RUNCHECK: car must be stopped after calling stopCarSpecific() or forceCheck must be true
    {
      SerialUSB.println("Car still running, freezing program");
      freeze();
    }
    
    digitalWrite(LED_BUILTIN, HIGH);
    lastAction=millis();
    SerialUSB.println("Exiting stopCar()");
  }
  else
  {
    SerialUSB.println("Acion frequency too high!");
  }
}

void stopCar(bool force)
{
  stopCar(force, force);
}

void stopCar()
{
  stopCar(false, false);
}

bool carRunning()//determine whether the car is running by measuring the voltage
{
  float currentVoltage=avgBatteryVoltage(10);
  float threshold=(avgBat+avgIdle)/2;

  return currentVoltage>threshold;
}

float avgBatteryVoltage(unsigned long timespan)//returns the average battery voltage over a timespan in ms
{
  float dataPoints=0;
  float sum=0;

  unsigned long startTime=millis();

  while((millis()-startTime)<timespan)
  {
    sum+=readBatteryVoltage();
    dataPoints++;
    delay(1);
  }

  return (sum/dataPoints);
}

float readBatteryVoltage()//returns the battery voltage, known good
{
  float voltage=M2IO.Supply_Volts();
  voltage=voltage/1000;
  voltage=.1795*voltage*voltage-2.2321*voltage+14.596;//calibration curve determined with DSO, assumed good

  //additional correction for M2 V4, comment out line below for V3 or earlier
  voltage=-.0168*voltage*voltage+1.003*voltage+1.3199;//calibration curve determined with DMM, assumed good (M2 V4 only!)

  return voltage;
}

void freeze()//freeze the program and flash the LEDs to indicate a problem has occured, thaw and return to program by simultaneously pressing SW1 and SW2
{
  SerialUSB.println("Entering freeze()");

  unsigned long lastTime=millis();
  unsigned long period=200;
  period=period/2;

  bool thaw=false;
  while(!thaw)
  {
    digitalWrite(RED_LED, !digitalRead(RED_LED));
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
    
    while((millis()-lastTime)<period)
    {
      if(digitalRead(Button1)==LOW && digitalRead(Button2)==LOW)
      {
        thaw=true;
      }
    }
    lastTime=millis();
  }

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  SerialUSB.println("Thawing...");
  SerialUSB.println("3...");
  delay(1000);
  SerialUSB.println("2...");
  delay(1000);
  SerialUSB.println("1...");
  delay(1000);
}