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

  digitalWrite(RED_LED, HIGH);//turn off LED outputs
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  M2IO.Init_12VIO();//initialize 12VIO object

  avgBat=avgBatteryVoltage(3000);//measure battery voltage and idle voltage
  startCar(true);
  delay(3000);
  avgIdle=avgBatteryVoltage(3000);

  /*if((avgIdle-avgBat)<ALLOW_DELTA)//verify voltage data
  {
    SerialUSB.println("Idle and battery voltage too close, terminating");
    while(true)
    {
      digitalWrite(RED_LED, !digitalRead(RED_LED));
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      delay(100);
    }
  }*/

  SerialUSB.println("Setup complete");
}

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

void startCarSpecific()//unique to make, model, year
{
  //code to start car
}

void stopCarSpecific()//unique to make, model, year
{
  //code to stop car
}

void startCar(bool forceTime, bool forceCheck)
{
  SerialUSB.println("Calling startCar() with forceTime="+String(forceTime)+", forceCheck="+String(forceCheck));
  
  if(forceTime || (millis()-lastAction)>MIN_PERIOD)
  {
    for(int i=0; i<3; i++)
    {
      digitalWrite(GREEN_LED, LOW);
      delay(100);
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
    }
  
    startCarSpecific();//run the start code specific to the car being used
    
    /*delay(3000);
    if(!forceCheck && !carRunning())//check if car is running after sending start command
    {
      while(true)//error, lock into holding pattern
      {
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
        delay(100);
      }
    }*/
    
    digitalWrite(LED_BUILTIN, LOW);
    lastAction=millis();
  }
  else
  {
    SerialUSB.println("Acion frequency too high!");
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

void stopCar(bool forceTime, bool forceCheck)
{
  SerialUSB.println("Calling stopCar() with forceTime="+String(forceTime)+", forceCheck="+String(forceCheck));
  
  if(forceTime || (millis()-lastAction)>MIN_PERIOD)
  {
    for(int i=0; i<3; i++)
    {
      digitalWrite(RED_LED, LOW);
      delay(100);
      digitalWrite(RED_LED, HIGH);
      delay(100);
    }
  
    stopCarSpecific();//run the stop code specific to the car being used

    /*delay(3000);
    if(!forceCheck && carRunning())//check if car is still running after sending stop command
    {
      while(true)//error, lock into holding pattern
      {
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
        delay(100);
      }
    }*/
    
    digitalWrite(LED_BUILTIN, HIGH);
    lastAction=millis();
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
  float currentVoltage=readBatteryVoltage();
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
    sum++;//=readBatteryVoltage();
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

  //additional correction for M2 V4
  voltage=-.0168*voltage*voltage+1.003*voltage+1.3199;//calibration curve determined with DMM, assumed good (M2 V4 only!)

  return voltage;
}
