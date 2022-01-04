/************************************************************/
/************************************************************/
/*                                                          */
/*                                                          */
/* The Internet of Things: Communication and Cloud          */
/*                                                          */
/*                                                          */
/* Author: Nagaraju Seelam                                  */
/*                                                          */
/* Assignment :  Project 1 - Fancy lighting System          */
/*                                                          */
/*                                                          */
/* File name: Fancy_lighting                                */
/*                                                          */
/*                                                          */
/* Objective:                                               */
/*  Implementing an Automotive fancy lighting system        */
/*  based on LDR, PotentioMeter and Push buttons            */
/*                                                          */
/************************************************************/

/************************************************************/
#include <Timer.h>

#define PIN_RED   11
#define PIN_GREEN 10
#define PIN_BLUE  9

#define PIN_SWITCH  2   // switch 1
#define PIN_COLOR   3   // switch 2

#define PIN_LIGHT   A0
#define PIN_POT     A1


#define	NUM_COLORS	 	6
float colors[NUM_COLORS][3] = {{1.0,0.0,0.0}, {1.0,1.0,0.0}, {0.0,1.0,0.0}, {0.0,1.0,1.0}, {0.0,0.0,1.0}, {1.0,0.0,1.0}};

#define NUM_COLORS3  21
float colors3[NUM_COLORS3][3] =  {{0.3,0.0,0.0},{0.7,0.0,0.0},{1.0,0.0,0.0}, 
                {0.7,0.3,0.0},{0.3,0.7,0.0},{0.0,1.0,0.0}, 
                {0.0,0.7,0.3},{0.0,0.3,0.7},{0.0,0.0,1.0}, 
                {0.3,0.3,0.7},{0.7,0.7,0.3},{1.0,1.0,0.0}, 
                {1.0,0.7,0.3},{1.0,0.3,0.7},{1.0,0.0,1.0},
                {0.7,0.3,1.0},{0.3,0.7,1.0},{0.0,1.0,1.0},
                {0.3,1.0,1.0},{0.7,1.0,1.0},{1.0,1.0,1.0}};
                  
                 
volatile boolean ledState=LOW;
volatile int currentColor = 0;
volatile int previousColor = 0;

#define NUM_STEPPING  100
volatile int stepping=0;

// cut-off value for light control
int lightCutOff, PotCutOff;

Timer timer;

void setLedState(void *);
void checkPhotoSensor(void *);
void checkPotentioMeter(void);
void PotCalibration(void);

#define	SET_LED_STATE_PERIOD          100
#define	CHECK_PHOTO_SENSOR_PERIOD	  200
#define	CHECK_POTENTIO_SENSOR_PERIOD  300

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void setup() 
{
   Serial.begin(115200);

    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    pinMode(PIN_COLOR,  INPUT_PULLUP);
    pinMode(PIN_LIGHT,  INPUT);
    
    attachInterrupt(0, buttonSwitchPushed, FALLING); // FALLING // RISING
    attachInterrupt(1, buttonColorPushed,  FALLING); // RISING
    
    lightCalibration();
    PotCalibration();
    
    
    timer.every(SET_LED_STATE_PERIOD, setLedState, NULL);
	timer.every(CHECK_PHOTO_SENSOR_PERIOD, checkPhotoSensor, NULL);
	timer.every(CHECK_POTENTIO_SENSOR_PERIOD, checkPotentioMeter, NULL);
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void loop() 
{
  timer.update(); 
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void setLedState(void *) 
{
  if (!ledState) 
  {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE, 0);
    return;
  }

  // calculate change required for each stepping for each color
  float deltaRed    = (colors[currentColor][0] - colors[previousColor][0]) / NUM_STEPPING;
  float deltaGreen  = (colors[currentColor][1] - colors[previousColor][1]) / NUM_STEPPING;
  float deltaBlue   = (colors[currentColor][2] - colors[previousColor][2]) / NUM_STEPPING;

  analogWrite(PIN_RED, (colors[currentColor][0]   - deltaRed * stepping) *255);
  analogWrite(PIN_GREEN, (colors[currentColor][1] - deltaGreen * stepping) *255);
  analogWrite(PIN_BLUE, (colors[currentColor][2]  - deltaBlue * stepping) *255);
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void checkPhotoSensor(void *)
{
  static boolean sensorSwitch=false;
  
  int sensorValue = analogRead(PIN_LIGHT);
  boolean newState;
  
  if (sensorValue < lightCutOff)
    newState = true;
  else
    newState = false;
    
  if (sensorSwitch == false && newState == true) {
    Serial.println("light sensor turn on light");
    ledState = true;
  }
  
  if (sensorSwitch == true && newState == false) {
    Serial.println("light sensor turn off light");
    ledState = false;
  }
  
  sensorSwitch = newState;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void checkPotentioMeter(void *) 
{
  static boolean PotSwitch=false;
  
  int PotValue = analogRead(PIN_LIGHT);
  boolean newState;
  
  if (PotValue < PotCutOff)
    newState = true;
  else
    newState = false;
    
  if (PotSwitch == false && newState == true) {
    Serial.println("Potentiometer turn on light");
    ledState = true;
  }
  
  if (PotSwitch == true && newState == false) {
    Serial.println("Potentiometer turn off light");
    ledState = false;
  }
  
  PotSwitch = newState;
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void buttonSwitchPushed(void) 
{
  static long lastTime = 0;
  
  if (millis() - lastTime < 200) return;
  lastTime = millis();
  
  Serial.println("Push button toggle light");
    ledState = !ledState;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void buttonColorPushed(void) 
{
  static long lastTime = 0;
  
  // don't change color when light is not on or we are in the middle of stepping
  if (!ledState || stepping>0) return;
  
  if (millis() - lastTime < 20) return;
  lastTime = millis();
  
  Serial.println("Push button changes color");
  previousColor = currentColor;
  if (++currentColor == NUM_COLORS)
    currentColor = 0;
  
  // we gradually change color in 2 seconds, and in NUM_STEPPING steps
  stepping = NUM_STEPPING;
  timer.every(2000/NUM_STEPPING, decreaseStepping, NUM_STEPPING, (void*)0);
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void decreaseStepping(void *)
{
  stepping --;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void lightCalibration(void) 
{ 
  int sensorHigh=0;
  int sensorLow = 10000;
  int sensorValue;
  
  unsigned long currTime = millis();

  Serial.println("Please vary light condition to start light calibration");
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // calibrate for five seconds
    while (millis() < currTime + 5000) 
  {
      sensorValue = analogRead(PIN_LIGHT);
      if (sensorValue > sensorHigh) {
          sensorHigh = sensorValue;
      }
      if (sensorValue < sensorLow) {
          sensorLow = sensorValue;
      }
      delay(100);
    }
    digitalWrite(13, LOW);  
    lightCutOff = (sensorLow + sensorHigh) /2;
    Serial.print("Light cutoff value is ");
    Serial.println(lightCutOff);
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void PotCalibration(void) 
{ 
  int PotHigh=0;
  int PotLow = 10000;
  int PotValue;
  
  unsigned long currTime = millis();

  Serial.println("Please vary PotentioMeter to start POT calibration : ");
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // calibrate for five seconds
  while (millis() < currTime + 5000) 
  {
      PotValue = analogRead(PIN_POT);
      if (PotValue > PotHigh) {
          PotHigh = PotValue;
      }
      if (PotValue < PotLow) {
          PotLow = PotValue;
      }
      delay(100);
    }
    digitalWrite(13, LOW);  
    PotCutOff = (PotLow + PotHigh) /2;
    Serial.print("POT cutoff value is ");
    Serial.println(PotCutOff);
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/




