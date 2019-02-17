#include <AutoPID.h>
#include <LiquidCrystal.h>
#include <MovingAverage.h>
#include <TimerOne.h>

#define aref_voltage 3.3 

//Used for PID control
#define OUTPUT_MIN 0.0
#define OUTPUT_MAX 100.0
#define KP 3
#define KI .05
#define KD 0.0
#define DEADBAND 1.0

#define EXHAUST_TEMP_PIN A0
#define EXHAUST_FAN_CTRL_PIN 9 
#define EXHAUST_FAN_SPD_PIN 2

#define INTAKE_TEMP_PIN  A1
#define INTAKE_FAN_CTRL_PIN 10 
#define INTAKE_FAN_SPD_PIN 4

#define TEMP_READ_DELAY 500

unsigned long lastTempUpdate; //tracks clock time of last temp update
unsigned long exhaustTime;
unsigned long intakeTime;
double exhaustTemp, intakeTemp, exhaustFanOutput, intakeFanOutput, exhaustSetPoint, intakeSetPoint;
int potValue;
MovingAverage exhaustAvg, intakeAvg;

AutoPID exhaustPID(&exhaustTemp, &exhaustSetPoint, &exhaustFanOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID intakePID(&intakeTemp, &intakeSetPoint, &intakeFanOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

LiquidCrystal lcd(13,6,12,11,3,8,7);

void setup()
{
   Timer1.initialize(40);
   
   //use negative setpoint for reverse acting
   exhaustSetPoint = -29.0;
   intakeSetPoint = -31.0; 
   //setup LCD
   pinMode(5, OUTPUT);
   analogWrite(5, 100); //contrast
    
   digitalWrite(EXHAUST_FAN_SPD_PIN, HIGH);   // Starts reading
   digitalWrite(INTAKE_FAN_SPD_PIN, HIGH);
   Serial.begin(9600);

   //change voltage reference as temps are using 3.3V
   analogReference(EXTERNAL);

   //setup PID parameters
   exhaustPID.setTimeStep(1000);
   intakePID.setTimeStep(1000);
   exhaustPID.setBangBang(5.0);
   intakePID.setBangBang(4.0);

   //setup moving average calculator for smoothing temperature readings
   exhaustAvg.reset(analogRead(EXHAUST_TEMP_PIN));
   intakeAvg.reset(analogRead(INTAKE_TEMP_PIN));

   lcd.begin(16,2);
}
void loop()
{
  //potValue = analogRead(A5);
  Serial.println("Pot Value: " + String(potValue));
  
  //i/o update
  updateTemperature();
  exhaustPID.run();
  intakePID.run();

  //read in fan RPMs
  unsigned int exhaustSpd = getRPM(EXHAUST_FAN_SPD_PIN);
  unsigned int intakeSpd = getRPM(INTAKE_FAN_SPD_PIN);

  //set ouput signal to the fans. only if its outside deadband
  if(!intakePID.atSetPoint(DEADBAND)){
    Timer1.pwm(INTAKE_FAN_CTRL_PIN, (intakeFanOutput / 100) * 1023);
  }
  if(!exhaustPID.atSetPoint(DEADBAND)){
    Timer1.pwm(EXHAUST_FAN_CTRL_PIN, (exhaustFanOutput / 100) * 1023);
  }
  lcd.clear();
  lcd.setCursor(0,0);
  int exhaustT = int(-round(exhaustTemp));
  int intakeT = int(-round(intakeTemp));
 
  lcd.print("exT:" + String(exhaustT));
  lcd.setCursor(8,0);
  lcd.print("inT:" + String(intakeT));
  lcd.setCursor(0,1);
  lcd.print("EF:"+ String(exhaustSpd));
  lcd.setCursor(8,1);
  lcd.print("IF:" + String(intakeSpd));  
  
  delay(2000);
}

//grab temperatures
bool updateTemperature() {
  //Set temperatures to negative such that the PID will control in 
  //a makeshift reverse action control. 
  //delay read of temperatures to every 500 msec
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    //get temperatures and convert to Celsius
    int exhaust = exhaustAvg.update(analogRead(EXHAUST_TEMP_PIN));
    //Serial.println("Exhaust Voltage: " + String(exhaust));
    exhaustTemp = -get_analog_temp(exhaust); // - ((30 * potValue)/1023);
    
    int intake = intakeAvg.update(analogRead(INTAKE_TEMP_PIN));
    intakeTemp = -get_analog_temp(intake);// - ((30 * potValue)/1023);
    
    lastTempUpdate = millis();
    
    return true;
  }
  return false;
}//void updateTemperature

//helper to do the conversion of input to temperature
float get_analog_temp(int value){
  float volts = (value * aref_voltage)/1024.0 ;
  return (volts - .5) * 100;
}

unsigned int getRPM(int pin) {
 unsigned long t = pulseIn(pin, HIGH);
 if(t ==0) //prevent 0 division (output of 65k)
  return 0;
 unsigned int rpm = (1000000 * 60) / (t * 4);
 return rpm;
}
