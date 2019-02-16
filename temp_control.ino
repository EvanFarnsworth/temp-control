#include <AutoPID.h>
#include <LiquidCrystal.h>
#include <MovingAverage.h>
#include <TimerOne.h>

#define aref_voltage 3.3 

#define OUTPUT_MIN 1.0
#define OUTPUT_MAX 79.0
#define KP 3
#define KI .05
#define KD 0.0

#define REAR_TEMP_PIN A0
#define REAR_FAN_CTRL_PIN 9 //OCR2B
#define REAR_FAN_SPD_PIN 2

#define FRONT_TEMP_PIN  A1
#define FRONT_FAN_CTRL_PIN 10 //OCR0B
#define FRONT_FAN_SPD_PIN 4


#define TEMP_READ_DELAY 500

unsigned long lastTempUpdate; //tracks clock time of last temp update
unsigned long rearTime;
unsigned long frontTime;
double rearTemp, frontTemp, rearFanOutput, frontFanOutput, setPoint;
MovingAverage rearAvg, frontAvg;

AutoPID rearPID(&rearTemp, &setPoint, &rearFanOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID frontPID(&frontTemp, &setPoint, &frontFanOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

LiquidCrystal lcd(13,6,12,11,3,8,7);

void setup()
{
   Timer1.initialize(40);
   
   //use negative setpoint for reverse acting
   setPoint = -30.0;
   
   // generate 25kHz PWM pulse rate on Pin 3
   //pinMode(REAR_FAN_CTRL_PIN, OUTPUT);   // OCR2B sets duty cycle
   //pinMode(FRONT_FAN_CTRL_PIN, OUTPUT);
   // Set up Fast PWM on Pin 3
   //TCCR2A = 0x23;     // COM2B1, WGM21, WGM20 
   // Set prescaler  
   //TCCR2B = 0x09;   // WGM21, Prescaler = /8
   // Set TOP and initialize duty cycle to zero(0)
   //OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
   //OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0 :
   
   //setup LCD
   pinMode(5, OUTPUT);
   analogWrite(5, 100); //contrast

   digitalWrite(REAR_FAN_SPD_PIN, HIGH);   // Starts reading
   digitalWrite(FRONT_FAN_SPD_PIN, HIGH);
   Serial.begin(9600);

   //change voltage reference as temps are using 3.3V
   analogReference(EXTERNAL);

   //setup PID parameters
   rearPID.setTimeStep(1000);
   frontPID.setTimeStep(1000);
   rearPID.setBangBang(10.0);
   frontPID.setBangBang(10.0);

   //setup moving average calculator for smoothing temperature readings
   rearAvg.reset(analogRead(REAR_TEMP_PIN));
   frontAvg.reset(analogRead(FRONT_TEMP_PIN));

   lcd.begin(16,2);
}
void loop()
{
  //i/o update
  updateTemperature();
  rearPID.run();
  frontPID.run();
  
  unsigned int rearSpd = getRPM(REAR_FAN_SPD_PIN);
  unsigned int frontSpd = getRPM(FRONT_FAN_SPD_PIN);

  float dutyCycle = 20.0;
  Serial.print("PWM Fan, Duty Cycle = ");
  Serial.println(dutyCycle);
  Timer1.pwm(FRONT_FAN_CTRL_PIN, (dutyCycle / 100) * 1023);
  Timer1.pwm(REAR_FAN_CTRL_PIN, (20.0 / 100) * 1023);
  
  lcd.clear();
  lcd.setCursor(0,0);
  int rearT = int(-round(rearTemp));
  int frontT = int(-round(frontTemp));
 
  lcd.print("rt:" + String(rearT));
  lcd.setCursor(8,0);
  lcd.print("ft:" + String(frontT));
  lcd.setCursor(0,1);
  lcd.print("f1:"+ String(rearSpd));
  lcd.setCursor(8,1);
  lcd.print("f2:" + String(frontSpd));  
  
  delay(2000);
  
}

bool updateTemperature() {
  //Set temperatures to negative such that the PID will control in 
  //a makeshift reverse action control. 
  
  //delay read of temperatures to every 500 msec
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    //get temperatures and convert to Celsius
    int rear = rearAvg.update(analogRead(REAR_TEMP_PIN));
    rearTemp = -get_analog_temp(rear);
    
    int front = frontAvg.update(analogRead(FRONT_TEMP_PIN));
    frontTemp = -get_analog_temp(front);
    
    lastTempUpdate = millis();
    
    return true;
  }
  return false;
}//void updateTemperature

float get_analog_temp(int value){
  float volts = (value * aref_voltage)/1024.0 ;
  return (volts - .5) * 100;
}

unsigned int getRPM(int pin) {
 unsigned long t = pulseIn(pin, HIGH);
 if(t ==0)
  return 0;
 unsigned int rpm = (1000000 * 60) / (t * 4);
 return rpm;
}

void printTempMessage(){
  lcd.setCursor(0,0);
  int rearT = int(-round(rearTemp));
  int frontT = int(-round(frontTemp));
 
  lcd.print("rt:" + String(rearT));
  lcd.setCursor(8,0);
  lcd.print("ft:" + String(frontT));
}

String printSpeedMessage(unsigned int rearSpeed, unsigned int frontSpeed){
  lcd.setCursor(0,1);
  lcd.print("f1:"+ String(rearSpeed));
  lcd.setCursor(8,1);
  lcd.print("f2:" + String(frontSpeed));  
  lcd.setCursor(0,0);
}
