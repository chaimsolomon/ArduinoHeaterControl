#include <EEPROM.h>
#include <TimerOne.h>
#include <Adafruit_SleepyDog.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Connect to a Arduino Nano compatible device:
// Generic I2C-LCD bridge to I2C pins
// DS18B20 sensor (1Wire) to Pin 5 (wire a 4k7 as pull-up to VCC (5V) to this pin too, wire 5V to the VCC pin of the sensor)
// Generic 4 channel relay card on Pin D9-D12
// Buttons on Pin D6-D8

// Addresses in EEPROM for persistant storage
#define EE_ON_OFF 0
// Setpoint start address - this is a float, so it spans multiple bytes
#define EE_SP 3

// PID controller variables
double Setpoint, Input, Output;
// kP and kI parameters for controller
double kp=75, ki=0.5;

// General device state - on or off
int device_on = 0;
// Counter for PWM
int pwm_counter=0;
// Counters for button press - as long as the buttons are pressed, their counter is incremented (until max_button)
int button_counter0=0, button_counter1=0, button_counter2=0;
const int max_button = 100;
// Counter for keeping the fan on for a while after turning the device 
int turnoff_counter = 0;

// PID controller
PID myPID(&Input, &Output, &Setpoint, 75, 0.5, 0, DIRECT);

LiquidCrystal_I2C lcd(0x27,16,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

OneWire oneWire(5);
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress;

// ISR for timer interrupt
void callback()
{
  if (device_on == 1) {  
    digitalWrite(11, LOW);
    if (Output > 100) {
      if (pwm_counter > Output-100)
      {
        digitalWrite(10, LOW);//On
        digitalWrite(9, LOW);//Off
      }
      else
      {
        digitalWrite(10, LOW); //On
        digitalWrite(9, HIGH);//On
      }
    }
  else if (pwm_counter >= Output )
    {
      digitalWrite(11, LOW);
    digitalWrite(10, HIGH); //Off
      digitalWrite(9, LOW); //Off
    }
    else
    {
      digitalWrite(11, LOW);
      digitalWrite(10, HIGH); //Off
      digitalWrite(9, HIGH); // On
    }
  }
  else {
      digitalWrite(9, LOW);//Off
      digitalWrite(10, HIGH);//Off
      if (turnoff_counter > 0) {
        digitalWrite(11, LOW);//On
      }
      else
      {
        digitalWrite(11, HIGH);//Off
      }
      digitalWrite(12, HIGH);//Off
  }
  pwm_counter += 1;
  if (pwm_counter > 100) pwm_counter = 0;
  if (turnoff_counter > 0) turnoff_counter--;
 
  if (digitalRead(6) == 1 )
    {button_counter0 += 1; if (button_counter0 > max_button) button_counter0 = max_button; } 
    else { button_counter0 = 0;}
  if (digitalRead(7) == 1)
    {button_counter1 += 1; if (button_counter1 > max_button) button_counter1 = max_button; } 
    else { button_counter1 = 0;}
  if (digitalRead(8) == 1)
    {button_counter2 += 1; if (button_counter2 > max_button) button_counter2 = max_button; } 
    else { button_counter2 = 0;}

}

void save_setpoint(void)
{
  EEPROM.put(EE_SP, Setpoint);
}

void load_setpoint(void) {
  //Setpoint = 23;
  EEPROM.get(EE_SP, Setpoint);
}

void setup()
{
  Timer1.initialize(100000);         // initialize timer1
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer interrupt SR
  device_on =  EEPROM.read(EE_ON_OFF); // Initialize with values from EEPROM
  load_setpoint();
  int countdownMS = Watchdog.enable(20000); // Enable watchdog
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("HeaterControl");
  lcd.setCursor(0,1);
  lcd.print("Sensors");

  // Temperature sensor initialization
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);

  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a splash screen message to the LCD.
  lcd.setCursor(0,1);
  lcd.print("Initializing");
  lcd.setCursor(0,0);
  
//LED
pinMode(13, OUTPUT);
//Relay
pinMode(12, OUTPUT);
pinMode(11, OUTPUT);
pinMode(10, OUTPUT);
pinMode(9, OUTPUT);
//Buttons
pinMode(8, INPUT);
pinMode(7, INPUT);
pinMode(6, INPUT);

sensors.requestTemperatures(); // Send the command to get temperatures
Input = sensors.getTempCByIndex(0);
 //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 200);
  myPID.SetSampleTime(5000);
  myPID.SetTunings(kp,ki,0);
}


void loop()
{
sensors.requestTemperatures(); // Send the command to get temperatures
Input = sensors.getTempCByIndex(0);
if (device_on == 1) {
  myPID.Compute();
}
// First line: Input temperature, Set temperature and ON/OFF
lcd.clear();
lcd.setCursor(0,0);
lcd.print(Input);
lcd.setCursor(6,0);
lcd.print(Setpoint);
lcd.setCursor(12,0);
if (device_on == 1) {
  lcd.print("ON");
} 
else
{
  lcd.print("OFF");
}

// Second line: Output value of PID, kP, kI and the state (0: on/off, 1: temperature setpoint, 2: kP, 3: kI)
lcd.setCursor(0,1);
lcd.print(Output);
lcd.setCursor(7,1);
//lcd.print(kp);
//lcd.setCursor(12,1);
lcd.print(digitalRead(6));

// Buttons
if (button_counter0 > 2) {
  button_counter0 = 0; 
  if (device_on == 0)
  {
    device_on = 1;
  }
  else
  {
    device_on = 0;
    Output = 0;
    turnoff_counter = 10000;
  };
  EEPROM.update(EE_ON_OFF, device_on);
  } 
if (button_counter1 > 2) {button_counter1 = 0; Setpoint += 0.25; save_setpoint();} 
if (button_counter2 > 2) {button_counter2 = 0; Setpoint -= 0.25; save_setpoint();} 

Watchdog.reset();
}
