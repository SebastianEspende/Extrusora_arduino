//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.
#include <ClickEncoder.h>
#include <TimerOne.h>

//Thermistor needed libraries
#include <Configuration.h>
#include <thermistor.h>           //Download it here: https://electronoobs.com/eng_arduino_thermistor.php
thermistor therm1(A0,0);          //Connect thermistor on A0, 0 represents TEMP_SENSOR_0 ( configuration.h for more)


//I/O
int PWM_pin = 10;                  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int speed_pot = A1;
int but1 = 13;
int EN = 2;
int STEP = 4;
int DIR = 5;


//Variables
float set_temperature = 0;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0.0;
int max_PWM = 255;
float last_temperature_read = 0.0;

//Stepper Variables
int max_speed = 200;
int main_speed = 0;
bool but1_state = true;
bool activate_stepper = false;
int rotating_speed = max_speed/10;
int last_set_velocidad;


//ENCODER
ClickEncoder *encoder;
volatile int16_t last, value;
int CLK = 8;
int DT = 7;
int SW = 6;
boolean up = false;
boolean down = false;
boolean middle = false;

#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper stepper1(1, STEP, DIR); // (Type of driver: with 2 pins, STEP, DIR)


//MENU
int frame = 0;
int lastframe = -1;
int estado;
int page = 0;
int lastpage = 0;
int itemmenu = 0;
int numerodeframes = 5;

//PID constants
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;



void setup() {  
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);     //Stepper driver is disbled
  stepper1.setMaxSpeed(max_speed);  
  pinMode(but1, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  
  pinMode(PWM_pin,OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000010;    // D6 adn D6 PWM frequency of 7812.50 Hz
  Time = millis();

  TCCR1A = 0;             //Reset entire TCCR1A register
  TCCR1B = 0;             //Reset entire TCCR1B register
  TCCR1A |= B00000010;    //   /8
  TCNT1 = 0;              //Reset Timer 1 value to 0
  Serial.begin(9600);
  encoder = new ClickEncoder(DT, CLK, SW, 4, false);
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  lcd.init();
  lcd.backlight();  
  last = -1;
  }

void loop() {

  
  if(activate_stepper){
    delay(10);
    digitalWrite(EN, LOW);    //We activate stepper driver
    Serial.println(rotating_speed);
    stepper1.setSpeed(rotating_speed); 
    stepper1.runSpeed();
  }
  else
  {
    digitalWrite(EN, HIGH);    //We deactivate stepper driver
    stepper1.setSpeed(0);
  }
  // First we read the real value of temperature
  temperature_read = therm1.analog2temp(); // read temperature
  
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read + 6;
  //Calculate the P value
  PID_p = 0.01*kp * PID_error;
  //Calculate the I value in a range on +-6
  PID_i = 0.01*PID_i + (ki * PID_error);
  
  
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  
  //We define PWM range between 0 and 255
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > max_PWM){
    PID_value = max_PWM;
  }
  
  //Now we can write the PWM signal to the mosfet on digital pin D5
  analogWrite(PWM_pin, PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
  
  middle = boton();
  if (page == 0) {
  frame = cambiarframe();
  }
    if (frame != lastframe) {lastframe = frame;}
    if (middle) {
      displayitem();
      } else {
        page = 0;
      }  
      
    stepper1.runSpeed();
}//Void loop end

int cambiarframe() {
    if(up && frame >= 0) {
      up=false;
      lastframe = frame;
      frame++;
      }
      if (frame >= numerodeframes) {
        lastframe = frame;
        frame = numerodeframes;
       }
    if(down && frame <= numerodeframes) {
      down = false;
      lastframe = frame;
      frame--;
      }
      if (frame < 0) {
        lastframe = frame;
        frame = 0;
       }
       mainmenu();
    return frame;
  }
void displayitem() {
  switch(frame) {
    case 0:
    if (value != last_set_temperature || estado == 0) {
      lcd.clear();
      page = 1;
      estado = 1;
      if (value > last_set_temperature) {
      set_temperature += 10;
      } else if (value < last_set_temperature) {
        set_temperature -= 10;
        }
      if (set_temperature > 250) set_temperature = 250;
      last_set_temperature = value;
      lcd.print("Temperatura");
      lcd.setCursor(0,1);
      lcd.print("Valor: ");
      lcd.print(set_temperature);
    }
      break;
      
    case 1:
    if (value != last_set_velocidad || estado == 0) {
      lcd.clear();
      page = 2;
      estado = 1;
      if (value > last_set_velocidad) {
      rotating_speed += 10;
      } else if (value < last_set_velocidad) {
        rotating_speed -= 10;
        }
      if (rotating_speed > max_speed) rotating_speed = max_speed;
      last_set_velocidad = value;
      lcd.print("Velocidad");
      lcd.setCursor(0,1);
      lcd.print("Valor: ");
      lcd.print(rotating_speed);
    }
      break;
   case 3:
   if(set_temperature != 200 || rotating_speed != 500)  {
   lcd.clear();
   page = 3;
   lcd.setCursor(0,0);
   lcd.print("Los valores se han devuelto a su configuracion normal");
   delay(2000);
   set_temperature = 200;
   rotating_speed = max_speed/10;
   }
   break;
   case 4:
      if (temperature_read != last_temperature_read || rotating_speed != last_set_velocidad) {
        lcd.clear();
        page = 4;
        lcd.setCursor(0,0);
        lcd.print("Temperatura: ");
        last_temperature_read = temperature_read;
        lcd.print(temperature_read);
        lcd.setCursor(0,1);
        lcd.print("Velocidad: ");
        lcd.print(rotating_speed);
        }
        middle = false;
   break;
   case 2:
      if(activate_stepper == false) {
        lcd.clear();
        lcd.print("motor prendido");
        delay(1000);
      activate_stepper = true;
      } else if (activate_stepper == true) {
        lcd.clear();
        lcd.print("motor apagado");
        delay(1000);
      activate_stepper = false;
        }
        middle = false;
   break;
  }
  }
  bool boton() {
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Clicked:
        middle = !middle;
        Serial.println("boton");
        break;
      }
    }
    return middle;
  }
void mainmenu() {
  if (frame != lastframe){
  switch (frame) {
    case 0:
      lcd.clear();
      lcd.print("Main Menu");
      lcd.setCursor(0,1);
      lcd.print("->Temperatura");
      break;
    case 1:
      lcd.clear();
      lcd.print("Temperatura");
      lcd.setCursor(0,1);
      lcd.print("->Velocidad");
      break;
    case 2:
      lcd.clear();
      lcd.print("Velocidad");
      lcd.setCursor(0,1);
      lcd.print("->Iniciar");
      break;
    case 3:
      lcd.clear();
      lcd.print("Iniciar");
      lcd.setCursor(0,1);
      lcd.print("->Reset");
      break;
    case 4:
      lcd.clear();
      lcd.print("Reset");
      lcd.setCursor(0,1);
      lcd.print("->Monitoreo");
      break;
    }
  }
 }

void timerIsr() {
  encoder->service();
  value += encoder->getValue();
  if  (value > last) {
    up = true;
    down=false;
    }
    
  if  (value < last) {
    up = false;
    down=true;
    }
  if (value != last) {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);
  }
}
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  stepper1.runSpeed();
}
