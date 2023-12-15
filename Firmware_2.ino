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

struct Frame {
  const char *nombre;
  int *variable;
  int minValue;
  int maxValue;
  int paso;
};

Frame frames[] = {
  {"Temperatura", &set_temperature, 0, 250, 10},
  {"Velocidad", &rotating_speed, 0, max_speed, 10},
  // se pueden agregar mas frames de la siguiente manera: {"Nombre", variable a cambiar, valor minimo, valor maximo, cuanto aumenta o disminuye por cada click del encoder}
};

//MENU
int numerodeframes = sizeof(frames) / sizeof(frames[0]);

int frame = 0;
int lastframe = 0;
int page = 0;
int estado = 0;
bool middle = false;

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
  digitalWrite(EN, HIGH);     
  stepper1.setMaxSpeed(max_speed);  
  pinMode(but1, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  
  pinMode(PWM_pin,OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000010;
  Time = millis();

  TCCR1A = 0;             
  TCCR1B = 0;             
  TCCR1A |= B00000010;    
  TCNT1 = 0;              
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
    digitalWrite(EN, LOW);    
    Serial.println(rotating_speed);
    stepper1.setSpeed(rotating_speed); 
    stepper1.runSpeed();
  }
  else
  {
    digitalWrite(EN, HIGH);    
    stepper1.setSpeed(0);
  }
  temperature_read = therm1.analog2temp(); 
  
 
  PID_error = set_temperature - temperature_read + 6;
  
  PID_p = 0.01*kp * PID_error;
  
  PID_i = 0.01*PID_i + (ki * PID_error);
  
  
  timePrev = Time;                            
  Time = millis();                            
  elapsedTime = (Time - timePrev) / 1000; 
 
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  
  PID_value = PID_p + PID_i + PID_d;

  
  
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > max_PWM){
    PID_value = max_PWM;
  }
  
  analogWrite(PWM_pin, PID_value);
  previous_error = PID_error;    .
  
  middle = boton();
  if (page == 0) {
    frame = cambiarframe();
  }
  if (frame != lastframe) {
    lastframe = frame;
  }
  if (middle) {
    displayitem();
  } else {
    page = 0;
  }

  stepper1.runSpeed();
} // loop void fin
int cambiarframe() {
  if (up && frame >= 0) {
    up = false;
    lastframe = frame;
    frame++;
  }
  if (frame >= numerodeframes) {
    lastframe = frame;
    frame = numerodeframes;
  }
  if (down && frame <= numerodeframes) {
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
  Frame currentFrame = frames[frame];

  if (*(currentFrame.variable) != value || estado == 0) {
    lcd.clear();
    page = frame + 1;
    estado = 1;
    if (value > *(currentFrame.variable)) {
      *(currentFrame.variable) += currentFrame.paso;
    } else if (value < *(currentFrame.variable)) {
      *(currentFrame.variable) -= currentFrame.paso;
    }
    if (*(currentFrame.variable) > currentFrame.maxValue) {
      *(currentFrame.variable) = currentFrame.maxValue;
    }
    value = *(currentFrame.variable);
    lcd.print(currentFrame.nombre);
    lcd.setCursor(0, 1);
    lcd.print("Valor: ");
    lcd.print(value);
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
  if (frame != lastframe) {
    lcd.clear();
    lcd.print("Main Menu");
    lcd.setCursor(0, 1);
    lcd.print("->");
    lcd.print(frames[frame].nombre);
  }
}
void timerIsr() {
  encoder->service();
  value += encoder->getValue();
  if  (value > last) {
    up = true;
    down = false;
    }
    
  if  (value < last) {
    up = false;
    down = true;
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
