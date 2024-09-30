#include <SimplyAtomic.h>  // For the position read, to avoid missed counts
// Testing if updates are read in VSCode
// Testing if updates are read in arduino

// Define Encoder pins, pwm pin, and motor driver pins
// Motor 1
// Inputs
#define MOT1ENCA 0     // interrupt pin encoder A
#define MOT1ENCB 1     // interrupt pin encoder B
// Outputs
#define MOT1PWM_pin 2  // pin to send pwm value to motor driver
#define MOT1IN1 3      // motor driver direction pins
#define MOT1IN2 4

// Motor 2
// Outputs
#define MOT2PWM_pin 6  // pin to send pwm value to motor driver
#define MOT2IN1 7      // motor driver direction pins
#define MOT2IN2 8
// Inputs
#define MOT2ENCA 10
#define MOT2ENCB 9


volatile int posi = 0;     // specify posi as volatile, integer because counts are discrete
int print_interval = 100;  // define how often values are sent to the serial monitor
int interval_count = 0;
float interval_start = 0;
float ref = 0;
            // u is the control signal
int u_amplitude = 150;  // control signal amplitude (must be < 256)
long time_start = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;
int rotations = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds

  

class Motor {
  public:
    int ENCA;
    int ENCB;
    int PWM_pin;
    int IN1;
    int IN2;
    
    volatile int posi = 0;
    int pos = 0;
    int e = 0; // Error
    int u = 0; // Control Signal

    Motor(int eA, int eB, int pwm, int in1, int in2) : ENCA(eA), ENCB(eB), PWM_pin(pwm), IN1(in1), IN2(in2) {
        // Constructor stores pin values but does not configure them
    }
    void begin() {
        pinMode(ENCA, INPUT);
        pinMode(ENCB, INPUT);
        pinMode(PWM_pin, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
    }
    void set_motor(int u) {
      this->u = u;
      int pwmVal;
      int dir;
      if (u < 0) {
        dir = -1;
        pwmVal = u * -1;
      }
      if (u > 0) {
        dir = 1;
        pwmVal = u;
      }
      analogWrite(PWM_pin, pwmVal);
      //set direction
      if (dir == 1) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      } else if (dir == -1) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }
  }
    void get_error(int ref) {
      e = ref - pos;
    }


};

// Declaring the motors
Motor motor1( MOT1ENCA,MOT1ENCB, MOT1PWM_pin,MOT1IN1, MOT1IN2);
Motor motor2( MOT2ENCA,MOT2ENCB, MOT2PWM_pin,MOT2IN1, MOT2IN2);


int Bang_Bang_Control(int e, int u_amplitude) {
  int u = 0;
  if (e < 0 ) {
    u = -1 * u_amplitude;
  }
  if (e > 0) {
    u = u_amplitude;
  }
  return u;
}


void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  motor1.begin();
  motor2.begin();
  
  attachInterrupt(digitalPinToInterrupt(motor1.ENCA), readEncoderMot1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.ENCA), readEncoderMot2, RISING);

  
  time_start = millis();
}

// This loop implements bang-bang code
void loop() {

  // Set a reference angle position for the motor. It will be in counts, so conversion is needed to degrees for output.
  // Will change by +360 degrees every time_per_rotation milliseconds
  if ((millis() - time_start) > time_per_rotation) {
    rotations++;
    time_start = millis();
  }

  ref = ref_amplitude * rotations;

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.pos = motor1.posi;
    motor2.pos = motor2.posi;
  }

  // calculate position error
  //error = ref - pos
  motor1.get_error(ref); 
  motor2.get_error(ref);

  // calculate control signal
  motor1.u = Bang_Bang_Control(motor1.e, u_amplitude);
  motor2.u = Bang_Bang_Control(motor2.e, u_amplitude);


  // call setMotor function to drive the motor at a set direction and torque based on a u output
  motor1.set_motor(motor1.u);
  motor2.set_motor(motor2.u);


  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print(ref);
    Serial.print(" ");
    Serial.print(motor1.pos);
    Serial.print(" ");
    Serial.print(motor1.e);
    Serial.print(" ");
    Serial.print(motor1.u);
    Serial.print(" ");
    Serial.println();
  }
}

// function to send signal to motor driver (could also be defined at top of code)
// set inputs and their classifications

// Interrupt service routine 
void readEncoderMot1() {
  int b = digitalRead(motor1.ENCB);
  if (b > 0) {
    motor1.posi++;
  } else {
    motor1.posi--;
  }
}
void readEncoderMot2() {
  int b = digitalRead(motor2.ENCB);
  if (b > 0) {
    motor2.posi++;
  } else {
    motor2.posi--;
  }
}