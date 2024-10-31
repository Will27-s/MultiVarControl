#include <SimplyAtomic.h>  // For the position read, to avoid missed counts
#include "functionDefinition.h"
//#include "1DoFCircleReferenceSignals.h"
#include "2DoFCircleReferenceSignals.h"
//#include "2DoFTriangleReferenceSignals.h"
//#include "2DoFSquareReferenceSignals.h"



// Define Encoder pins, PWM pin, and motor driver pins
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
#define MOT2PWM_pin 8  // pin to send pwm value to motor driver
#define MOT2IN1 6      // motor driver direction pins
#define MOT2IN2 7
// Inputs
#define MOT2ENCA 10
#define MOT2ENCB 9


int print_interval = 100;  // define how often values are sent to the serial monitor
int interval_count = 0;
float interval_start = 0;
float ref = 0;
            // u is the control signal
int u_amplitude = 200 ;  // control signal amplitude (must be < 256)
long time_start = 0;
long loop_start_time = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;
int rotations = 0; // TODO Make a static var in roation ref sig func
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds for rotate reference
float delta_time_micros = 2000; //microseconds
float delta_time_seconds = delta_time_micros/1e6;


// PID Control Values
float kp = 18.0;
float ki = 129.0;
float kd = 0.79;

float kp1 = kp;
float ki1 = ki;
float kd1 = kd;

float kp2 = kp;
float ki2 = ki;
float kd2 = kd;


// reference signal code  
// Future use pointers to make code more clean
bool initial_position = false;
int ref1,ref2;
int ref_index = 0;

  

class Motor {
  public:
    int ENCA;
    int ENCB;
    int PWM_pin;
    int IN1;
    int IN2;
    float kp;
    float ki;
    float kd;
    
    volatile int posi = 0;
    int pos = 0;
    int e = 0; // Error
    int e_sum = 0;
    int e_prev = 0;
    float u = 0; // Control Signal

    Motor(int eA, int eB, int pwm, int in1, int in2, float p, float i , float d) : ENCA(eA), ENCB(eB), PWM_pin(pwm), IN1(in1), IN2(in2), kp(p), ki(i), kd(d) {
        // Constructor stores pin values but does not configure them
    }
    void begin() {
        pinMode(ENCA, INPUT);
        pinMode(ENCB, INPUT);
        pinMode(PWM_pin, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
    }

    // Setters
    void set_motor(float u_) {
      
      int pwmVal;
      int dir;
      if (u_ < 0) {
        dir = -1;
        pwmVal = u_ * -1;
      }
      if (u_ > 0) {
        dir = 1;
        pwmVal = u_;
      } if (u_ = 0) {
        pwmVal = 0;
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
    void set_error(int ref) {
      e = ref - pos;
      
    }
    void set_error_prev(int ref) {
      e_prev = ref - pos;
    }
    void set_error_sum() {
      if (abs(e * kp) > 255) {

      } else {
        e_sum += e;
      }
      
    }

    void set_posi(int posi_) {
      posi = posi_;
    }

    void set_pos_with_posi() {
      pos = posi;
    }

    void set_u(float u_) {
      u = u_;
    }
    void increment_posi() {
      posi++;
    }
    void decrement_posi() {
      posi--;
    }

    void set_error_sum_to_0() {
      e_sum = 0;
    }

    // Getters
    int get_error() {
      return e;
    }

    int get_error_sum() {
      return e_sum;
    }
    
    int get_error_prev() {
      return e_prev;
    }
    float get_u() {
      return u;
    }

    int get_ENCB_Pin() {
      return ENCB;
    }
    int get_pos() {
      return pos;
    }

};

// Declaring the motors
Motor motor1( MOT1ENCA,MOT1ENCB, MOT1PWM_pin,MOT1IN1, MOT1IN2,kp1,ki1,kd1);
Motor motor2( MOT2ENCA,MOT2ENCB, MOT2PWM_pin,MOT2IN1, MOT2IN2,kp2,ki2,kd2);

void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  // Setting Motor PinModes
  motor1.begin();
  motor2.begin();
  
  attachInterrupt(digitalPinToInterrupt(motor1.ENCA), readEncoderMot1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.ENCA), readEncoderMot2, RISING);

  motor1.set_posi(ref1_circle[0]);
  motor2.set_posi(ref2_circle[0]);
  time_start = millis();
}

// This loop implements bang-bang code
void loop() {

  // Set Reference

  //ref1 = rotate_every_time_per_rotation_ref_signal();
  //ref2 = rotate_every_time_per_rotation_ref_signal();


  

  ref1 = ref1_circle[ref_index];
  ref2 = ref2_circle[ref_index];
  if (ref_index <= ref_length) {
    ref_index++;
  } else {
    ref_index = 0;
  }

  motor1.set_error_prev(ref1);
  motor2.set_error_prev(ref2);

  // TODO Test whether placing it at the end and loop_start_time at the start works
  loop_start_time = micros();
  dt_regulator(delta_time_micros);

  //record_pos_data();

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.set_pos_with_posi();
    motor2.set_pos_with_posi();
  }
  
  // Calculate Position error
  //error = ref - pos
  motor1.set_error(ref1); 
  motor2.set_error(ref2);

  motor1.set_error_sum();
  motor2.set_error_sum();

  // Control signal

  // Proportional Control
  //motor1.set_u(Proportional_Control(motor1.get_error(), 18));
  //motor2.set_u(Proportional_Control(motor2.get_error(), 5));
 
  
  // PID Control
  motor1.set_u(PID_Control( motor1.e, motor1.e_sum, motor1.e_prev, motor1.kp,motor1.ki,motor1.kd));
  motor2.set_u(PID_Control( motor2.e, motor2.e_sum, motor2.e_prev, motor2.kp,motor2.ki,motor2.kd));
  
  // Constant Values
  //motor1.set_u(0.0);
  //motor2.set_u(0);
  //motor2.u = 150;

  // Step Change
  //motor1.set_u(Step_Input(3));
  //motor2.set_u(Step_Input(3));

  // Drive Motor Based on u value
  motor1.set_motor(motor1.get_u());
  motor2.set_motor(motor2.get_u());


  serial_plotting(); // Function in DataRecording.ino

  
  
}
