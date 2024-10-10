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
int rotations = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds
long delta_time = 0.1 * 1000; // ms * 1000 microseconds

  

class Motor {
  public:
    int ENCA;
    int ENCB;
    int PWM_pin;
    int IN1;
    int IN2;
    
    volatile int posi;
    int pos;
    int e = 0; // Error
    int e_sum = 0;
    int e_prev = 0;
    float u = 0; // Control Signal

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
      e_prev = e;
      e = ref - pos;
      e_sum = e_sum + e ;
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
Motor motor1( MOT1ENCA,MOT1ENCB, MOT1PWM_pin,MOT1IN1, MOT1IN2);
Motor motor2( MOT2ENCA,MOT2ENCB, MOT2PWM_pin,MOT2IN1, MOT2IN2);


float Bang_Bang_Control(int e, int u_amplitude) {
  float u = 0;
  if (e < 0 ) {
    u = -1 * u_amplitude;
  }
  if (e > 0) {
    u = u_amplitude;
  }
  return u;
}

float Proportional_Control(int e, float kp) {
  float u;
  u = kp * e;
  if (u > 255) {
    u = 255;
  } 
  if ( u < -255) {
    u = -255;
  }
  return u;
}

float PID_Control(int e, int e_sum, int e_prev, float kp, float ki, float kd, long delta_time) {
  float u;
  float proportional = kp * e;
  float integral = ki * e_sum * delta_time;
  float derivative = kd * (e - e_prev)/delta_time;

  if (u > 255) {
    u = 255;
  } 
  if ( u < -255) {
    u = -255;
  }
  u = proportional + integral + derivative;
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
  loop_start_time = millis();
  // Set a reference angle position for the motor. It will be in counts, so conversion is needed to degrees for output.
  // Will change by +360 degrees every time_per_rotation milliseconds
  if ((millis() - time_start) > time_per_rotation) {
    rotations++;
    time_start = millis();
    motor1.set_error_sum_to_0();
    motor2.set_error_sum_to_0();

  }
  int ref1,ref2;
  ref1 = ref_amplitude * rotations;
  ref2 = ref_amplitude * (rotations) ;

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.set_pos_with_posi();
    motor2.set_pos_with_posi();
  }
  // calculate position error
  //error = ref - pos
  motor1.set_error(ref1); 
  motor2.set_error(-ref2);

  // calculate control signal
  // Proportional Control
  motor1.set_u(Proportional_Control(motor1.get_error(), 18));
  motor2.set_u(Proportional_Control(motor2.get_error(), 5));
 
  // PID Control
  motor1.set_u(PID_Control( motor1.get_error(), motor1.get_error_sum(), motor1.get_error_prev(),18.0 , 129, 0.79, delta_time));
  
  // Constant Values
  //motor1.u = 150;
  //motor2.u = -150;

  // call setMotor function to drive the motor at a set direction and torque based on a u output
  motor1.set_motor(motor1.get_u());
  motor2.set_motor(motor2.get_u());


  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print(ref1);
    Serial.print(" ");
    Serial.print(motor1.get_pos());
    Serial.print(" ");
    Serial.print(motor1.get_error());
    Serial.print(" ");
    Serial.print(motor1.get_u());
    Serial.print(" ");
    Serial.println();
   
  }

  // Place do while here
  dt_regulator(delta_time);
}

void dt_regulator(long delta_time) {
  float current_loop_time;
  do {
    current_loop_time = micros() - loop_start_time;
  } while (current_loop_time < delta_time);
}

// function to send signal to motor driver (could also be defined at top of code)
// set inputs and their classifications

// Interrupt service routine 
void readEncoderMot1() {
  int b = digitalRead(motor1.get_ENCB_Pin());
  if (b > 0) {
    motor1.increment_posi();
  } else {
    motor1.decrement_posi();
  }
}
void readEncoderMot2() {
  int b = digitalRead(motor2.get_ENCB_Pin());
  if (b > 0) {
    motor2.increment_posi();
  } else {
    motor2.decrement_posi();
  }
}