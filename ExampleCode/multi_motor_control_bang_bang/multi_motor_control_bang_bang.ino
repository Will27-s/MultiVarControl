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
float delta_time_micros = 6000; //microseconds
float delta_time_seconds = delta_time_micros/1e6;
  
int ref_index = 0;
const int ref_length = 1000;
int ref1_circle[ref_length] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -3, -5, -6, -8, -10, -12, -14, -16, -18, -19, -21, -23, -25, -27, -29, -30, -32, -34, -36, -38, -40, -41, -43, -45, -47, -49, -50, -52, -54, -56, -57, -59, -61, -63, -64, -66, -68, -70, -71, -73, -75, -76, -78, -80, -81, -83, -85, -86, -88, -90, -91, -93, -94, -96, -98, -99, -101, -102, -104, -105, -107, -108, -110, -111, -113, -114, -116, -117, -119, -120, -122, -123, -124, -126, -127, -128, -130, -131, -132, -134, -135, -136, -138, -139, -140, -141, -143, -144, -145, -146, -147, -149, -150, -151, -152, -153, -154, -155, -157, -158, -159, -160, -161, -162, -163, -164, -165, -166, -167, -168, -169, -170, -171, -172, -172, -173, -174, -175, -176, -177, -178, -178, -179, -180, -181, -182, -182, -183, -184, -184, -185, -186, -187, -187, -188, -189, -189, -190, -190, -191, -192, -192, -193, -193, -194, -194, -195, -195, -196, -196, -197, -197, -198, -198, -199, -199, -200, -200, -200, -201, -201, -202, -202, -202, -203, -203, -203, -203, -204, -204, -204, -205, -205, -205, -205, -206, -206, -206, -206, -206, -207, -207, -207, -207, -207, -207, -207, -207, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -208, -207, -207, -207, -207, -207, -207, -207, -207, -206, -206, -206, -206, -206, -205, -205, -205, -205, -205, -204, -204, -204, -204, -203, -203, -203, -203, -202, -202, -202, -201, -201, -201, -200, -200, -200, -199, -199, -199, -198, -198, -198, -197, -197, -196, -196, -196, -195, -195, -194, -194, -194, -193, -193, -192, -192, -191, -191, -190, -190, -189, -189, -188, -188, -187, -187, -186, -186, -185, -185, -184, -184, -183, -183, -182, -182, -181, -180, -180, -179, -179, -178, -178, -177, -176, -176, -175, -175, -174, -173, -173, -172, -171, -171, -170, -170, -169, -168, -168, -167, -166, -166, -165, -164, -164, -163, -162, -162, -161, -160, -159, -159, -158, -157, -157, -156, -155, -154, -154, -153, -152, -151, -151, -150, -149, -149, -148, -147, -146, -145, -145, -144, -143, -142, -142, -141, -140, -139, -138, -138, -137, -136, -135, -134, -134, -133, -132, -131, -130, -130, -129, -128, -127, -126, -125, -125, -124, -123, -122, -121, -120, -120, -119, -118, -117, -116, -115, -114, -114, -113, -112, -111, -110, -109, -108, -107, -107, -106, -105, -104, -103, -102, -101, -100, -99, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0
}; 
int ref2_circle[ref_length] = {
    0, 2, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 95, 97, 99, 101, 103, 105, 107, 109, 111, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 177, 179, 181, 183, 185, 187, 189, 191, 193, 195, 198, 200, 202, 204, 206, 208, 210, 212, 214, 217, 219, 221, 223, 225, 227, 229, 231, 233, 235, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256, 259, 261, 263, 265, 267, 269, 271, 273, 275, 277, 280, 282, 284, 286, 288, 290, 292, 294, 296, 298, 301, 303, 305, 307, 309, 311, 313, 315, 317, 320, 322, 324, 326, 328, 330, 332, 334, 336, 338, 341, 343, 345, 347, 349, 351, 353, 355, 357, 359, 362, 364, 366, 368, 370, 372, 374, 376, 378, 380, 383, 385, 387, 389, 391, 393, 395, 397, 399, 402, 404, 406, 408, 410, 412, 414, 416, 418, 420, 423, 425, 427, 429, 431, 433, 435, 437, 439, 441, 444, 446, 448, 450, 452, 454, 456, 458, 460, 462, 465, 467, 469, 471, 473, 475, 477, 479, 481, 483, 486, 488, 490, 492, 494, 496, 498, 500, 502, 505, 507, 509, 511, 513, 515, 517, 519, 521, 523, 526, 528, 530, 532, 534, 536, 538, 540, 542, 544, 547, 549, 551, 553, 555, 557, 559, 561, 563, 565, 568, 570, 572, 574, 576, 578, 580, 582, 584, 586, 589, 591, 593, 595, 597, 599, 601, 603, 605, 608, 610, 612, 614, 616, 618, 620, 622, 624, 626, 629, 631, 633, 635, 637, 639, 641, 643, 645, 647, 650, 652, 654, 656, 658, 660, 662, 664, 666, 668, 671, 673, 675, 677, 679, 681, 683, 685, 687, 689, 692, 694, 696, 698, 700, 702, 704, 706, 708, 711, 713, 715, 717, 719, 721, 723, 725, 727, 729, 732, 734, 736, 738, 740, 742, 744, 746, 748, 750, 753, 755, 757, 759, 761, 763, 765, 767, 769, 771, 774, 776, 778, 780, 782, 784, 786, 788, 790, 792, 795, 797, 799, 801, 803, 805, 807, 809, 811, 814, 816, 818, 820, 822, 824, 826, 828, 830, 832, 835, 837, 839, 841, 843, 845, 847, 849, 851, 853, 856, 858, 860, 862, 864, 866, 868, 870, 872, 874, 877, 879, 881, 883, 885, 887, 889, 891, 893, 895, 898, 900, 902, 904, 906, 908, 910, 912, 914, 917, 919, 921, 923, 925, 927, 929, 931, 933, 935, 938, 940, 942, 944, 946, 948, 950, 952, 954, 956, 959, 961, 963, 965, 967, 969, 971, 973, 975, 977, 980, 982, 984, 986, 988, 990, 992, 994, 996, 998, 1001, 1003, 1005, 1007, 1009, 1011, 1013, 1015, 1017, 1020, 1022, 1024, 1026, 1028, 1030, 1032, 1034, 1036, 1038, 1041, 1043, 1045, 1047, 1049, 1049, 1047, 1045, 1043, 1041, 1038, 1036, 1034, 1032, 1030, 1028, 1026, 1024, 1022, 1020, 1017, 1015, 1013, 1011, 1009, 1007, 1005, 1003, 1001, 998, 996, 994, 992, 990, 988, 986, 984, 982, 980, 977, 975, 973, 971, 969, 967, 965, 963, 961, 959, 956, 954, 952, 950, 948, 946, 944, 942, 940, 938, 935, 933, 931, 929, 927, 925, 923, 921, 919, 917, 914, 912, 910, 908, 906, 904, 902, 900, 898, 895, 893, 891, 889, 887, 885, 883, 881, 879, 877, 874, 872, 870, 868, 866, 864, 862, 860, 858, 856, 853, 851, 849, 847, 845, 843, 841, 839, 837, 835, 832, 830, 828, 826, 824, 822, 820, 818, 816, 814, 811, 809, 807, 805, 803, 801, 799, 797, 795, 792, 790, 788, 786, 784, 782, 780, 778, 776, 774, 771, 769, 767, 765, 763, 761, 759, 757, 755, 753, 750, 748, 746, 744, 742, 740, 738, 736, 734, 732, 729, 727, 725, 723, 721, 719, 717, 715, 713, 711, 708, 706, 704, 702, 700, 698, 696, 694, 692, 689, 687, 685, 683, 681, 679, 677, 675, 673, 671, 668, 666, 664, 662, 660, 658, 656, 654, 652, 650, 647, 645, 643, 641, 639, 637, 635, 633, 631, 629, 626, 624, 622, 620, 618, 616, 614, 612, 610, 608, 605, 603, 601, 599, 597, 595, 593, 591, 589, 586, 584, 582, 580, 578, 576, 574, 572, 570, 568, 565, 563, 561, 559, 557, 555, 553, 551, 549, 547, 544, 542, 540, 538, 536, 534, 532, 530, 528, 526, 523, 521, 519, 517, 515, 513, 511, 509, 507, 505, 502, 500, 498, 496, 494, 492, 490, 488, 486, 483, 481, 479, 477, 475, 473, 471, 469, 467, 465, 462, 460, 458, 456, 454, 452, 450, 448, 446, 444, 441, 439, 437, 435, 433, 431, 429, 427, 425, 423, 420, 418, 416, 414, 412, 410, 408, 406, 404, 402, 399, 397, 395, 393, 391, 389, 387, 385, 383, 380, 378, 376, 374, 372, 370, 368, 366, 364, 362, 359, 357, 355, 353, 351, 349, 347, 345, 343, 341, 338, 336, 334, 332, 330, 328, 326, 324, 322, 320, 317, 315, 313, 311, 309, 307, 305, 303, 301, 298, 296, 294, 292, 290, 288, 286, 284, 282, 280, 277, 275, 273, 271, 269, 267, 265, 263, 261, 259, 256, 254, 252, 250, 248, 246, 244, 242, 240, 238, 235, 233, 231, 229, 227, 225, 223, 221, 219, 217, 214, 212, 210, 208, 206, 204, 202, 200, 198, 195, 193, 191, 189, 187, 185, 183, 181, 179, 177, 174, 172, 170, 168, 166, 164, 162, 160, 158, 156, 153, 151, 149, 147, 145, 143, 141, 139, 137, 135, 132, 130, 128, 126, 124, 122, 120, 118, 116, 114, 111, 109, 107, 105, 103, 101, 99, 97, 95, 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 71, 69, 67, 65, 63, 61, 59, 57, 55, 53, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 8, 6, 4, 2, 0
};
  

  

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
      e_sum += e;
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
 
float PID_Control(int e, int e_sum, int e_prev, float kp, float ki, float kd) {
  float u;
  float proportional = kp * e;
  float integral = ki * e_sum * delta_time_seconds;
  float derivative = kd * (e - e_prev)/delta_time_seconds;

  if (u > 255) {
    u = 255;
  } 
  if ( u < -255) {
    u = -255;
  }
  u = proportional + integral + derivative ;
  return u;
}

float Step_Response(int step_time) { // step_time seconds
  static int increment = 0;
  static float u = u_amplitude;
  int step_cycles = step_time * 1e6/delta_time_micros; // Gets the number of cycles that is equivalent to the chosen step_time
  if (increment >= step_cycles) {
    u *= -1; // Reverse polarity after step time
    increment = 0;
  }
  increment++;
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
    //motor1.set_error_sum_to_0();
    //motor2.set_error_sum_to_0();

  }

  
  int ref1,ref2;
  //ref1 = ref_amplitude * rotations;
  //ref2 = ref_amplitude * (rotations);


  ref1 = ref1_circle[ref_index];
  ref2 = ref2_circle[ref_index];
  if (ref_index <= ref_length) {
    ref_index++;
  }

  motor1.set_error_prev(ref1);
  motor2.set_error_prev(ref2);

  // Place do while here
  loop_start_time = micros();
  dt_regulator(loop_start_time);

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.set_pos_with_posi();
    motor2.set_pos_with_posi();
  }
  
  // calculate position error
  //error = ref - pos
  motor1.set_error(ref1); 
  motor2.set_error(ref2);

  motor1.set_error_sum();
  motor2.set_error_sum();

  // calculate control signal
  // Proportional Control
  //motor1.set_u(Proportional_Control(motor1.get_error(), 18));
  //motor2.set_u(Proportional_Control(motor2.get_error(), 5));
 
  // PID Control
  float kp = 18.0;
  float ki = 129.0;
  float kd = 0.79;

  //motor1.set_u(PID_Control( motor1.e, motor1.e_sum, motor1.e_prev, kp,ki,kd));
  //motor2.set_u(PID_Control( motor2.e, motor2.e_sum, motor2.e_prev, kp,ki,kd));
  
  // Constant Values
  //motor1.set_u(0.0);
  //motor2.set_u(0);
  //motor2.u = 150;

  // Step Change
  motor1.set_u(Step_Response(3));

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

  
  
}

void dt_regulator(long loop_start_time) { // Regulates time so that length of loop is the same
  int current_loop_time = 0;
  do {
    current_loop_time = micros() - loop_start_time;
    ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.set_pos_with_posi();
    motor2.set_pos_with_posi();
  }
  } while (current_loop_time < delta_time_micros);
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