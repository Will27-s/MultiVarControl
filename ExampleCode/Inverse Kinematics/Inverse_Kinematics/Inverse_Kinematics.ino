#include <cmath>
// This code contains a function that should return th1 and th2 positions of the motors
// based on input x,y positions
// It also has a function that converts angles to a encoder position value

struct Angles {
  float th1;
  float th2;
}

void inverse_kinematics(float x, float y, float l1, float l2) {
  Angles angles; // Not sure if this is the most efficient way to store
  float costh2 = (pow(x,2) + pow(y,2) -pow(l1,2)-pow(l2,2))/(2 * l1 * l2);
  float sinth2 = sqrt(1-pow(costh2,2));
  angles.th2 = atan2(sinth2,costh2);

  float A = x;
  float B = l1 + (l2* costh2);
  float C = -l2 * sinth2;

  float w = (C- sqrt(pow(C,2)+pow(B,2)-pow(A,2)))/(A+B);
  angles.th1 = 2 *atan(w);
  // Need to test as there are multiple values to these sols.
}


const float counts_per_rotation = 131.25 * 16;

float angle_to_encoder_pos(theta) {
  static const float counts_per_rotation = 131.25 * 16; //Only called the first time the function is called

  return ceil(counts_per_rotation * theta/360); // Not sure whether to use floor or ceil
}