#include <ESP32Servo.h>
#include <math.h>
#define HIP_MIN_ANGLE 0
#define HIP_MAX_ANGLE 45
#define KNEE_MIN_ANGLE 0
#define KNEE_MAX_ANGLE 90
#define ANKLE_MIN_ANGLE 0
#define ANKLE_MAX_ANGLE 180
const float L1 = 80.0; 
const float L2 = 100.0; 


const int CYCLE_DURATION_MS = 1000; // 1 second for a full step cycle
const float STEP_LENGTH = 40;    // How far forward each step goes (mm)
const float STEP_HEIGHT = 30;    // How high the foot lifts (mm)

int flHipPin=13; int flKneePin=12; int flAnklePin=11;
int frHipPin=10; int frKneePin=9; int frAnklePin=8;
int rlHipPin=7; int rlKneePin=6; int rlAnklePin=5;
int rrHipPin=4; int rrKneePin=3; int rrAnklePin=2;

class Leg {
  public:
    Leg(int pinHip, int pinKnee, int pinAnkle) {
      hipPin = pinHip;
      kneePin = pinKnee;
      anklePin=pinAnkle;
    }

    void attachServos() {
      hipServo.attach(hipPin);
      kneeServo.attach(kneePin);
      ankleServo.attach(anklePin);
    }

    void moveTo(float x, float y, float z) {
      float theta1 = atan2(y, x);
      float D = sqrt(x*x + y*y + z*z);
      float cosArgTheta3 = (L1*L1 + L2*L2 - D*D) / (2 * L1 * L2);
      if (cosArgTheta3 > 1) cosArgTheta3 = 1;
      if (cosArgTheta3 < -1) cosArgTheta3 = -1;
      
      float theta3 = acos(cosArgTheta3);

      float alpha1 = atan2(-z, sqrt(x*x + y*y));
      
      float cosArgAlpha2 = (D*D + L1*L1 - L2*L2) / (2 * D * L1);
      if (cosArgAlpha2 > 1) cosArgAlpha2 = 1;
      if (cosArgAlpha2 < -1) cosArgAlpha2 = -1;
      
      float alpha2 = acos(cosArgAlpha2);

      float theta2 = alpha1 + alpha2;

      theta1 = theta1 * 180.0 / PI;
      theta2 = theta2 * 180.0 / PI;
      theta3 = theta3 * 180.0 / PI;

      int hipVal = map(theta1, HIP_MIN_ANGLE, HIP_MAX_ANGLE, 0, 180);
      int kneeVal = map(theta2, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE, 0, 180);
      int ankleVal = map(theta3, ANKLE_MIN_ANGLE, ANKLE_MAX_ANGLE, 0, 180);

      hipServo.write(hipVal);
      kneeServo.write(kneeVal);
      ankleServo.write(ankleVal);
    }
    private:

    Servo hipServo;
    Servo kneeServo;
    Servo ankleServo;

    int hipPin, kneePin, anklePin;
};

Leg legFL(flHipPin, flKneePin, flAnklePin);
Leg legFR(frHipPin, frKneePin, frAnklePin);
Leg legRL(rlHipPin, rlKneePin, rlAnklePin);
Leg legRR(rrHipPin, rrKneePin, rrAnklePin);


void gateCycle(){
  unsigned long current_time = millis();
  float cycle_phase = (current_time % CYCLE_DURATION_MS) / (float)CYCLE_DURATION_MS;

  // FL and RR use the cycle_phase directly
  float phase_A = cycle_phase;
  float xA = -cos(phase_A * 2 * PI) * (STEP_LENGTH / 2);
  float zA = sin(phase_A * 2 * PI) * STEP_HEIGHT;
  if (zA < 0) zA = 0; 

  // FR and RL are 0.5 out of phase
  float phase_B = fmod(cycle_phase + 0.5, 1.0);
  float xB = -cos(phase_B * 2 * PI) * (STEP_LENGTH / 2);
  float zB = sin(phase_B * 2 * PI) * STEP_HEIGHT;
  if (zB < 0) zB = 0; 

  // These z values need to be added to the robot default height
  float target_zA = -100 + zA; 
  float target_zB = -100 + zB;

  legFL.moveTo(xA, 0, target_zA);
  legRR.moveTo(xA, 0, target_zA);

  legFR.moveTo(xB, 0, target_zB);
  legRL.moveTo(xB, 0, target_zB);

  delay(10); 
}

void setup() {
  legFL.attachServos();
  legFR.attachServos();
  legRR.attachServos();
  legRL.attachServos();
}

void loop() {
  
  float x = 50; //random numbers to unit test a leg, will eventually have a read serial in for forward gate or left/right gate
  float y = 0;
  float z = -90;

  legFL.moveTo(x, y, z);

  delay(20);
}
