#include <math.h>
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter Kalman_Filter_Accx(0.02, 0.02, 0.1);//Filter for accx calculated from encoders
SimpleKalmanFilter Kalman_Filter_Velocity(0.001, 0.001, 2);//Filter for Velocity calculated from encoders
SimpleKalmanFilter Kalman_Filter_vx(0.001, 0.001, 1);//Filter for Velocity calculated from encoders
SimpleKalmanFilter Kalman_Filter_Omega(0.1, 0.1, 1);//Filter for Omega calculated from encoders

float error_R = 0, sum_error_R = 0, last_error_R = 0;
double Kp_R, Ki_R, Kd_R;
float error_L = 0, sum_error_L = 0, last_error_L = 0;
double Kp_L, Ki_L, Kd_L;
float omega_FLC_R, omega_FLC_L;

#define PI 3.1415926535897932384626433832795      // Define the PI
#define ENC_COUNT_REV_Left 330 // So xung mot vong banh phai cua kenh A 
#define ENC_IN_Right 2 // Encoder output to Arduino Interrupt pin (Chuan da kiem tra lai)
#define ENC_COUNT_REV_Right 330 // So xung mot vong banh xe right cua kenh A 
#define ENC_IN_Left 3 // C1 Encoder right output to Arduino Interrupt pin

#include <Fuzzy.h>                                // Fuzzy logic library
#include <SPI.h>                                  // Use for communicating SD Card
#include <SD.h>                                   // SD Card library

Fuzzy *fuzzy = new Fuzzy();                       // Instantiating a Fuzzy object

volatile long encoderValue_Left = 0;// Pulse count from encoder
volatile long encoderValue_Right = 0;// Pulse count from encoder right
int interval = 50;// interval for measurements velocity by encoders
float x_e, x_m, y_e, y_m, phi_e, phi_m, v_e, v_m, ome_e, ome_m, x_fusion, y_fusion, phi_fusion, v_fusion, ome_fusion; // variables for FUSION

// Counters for milliseconds during interval
long previousMillis_Left = 0, currentMillis_Left = 0, previousMillis_Right = 0, currentMillis_Right = 0;

// Variable for RPM measuerment
float rpm_Left = 0, rpm_Right = 0, Omega_Left = 0, Omega_Right = 0, Prev_Velocity_Odo = 0, Current_Velocity_Odo = 0, Omega = 0;
float currentEnergy = 0, lastEnergy = 0, deltaEnergy = 0, totalEnergy = 0;
float currentEnergy_mpu = 0, lastEnergy_mpu = 0, deltaEnergy_mpu = 0, totalEnergy_mpu = 0;
float currentEnergy_fusion = 0, lastEnergy_fusion = 0, deltaEnergy_fusion = 0, totalEnergy_fusion = 0;

// Variable for PWM motor speed output
int motorPwm_Left = 0, motorPwm_Right = 0;
float x = 0, y = 0, phi = 0; // initial robot position
float Dl, Dr, Dc;  // The variable for Odometry

int R_curent_tick = 0;       // The current number of tick of the encoder right. Count when motor rotates. Will not be reset
int R_last_tick = 0;         // The last number of tick of the encoder right. Will not be reset
int deltaRtick = 0;          // deltaRtick = Rtick - RtickAnt

int L_curent_tick = 0;       // The current number of tick of the encoder left. Count when motor rotates. Will not be reset
int L_last_tick = 0;         // The last number of tick of the encoder left. Will not be reset
int deltaLtick = 0;          // deltaLtick = Ltick - LtickAnt
//------------------------------- THE ROBOT PARAMETER ---------------------------------
float dia = 0.085;       // The diameter of wheel (m)
float b = 0.2;           // The distance between 2 wheels (m)
float m = 1.733;         // The mass of robot (kg)
float I = 0.08;          // The moment of initia of the robot (kg.m2)

//--------------------------DEFINE PIN FOR L298 + ARDUINO MEGA 2560---------------------------------------
const int IN1_MOTOR_LEFT = 9;                   //For IN1 pin motor left
const int IN2_MOTOR_LEFT = 8;                   //For IN2 pin motor left
const int ENA_MOTOR_LEFT = 11;                  //For PWM pin motot left
const int ENA_MOTOR_RIGHT = 6;                  // For PWM pin Motor right
const int IN3_MOTOR_RIGHT = 12;                 // For IN3 pin Motor right
const int IN4_MOTOR_RIGHT = 31;                 // For IN4 pin motor right
//------------------------------- STARTING POINT (0,0,0)--------------------------------------------------------

//---------------------------------- WAYPOINTS  ------------------------------------------------------
float XW1 = 2;
float YW1 = 2;

float XW2 = 4;
float YW2 = 2;

//------------------------------- THE DESTINATION  ------------------------------------------------------
float Xd = 2;
float Yd = 0;
float Phid = atan2(Yd - y_e, Xd - x_e);// when using encoder
//float Phid = atan2(Yd - y_m, Xd - x_m);// when using MPU
//float Phid = atan2(Yd - y_fusion, Xd - x_fusion);// when using fusion

//****** For FISION data***************************************************************
#define Nsta 5     // Five state values: x, y, phi, v, ome
#define Mobs 10    // 10 variables: x_e, x_m, y_e, y_m, phi_e, phi_m, v_e, v_m, ome_e, ome_m
#include <TinyEKF.h>

class Fuser : public TinyEKF {
  public:
    Fuser()
    {
      // We approximate the process noise using a small constant
      this->setQ(0, 0, .0001); // for x
      this->setQ(1, 1, .0001); // for y
      this->setQ(2, 2, .0001); // for phi
      this->setQ(3, 3, .0001); // for v
      this->setQ(4, 4, .0001); // for ome

      // Same for measurement noise
      this->setR(0, 0, .0001);  //for x_e
      this->setR(1, 1, .0001);  //for x_m
      this->setR(2, 2, .0001);  //for y_e
      this->setR(3, 3, .0001);  //for y_m
      this->setR(4, 4, .0001);  //for phi_e
      this->setR(5, 5, .0001);  //for phi_m
      this->setR(6, 6, .0001);  //for v_e
      this->setR(7, 7, .0001);  //for v_m
      this->setR(8, 8, .0001);  //for ome_e
      this->setR(9, 9, .0001);  //for ome_m
    }
  protected:
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
      // Process model is f(x) = x
      fx[0] = this->x[0];
      fx[1] = this->x[1];
      fx[2] = this->x[2];
      fx[3] = this->x[3];
      fx[4] = this->x[4];

      // So process model Jacobian is identity matrix
      F[0][0] = 1;
      F[1][1] = 1;
      F[2][2] = 1;
      F[3][3] = 1;
      F[4][4] = 1;

      // Measurement function simplifies the relationship between state and sensor readings for convenience.
      // A more realistic measurement function would distinguish between state value and measured value; e.g.:
      hx[0] = this->x[0]; // x_e from previous state
      hx[1] = this->x[0]; // x_m from previous state
      hx[2] = this->x[1]; // y_e from previous state
      hx[3] = this->x[1]; // y_m from previous state
      hx[4] = this->x[2]; // phi_e from previous state
      hx[5] = this->x[2]; // phi_m from previous state
      hx[6] = this->x[3]; // v_e from previous state
      hx[7] = this->x[3]; // v_m from previous state
      hx[8] = this->x[4]; // ome_e from previous state
      hx[9] = this->x[4]; // ome_m from previous state

      // Jacobian of measurement function
      H[0][0] = 1;        // x_e from previous state
      H[1][0] = 1 ;       // x_m from previous state
      H[2][1] = 1 ;       // y_e from previous state
      H[3][1] = 1 ;       // y_m from previous state
      H[4][2] = 1 ;       // phi_e from previous state
      H[5][2] = 1 ;       // phi_m from previous state
      H[6][3] = 1 ;       // v_e from previous state
      H[7][3] = 1 ;       // v_m from previous state
      H[8][4] = 1 ;       // ome_e from previous state
      H[9][4] = 1 ;       // ome_m from previous state
    }
};

Fuser ekf;

/***************************************************************************
  For the MPU9250_WE library
***************************************************************************/
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

float old_Theta = 0, new_Theta = 0, old_Omega = 0, new_Omega = 0, dT;
unsigned long timer;

xyzFloat old_a, new_a, old_v, new_v, old_s, new_s;
//------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Set encoder as input with internal pullup
  pinMode(ENC_IN_Right, INPUT_PULLUP);
  pinMode(ENC_IN_Left, INPUT_PULLUP);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_IN_Left), updateEncoder_Left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_Right), updateEncoder_Right, RISING);

  // ---------------Set PWM and IN connections as outputs-------------------------------
  pinMode (ENA_MOTOR_LEFT, OUTPUT);
  pinMode (IN1_MOTOR_LEFT, OUTPUT);
  pinMode (IN2_MOTOR_LEFT, OUTPUT);
  pinMode (ENA_MOTOR_RIGHT, OUTPUT);
  pinMode (IN3_MOTOR_RIGHT, OUTPUT);
  pinMode (IN4_MOTOR_RIGHT, OUTPUT);

  // Setup initial values for timer
  previousMillis_Right = millis();
  previousMillis_Left = millis();
  

  ////////////////////////////////////////////////////////////////////////
  Wire.begin();
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  }
  else {
    Serial.println("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  }
  else {
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
  timer = micros();

  //-----------------------***********************************--------------------------------
  // -----------------*******SETUP FOR FYZZY LOGIC CONTROLLER*****----------------------------
  //------------***********-----------INPUT 1: DISTANCE------------***************-------------
  FuzzyInput *distance = new FuzzyInput(1);                     //Instantiating a Fuzzy Input_1 object (INPUT_1 là khoảng cách từ robot dến waypoint (goal)), đơn vị met
  FuzzySet *VeryClose = new FuzzySet(0, 0, 0, 2.5);            // Instantiating a FuzzySet object
  distance->addFuzzySet(VeryClose);                             // Including the FuzzySet into FuzzyInput
  FuzzySet *Close = new FuzzySet(0, 2.5, 2.5, 5);           // Instantiating a FuzzySet object
  distance->addFuzzySet(Close);                                 // Including the FuzzySet into FuzzyInput
  FuzzySet *Medium = new FuzzySet(2.5, 5, 5, 7.5);        // Instantiating a FuzzySet object
  distance->addFuzzySet(Medium);                                // Including the FuzzySet into FuzzyInput
  FuzzySet *Far = new FuzzySet(5, 7.5, 7.5, 10);             // Instantiating a FuzzySet object
  distance->addFuzzySet(Far);                                   // Including the FuzzySet into FuzzyInput
  FuzzySet *VeryFar = new FuzzySet(7.5, 10, 10, 10);              // Instantiating a FuzzySet object
  distance->addFuzzySet(VeryFar);                               // Including the FuzzySet into FuzzyInput
  fuzzy->addFuzzyInput(distance);                               // Including the FuzzyInput into Fuzzy

  //------------***********-----------INPUT 2: DELTA_THETA------------***************-------------
  FuzzyInput *deltaTheta = new FuzzyInput(2);                   //Instantiating a Fuzzy Input_2 object (INPUT_2 được tính (deltaTheta = Phid -Phi) với (Phid = atan2(Yd - y, Xd - x))
  FuzzySet *BigNegative = new FuzzySet(-PI, -PI, -PI, -PI / 2); // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(BigNegative);                         // Including the FuzzySet into FuzzyInput
  FuzzySet *Negative = new FuzzySet(-PI, -PI / 2, -PI / 2, 0);  // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(Negative);                            // Including the FuzzySet into FuzzyInput
  FuzzySet *Zero = new FuzzySet(-PI / 2, 0, 0, PI / 2);         // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(Zero);                                // Including the FuzzySet into FuzzyInput
  FuzzySet *Positive = new FuzzySet(0, PI / 2, PI / 2, PI);     // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(Positive);                            // Including the FuzzySet into FuzzyInput
  FuzzySet *BigPositive = new FuzzySet(PI / 2, PI, PI, PI);     // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(BigPositive);                         // Including the FuzzySet into FuzzyInput
  fuzzy->addFuzzyInput(deltaTheta);                             // Including the FuzzyInput into Fuzzy

  //----------*************---------OUTPUT 1 OMEGA_R-------------**************--------------  
  FuzzyOutput *omegaR = new FuzzyOutput(1);                     // Instantiating a FuzzyOutput objects
  FuzzySet *VerySmallRight = new FuzzySet(0, 0, 0, 7.5);        // Instantiating a FuzzySet object  
  omegaR->addFuzzySet(VerySmallRight);                          // Including the FuzzySet into FuzzyOutput  
  FuzzySet *SmallRight = new FuzzySet(0.0035689100000002583, 7.5, 7.5, 8.52472246);         // Instantiating a FuzzySet object  
  omegaR->addFuzzySet(SmallRight);                              // Including the FuzzySet into FuzzyOutput  
  FuzzySet *MediumBigRight = new FuzzySet(7.5, 15, 15, 22.5);   // Instantiating a FuzzySet object  
  omegaR->addFuzzySet(MediumBigRight);                          // Including the FuzzySet into FuzzyOutput
  FuzzySet *BigRight = new FuzzySet(16.58266616, 22.5, 22.5, 22.51706397);        // Instantiating a FuzzySet object  
  omegaR->addFuzzySet(BigRight);                                // Including the FuzzySet into FuzzyOutput
  FuzzySet *VeryBigRight = new FuzzySet(22.5, 30, 30, 30);      // Instantiating a FuzzySet object  
  omegaR->addFuzzySet(VeryBigRight);                            // Including the FuzzySet into FuzzyOutput    
  fuzzy->addFuzzyOutput(omegaR);                                // Including the FuzzyOutput into Fuzzy

  //----------*************---------OUTPUT 2 OMEGA_L-------------**************--------------  
  FuzzyOutput *omegaL = new FuzzyOutput(2);                     // Instantiating a FuzzyOutput objects
  FuzzySet *VerySmallLeft = new FuzzySet(0, 0, 0, 7.5);         // Instantiating a FuzzySet object  
  omegaL->addFuzzySet(VerySmallLeft);                           // Including the FuzzySet into FuzzyOutput  
  FuzzySet *SmallLeft = new FuzzySet(0.0, 7.5, 7.5, 8.46009048);          // Instantiating a FuzzySet object  
  omegaL->addFuzzySet(SmallLeft);                               // Including the FuzzySet into FuzzyOutput  
  FuzzySet *MediumBigLeft = new FuzzySet(7.5, 15, 15, 22.5);    // Instantiating a FuzzySet object  
  omegaL->addFuzzySet(MediumBigLeft);                           // Including the FuzzySet into FuzzyOutput
  FuzzySet *BigLeft = new FuzzySet(15.00000004, 22.5, 22.5, 22.88670046);         // Instantiating a FuzzySet object  
  omegaL->addFuzzySet(BigLeft);                                 // Including the FuzzySet into FuzzyOutput
  FuzzySet *VeryBigLeft = new FuzzySet(22.5, 30, 30, 30);       // Instantiating a FuzzySet object  
  omegaL->addFuzzySet(VeryBigLeft);                             // Including the FuzzySet into FuzzyOutput    
  fuzzy->addFuzzyOutput(omegaL);                                // Including the FuzzyOutput into Fuzzy
  //||------------------------------------------------------------------------------------------------------------||
  //**********************************BUILDINGB FUZZY RULES FOR OMEGA RIGHT***************************************
  //||------------------------------------------------------------------------------------------------------------||
  // RULE_1R:"IF distance is VeryClose AND deltaTheta is BigNegative THEN omegaR is VerySmallRight ***********RULE 1
  FuzzyRuleAntecedent *ifDistanceVeryClose1 = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose1->joinWithAND(VeryClose, BigNegative);                                  // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight1 = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight1->addOutput(VerySmallRight);                                       // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifDistanceVeryClose1, thenomegaRVerySmallRight1);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule1);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_2R:"IF distance is VeryClose AND deltaTheta is Negative THEN omegaR is SmallRight ******************RULE 2
  FuzzyRuleAntecedent *ifDistanceVeryClose2 = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose2->joinWithAND(VeryClose, Negative);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight2 = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight2->addOutput(SmallRight);                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifDistanceVeryClose2, thenomegaRSmallRight2);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule2);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_3R:"IF distance is VeryClose AND deltaTheta is Zero THEN omegaR is VerySmallRight ******************RULE 3
  FuzzyRuleAntecedent *ifDistanceVeryClose3 = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose3->joinWithAND(VeryClose, Zero);                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight3 = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight3->addOutput(VerySmallRight);                                       // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifDistanceVeryClose3, thenomegaRVerySmallRight3);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule3);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_4R:"IF distance is VeryClose AND deltaTheta is Positive THEN omegaR is BigRight ********************RULE 4
  FuzzyRuleAntecedent *ifDistanceVeryClose4 = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose4->joinWithAND(VeryClose, Positive);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight4 = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight4->addOutput(BigRight);                                                   // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifDistanceVeryClose4, thenomegaRBigRight4);        // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule4);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_5R:"IF distance is VeryClose AND deltaTheta is BigPositive THEN omegaR is VeryBigRight *************RULE 5
  FuzzyRuleAntecedent *ifDistanceVeryClose5 = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose5->joinWithAND(VeryClose, BigPositive);                                  // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight5 = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight5->addOutput(VeryBigRight);                                           // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifDistanceVeryClose5, thenomegaRVeryBigRight5);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule5);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_6R:"IF distance is Close AND deltaTheta is BigNegative THEN omegaR is VerySmallRight ***************RULE 6
  FuzzyRuleAntecedent *ifDistanceClose6 = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose6->joinWithAND(Close, BigNegative);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight6 = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight6->addOutput(VerySmallRight);                                       // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifDistanceClose6, thenomegaRVerySmallRight6);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule6);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_7R:"IF distance is Close AND deltaTheta is Negative THEN omegaR is SmallRight **********************RULE 7
  FuzzyRuleAntecedent *ifDistanceClose7 = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose7->joinWithAND(Close, Negative);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight7 = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight7->addOutput(SmallRight);                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifDistanceClose7, thenomegaRSmallRight7);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule7);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_8R:"IF distance is Close AND deltaTheta is Zero THEN omegaR is SmallRight **************************RULE 8
  FuzzyRuleAntecedent *ifDistanceClose8 = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose8->joinWithAND(Close, Zero);                                                 // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight8 = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight8->addOutput(SmallRight);                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifDistanceClose8, thenomegaRSmallRight8);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule8);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_9R:"IF distance is Close AND deltaTheta is Positive THEN omegaR is BigRight ************************RULE 9
  FuzzyRuleAntecedent *ifDistanceClose9 = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose9->joinWithAND(Close, Positive);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight9 = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight9->addOutput(BigRight);                                                   // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifDistanceClose9, thenomegaRBigRight9);            // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule9);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_10R:"IF distance is Close AND deltaTheta is BigPositive THEN omegaR is VeryBigRight ***************RULE 10
  FuzzyRuleAntecedent *ifDistanceClose10 = new FuzzyRuleAntecedent();                         // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose10->joinWithAND(Close, BigPositive);                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight10 = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight10->addOutput(VeryBigRight);                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifDistanceClose10, thenomegaRVeryBigRight10);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule10);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_11R:"IF distance is Medium AND deltaTheta is BigNegative THEN omegaR is VerySmallRight ************RULE 11
  FuzzyRuleAntecedent *ifDistanceMedium11 = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium11->joinWithAND(Medium, BigNegative);                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight11 = new FuzzyRuleConsequent();                // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight11->addOutput(VerySmallRight);                                      // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifDistanceMedium11, thenomegaRVerySmallRight11); // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule11);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_12R:"IF distance is Medium AND deltaTheta is Negative THEN omegaR is SmallRight *******************RULE 12
  FuzzyRuleAntecedent *ifDistanceMedium12 = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium12->joinWithAND(Medium, Negative);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight12 = new FuzzyRuleConsequent();                    // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight12->addOutput(SmallRight);                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifDistanceMedium12, thenomegaRSmallRight12);     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule12);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_13R:"IF distance is Medium AND deltaTheta is Zero THEN omegaR is MediumBigRight *******************RULE 13
  FuzzyRuleAntecedent *ifDistanceMedium13 = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium13->joinWithAND(Medium, Zero);                                              // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRMediumBigRight13 = new FuzzyRuleConsequent();                // Instantiating a FuzzyRuleConsequent objects
  thenomegaRMediumBigRight13->addOutput(MediumBigRight);                                      // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifDistanceMedium13, thenomegaRMediumBigRight13); // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule13);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_14R:"IF distance is Medium AND deltaTheta is Positive THEN omegaR is BigRight *********************RULE 14
  FuzzyRuleAntecedent *ifDistanceMedium14 = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium14->joinWithAND(Medium, Positive);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight14 = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight14->addOutput(BigRight);                                                  // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifDistanceMedium14, thenomegaRBigRight14);       // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule14);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_15R:"IF distance is Medium AND deltaTheta is BigPositive THEN omegaR is VeryBigRight **************RULE 15
  FuzzyRuleAntecedent *ifDistanceMedium15 = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium15->joinWithAND(Medium, BigPositive);                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight15 = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight15->addOutput(VeryBigRight);                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifDistanceMedium15, thenomegaRVeryBigRight15);   // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule15);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_16R:"IF distance is Far AND deltaTheta is BigNegative THEN omegaR is VerySmallRight ***************RULE 16
  FuzzyRuleAntecedent *ifDistanceFar16 = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar16->joinWithAND(Far, BigNegative);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight16 = new FuzzyRuleConsequent();                // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight16->addOutput(VerySmallRight);                                      // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifDistanceFar16, thenomegaRVerySmallRight16);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule16);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_17R:"IF distance is Far AND deltaTheta is Negative THEN omegaR is SmallRight **********************RULE 17
  FuzzyRuleAntecedent *ifDistanceFar17 = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar17->joinWithAND(Far, Negative);                                                // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight17 = new FuzzyRuleConsequent();                    // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight17->addOutput(SmallRight);                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, ifDistanceFar17, thenomegaRSmallRight17);        // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule17);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_18R:"IF distance is Far AND deltaTheta is Zero THEN omegaR is BigRight ****************************RULE 18
  FuzzyRuleAntecedent *ifDistanceFar18 = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar18->joinWithAND(Far, Zero);                                                    // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight18 = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight18->addOutput(BigRight);                                                  // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifDistanceFar18, thenomegaRBigRight18);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule18);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_19R:"IF distance is Far AND deltaTheta is Positive THEN omegaR is BigRight ************************RULE 19
  FuzzyRuleAntecedent *ifDistanceFar19 = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar19->joinWithAND(Far, Positive);                                                // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight19 = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight19->addOutput(BigRight);                                                  // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifDistanceFar19, thenomegaRBigRight19);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule19);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_20R:"IF distance is Far AND deltaTheta is BigPositive THEN omegaR is VeryBigRight *****************RULE 20
  FuzzyRuleAntecedent *ifDistanceFar20 = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar20->joinWithAND(Far, BigPositive);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight20 = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight20->addOutput(VeryBigRight);                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, ifDistanceFar20, thenomegaRVeryBigRight20);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule20);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_21R:"IF distance is VeryFar AND deltaTheta is BigNegative THEN omegaR is VerySmallRight ***********RULE 21
  FuzzyRuleAntecedent *ifDistanceVeryFar21 = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar21->joinWithAND(VeryFar, BigNegative);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVerySmallRight21 = new FuzzyRuleConsequent();                // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVerySmallRight21->addOutput(VerySmallRight);                                      // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, ifDistanceVeryFar21, thenomegaRVerySmallRight21);// Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule21);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_22R:"IF distance is VeryFar AND deltaTheta is Negative THEN omegaR is SmallRight ******************RULE 22
  FuzzyRuleAntecedent *ifDistanceVeryFar22 = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar22->joinWithAND(VeryFar, Negative);                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSmallRight22 = new FuzzyRuleConsequent();                    // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSmallRight22->addOutput(SmallRight);                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, ifDistanceVeryFar22, thenomegaRSmallRight22);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule22);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_23R:"IF distance is VeryFar AND deltaTheta is Zero THEN omegaR is VeryBigRight ********************RULE 23
  FuzzyRuleAntecedent *ifDistanceVeryFar23 = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar23->joinWithAND(VeryFar, Zero);                                            // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight23 = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight23->addOutput(VeryBigRight);                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, ifDistanceVeryFar23, thenomegaRVeryBigRight23);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule23);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_24R:"IF distance is VeryFar AND deltaTheta is Positive THEN omegaR is BigRight ********************RULE 24
  FuzzyRuleAntecedent *ifDistanceVeryFar24 = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar24->joinWithAND(VeryFar, Positive);                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBigRight24 = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBigRight24->addOutput(BigRight);                                                  // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, ifDistanceVeryFar24, thenomegaRBigRight24);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule24);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_25R:"IF distance is VeryFar AND deltaTheta is BigPositive THEN omegaR is VeryBigRight *************RULE 25
  FuzzyRuleAntecedent *ifDistanceVeryFar25 = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar25->joinWithAND(VeryFar, BigPositive);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVeryBigRight25 = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVeryBigRight25->addOutput(VeryBigRight);                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, ifDistanceVeryFar25, thenomegaRVeryBigRight25);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule25);                                                           // Including the FuzzyRule into Fuzzy

  //|-----------------------------------------------------------------------------------------------------------|
  //**********************************BUILDINGB FUZZY RULES FOR OMEGA LEFT***************************************
  //|-----------------------------------------------------------------------------------------------------------|
  // RULE_1L:"IF distance is VeryClose AND deltaTheta is BigNegative THEN omegaL is VeryBigLeft***************RULE 1
  FuzzyRuleAntecedent *ifDistanceVeryClose1L = new FuzzyRuleAntecedent();                     // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose1L->joinWithAND(VeryClose, BigNegative);                                 // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft1L = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft1L->addOutput(VeryBigLeft);                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, ifDistanceVeryClose1L, thenomegaLVeryBigLeft1L); // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule26);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_2L:"IF distance is VeryClose AND deltaTheta is Negative THEN omegaL is BigLeft**********************RULE 2
  FuzzyRuleAntecedent *ifDistanceVeryClose2L = new FuzzyRuleAntecedent();                     // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose2L->joinWithAND(VeryClose, Negative);                                    // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft2L = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft2L->addOutput(BigLeft);                                                    // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, ifDistanceVeryClose2L, thenomegaLBigLeft2L);     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule27);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_3L:"IF distance is VeryClose AND deltaTheta is Zero THEN omegaL is VerySmallLeft********************RULE 3
  FuzzyRuleAntecedent *ifDistanceVeryClose3L = new FuzzyRuleAntecedent();                     // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose3L->joinWithAND(VeryClose, Zero);                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft3L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft3L->addOutput(VerySmallLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, ifDistanceVeryClose3L, thenomegaLVerySmallLeft3L);// Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule28);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_4L:"IF distance is VeryClose AND deltaTheta is Positive THEN omegaL is SmallLeft********************RULE 4
  FuzzyRuleAntecedent *ifDistanceVeryClose4L = new FuzzyRuleAntecedent();                     // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose4L->joinWithAND(VeryClose, Positive);                                    // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft4L = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft4L->addOutput(SmallLeft);                                                // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, ifDistanceVeryClose4L, thenomegaLSmallLeft4L);   // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule29);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_5L:"IF distance is VeryClose AND deltaTheta is BigPositive THEN omegaL is VerySmallLeft*************RULE 5
  FuzzyRuleAntecedent *ifDistanceVeryClose5L = new FuzzyRuleAntecedent();                      // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryClose5L->joinWithAND(VeryClose, BigPositive);                                  // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft5L = new FuzzyRuleConsequent();                  // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft5L->addOutput(VerySmallLeft);                                         // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, ifDistanceVeryClose5L, thenomegaLVerySmallLeft5L);// Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule30);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_6L:"IF distance is Close AND deltaTheta is BigNegative THEN omegaL is VeryBigLeft*******************RULE 6
  FuzzyRuleAntecedent *ifDistanceClose6L = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose6L->joinWithAND(Close, BigNegative);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft6L = new FuzzyRuleConsequent();                    // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft6L->addOutput(VeryBigLeft);                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, ifDistanceClose6L, thenomegaLVeryBigLeft6L);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule31);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_7L:"IF distance is Close AND deltaTheta is Negative THEN omegaL is BigLeft**************************RULE 7
  FuzzyRuleAntecedent *ifDistanceClose7L = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose7L->joinWithAND(Close, Negative);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft7L = new FuzzyRuleConsequent();                        // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft7L->addOutput(BigLeft);                                                     // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, ifDistanceClose7L, thenomegaLBigLeft7L);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule32);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_8L:"IF distance is Close AND deltaTheta is Zero THEN omegaL is SmallLeft****************************RULE 8
  FuzzyRuleAntecedent *ifDistanceClose8L = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose8L->joinWithAND(Close, Zero);                                                 // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft8L = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft8L->addOutput(SmallLeft);                                                 // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, ifDistanceClose8L, thenomegaLSmallLeft8L);        // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule33);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_9L:"IF distance is Close AND deltaTheta is Positive THEN omegaL is SmallLeft************************RULE 9
  FuzzyRuleAntecedent *ifDistanceClose9L = new FuzzyRuleAntecedent();                          // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose9L->joinWithAND(Close, Positive);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft9L = new FuzzyRuleConsequent();                      // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft9L->addOutput(BigRight);                                                  // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, ifDistanceClose9L, thenomegaLSmallLeft9L);        // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule34);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_10L:"IF distance is Close AND deltaTheta is BigPositive THEN omegaL is VerySmallLeft***************RULE 10
  FuzzyRuleAntecedent *ifDistanceClose10L = new FuzzyRuleAntecedent();                         // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceClose10L->joinWithAND(Close, BigPositive);                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft10L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft10L->addOutput(VerySmallLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, ifDistanceClose10L, thenomegaLVerySmallLeft10L);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule35);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_11L:"IF distance is Medium AND deltaTheta is BigNegative THEN omegaL is VeryBigLeft****************RULE 11
  FuzzyRuleAntecedent *ifDistanceMedium11L = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium11L->joinWithAND(Medium, BigNegative);                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft11L = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft11L->addOutput(VeryBigLeft);                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, ifDistanceMedium11L, thenomegaLVeryBigLeft11L);   // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule36);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_12L:"IF distance is Medium AND deltaTheta is Negative THEN omegaL is BigLeft***********************RULE 12
  FuzzyRuleAntecedent *ifDistanceMedium12L = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium12L->joinWithAND(Medium, Negative);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft12L = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft12L->addOutput(BigLeft);                                                    // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, ifDistanceMedium12L, thenomegaLBigLeft12L);       // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule37);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_13L:"IF distance is Medium AND deltaTheta is Zero THEN omegaL is MediumBigLeft*********************RULE 13
  FuzzyRuleAntecedent *ifDistanceMedium13L = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium13L->joinWithAND(Medium, Zero);                                              // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLMediumBigLeft13L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLMediumBigLeft13L->addOutput(MediumBigLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, ifDistanceMedium13L, thenomegaLMediumBigLeft13L); // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule38);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_14L:"IF distance is Medium AND deltaTheta is Positive THEN omegaL is SmallLeft*********************RULE 14
  FuzzyRuleAntecedent *ifDistanceMedium14L = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium14L->joinWithAND(Medium, Positive);                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft14L = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft14L->addOutput(SmallLeft);                                                // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, ifDistanceMedium14L, thenomegaLSmallLeft14L);     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule39);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_15L:"IF distance is Medium AND deltaTheta is BigPositive THEN omegaL is VerySmallLeft**************RULE 15
  FuzzyRuleAntecedent *ifDistanceMedium15L = new FuzzyRuleAntecedent();                        // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceMedium15L->joinWithAND(Medium, BigPositive);                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft15L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft15L->addOutput(VerySmallLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, ifDistanceMedium15L, thenomegaLVerySmallLeft15L); // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule40);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_16L:"IF distance is Far AND deltaTheta is BigNegative THEN omegaL is VeryBigLeft*******************RULE 16
  FuzzyRuleAntecedent *ifDistanceFar16L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar16L->joinWithAND(Far, BigNegative);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft16L = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft16L->addOutput(VeryBigLeft);                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, ifDistanceFar16L, thenomegaLVeryBigLeft16L);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule41);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_17L:"IF distance is Far AND deltaTheta is Negative THEN omegaL is BigLeft**************************RULE 17
  FuzzyRuleAntecedent *ifDistanceFar17L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar17L->joinWithAND(Far, Negative);                                                // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft17L = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft17L->addOutput(BigLeft);                                                    // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, ifDistanceFar17L, thenomegaLBigLeft17L);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule42);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_18L:"IF distance is Far AND deltaTheta is Zero THEN omegaL is BigLeft******************************RULE 18
  FuzzyRuleAntecedent *ifDistanceFar18L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar18L->joinWithAND(Far, Zero);                                                    // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft18L = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft18L->addOutput(BigLeft);                                                    // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, ifDistanceFar18L, thenomegaLBigLeft18L);          // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule43);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_19L:"IF distance is Far AND deltaTheta is Positive THEN omegaL is SmallLeft************************RULE 19
  FuzzyRuleAntecedent *ifDistanceFar19L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar19L->joinWithAND(Far, Positive);                                                // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft19L = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft19L->addOutput(SmallLeft);                                                // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, ifDistanceFar19L, thenomegaLSmallLeft19L);        // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule44);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_20L:"IF distance is Far AND deltaTheta is BigPositive THEN omegaL is VerySmallLeft*****************RULE 20
  FuzzyRuleAntecedent *ifDistanceFar20L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceFar20L->joinWithAND(Far, BigPositive);                                             // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft20L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft20L->addOutput(VerySmallLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, ifDistanceFar20L, thenomegaLVerySmallLeft20L);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule45);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_21L:"IF distance is VeryFar AND deltaTheta is BigNegative THEN omegaL is VeryBigLeft***************RULE 21
  FuzzyRuleAntecedent *ifDistanceVeryFar21L = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar21L->joinWithAND(VeryFar, BigNegative);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft21L = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft21L->addOutput(VeryBigLeft);                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, ifDistanceVeryFar21L, thenomegaLVeryBigLeft21L);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule46);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_22L:"IF distance is VeryFar AND deltaTheta is Negative THEN omegaL is BigLeft**********************RULE 22
  FuzzyRuleAntecedent *ifDistanceVeryFar22L = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar22L->joinWithAND(VeryFar, Negative);                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBigLeft22L = new FuzzyRuleConsequent();                       // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBigLeft22L->addOutput(BigLeft);                                                    // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, ifDistanceVeryFar22L, thenomegaLBigLeft22L);      // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule47);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_23L:"IF distance is VeryFar AND deltaTheta is Zero THEN omegaL is VeryBigLeft**********************RULE 23
  FuzzyRuleAntecedent *ifDistanceVeryFar23L = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar23L->joinWithAND(VeryFar, Zero);                                            // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVeryBigLeft23L = new FuzzyRuleConsequent();                   // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVeryBigLeft23L->addOutput(VeryBigLeft);                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, ifDistanceVeryFar23L, thenomegaLVeryBigLeft23L);  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule48);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_24L:"IF distance is VeryFar AND deltaTheta is Positive THEN omegaL is SmallLeft********************RULE 24
  FuzzyRuleAntecedent *ifDistanceVeryFar24L = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar24L->joinWithAND(VeryFar, Positive);                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSmallLeft24L = new FuzzyRuleConsequent();                     // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSmallLeft24L->addOutput(SmallLeft);                                                // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, ifDistanceVeryFar24L, thenomegaLSmallLeft24L);    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule49);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_25L:"IF distance is VeryFar AND deltaTheta is BigPositive THEN omegaL is VerySmallLeft*************RULE 25
  FuzzyRuleAntecedent *ifDistanceVeryFar25L = new FuzzyRuleAntecedent();                       // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVeryFar25L->joinWithAND(VeryFar, BigPositive);                                     // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVerySmallLeft25L = new FuzzyRuleConsequent();                 // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVerySmallLeft25L->addOutput(VerySmallLeft);                                        // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule50 = new FuzzyRule(50, ifDistanceVeryFar25L, thenomegaLVerySmallLeft25L);// Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule50);                                                            // Including the FuzzyRule into Fuzzy
  //|---------------------------------------------------------------------------------------------------------|
  // FFFFFFFFFFFFFFFFFFFFFFFFFFFFF***KẾT THÚC SETUP CHO FUZZY LOGIC CONTROLLER***FFFFFFFFFFFFFFFFFFFFFFFFFFFFF
  //|---------------------------------------------------------------------------------------------------------|
  Kp_R = 0.926;
  Ki_R = 9.276;
  Kd_R = 0.025;

  Kp_L = 0.753;
  Ki_L = 8.528;
  Kd_L = 0.015;
  delay (3000);
  timer = micros();// For start count dT
}

void loop() {
  //---------------------------------------------------------------
  //********************** ENCODERS CALCULATOR ********************
  //---------------------------------------------------------------
  // ---------------------------Su dung de xem tren matlab------------------------------------
  digitalWrite(IN1_MOTOR_LEFT, HIGH);
  digitalWrite(IN2_MOTOR_LEFT, LOW);
  digitalWrite(IN3_MOTOR_RIGHT, HIGH);
  digitalWrite(IN4_MOTOR_RIGHT, LOW);


  // Update RPM value every second
  currentMillis_Right = millis();
  currentMillis_Left = millis();

  if ((currentMillis_Right - previousMillis_Right > interval) && (currentMillis_Left - previousMillis_Left > interval))
  {
    previousMillis_Right = currentMillis_Right;
    // Calculate RPM
    rpm_Right = (encoderValue_Right * 1200 / ENC_COUNT_REV_Right);
    // Converting rpm_Right to radian per second
    Omega_Right = (rpm_Right * 2 * 3.14) / 60;
    previousMillis_Left = currentMillis_Left;
    // Calculate RPM
    rpm_Left = (encoderValue_Left * 1200 / ENC_COUNT_REV_Left);
    // Converting rpm_Left to radian per second
    Omega_Left = (rpm_Left * 2 * 3.14) / 60;
    //Calculate linear velocity of robot
    Current_Velocity_Odo = (0.0425 * (Omega_Right + Omega_Left)) / 2;
    v_e = Kalman_Filter_Velocity.updateEstimate(Current_Velocity_Odo);
    //Calculate angular velocity of the robot
    Omega = (0.0425 * (Omega_Right - Omega_Left)) / 0.2;
    ome_e = Kalman_Filter_Omega.updateEstimate(Omega);
    encoderValue_Left = 0;
    encoderValue_Right = 0;

    odometry();  //Odometry: Call x_e, y_e,phi_e, v_e, ome_e
    //*********************FINISH ENCODER****************************
    //-------------------------------------------------------------------------------------------------------

    xyzFloat accCorrRaw = myMPU9250.getCorrectedAccRawValues();//Get coorect acc values after calibrating
    xyzFloat gyr = myMPU9250.getGyrValues(); // Get gyro values in degrees/s

    float Gyro_z = ((gyr.z) * 3.14) / 180; //Angular velocity around the z_axis in rad/s
    float Correct_accx = ((accCorrRaw.x) / 16384) * 9.80665; // Corrected accx DLPF in m/s2
    Correct_accx = Kalman_Filter_Accx.updateEstimate(Correct_accx);

    if ((rpm_Right == 0) && (rpm_Left == 0))
    {
      Correct_accx = 0;
      Gyro_z = 0;
      ome_m = 0;
    }
    else
    {
      Correct_accx = Correct_accx;
      Gyro_z = Gyro_z;
      ome_m = (((gyr.z) * PI) / 180); //Angular velocity around the z_axis in rad/s
      ome_m = Kalman_Filter_Omega.updateEstimate(ome_m);
    }


    // updating acceleration
    old_a.x = new_a.x;                    // storing old data
    new_a.x = Correct_accx;             // reading out new data

    // update omega
    old_Omega = new_Omega;
    new_Omega = Gyro_z;

    // updating velocity
    old_v = new_v;                    // storing old data
    new_v.x = trapezoid(new_a.x, old_a.x, old_v.x, dT); // reading out new data
    v_m = trapezoid_v_m(new_a.x, old_a.x, old_v.x, dT); // for calculate x_m

    if ((rpm_Right == 0) && (rpm_Left == 0))
    {
      new_v.x = 0;
      v_m = 0;
    }
    else
    {
      if (new_v.x >= 0)
      {
        new_v.x = new_v.x;
        v_m = v_m;
      }
      else
      {
        new_v.x = 0;
        v_m = 0;
      }
    }

    // updating displacement
    if ((rpm_Right != 0) || (rpm_Left != 0))
    {
      old_Theta = new_Theta;
      new_Theta = trapezoid_phi(new_Omega, old_Omega, old_Theta, dT); // calculate robot orientation Theta
      new_Theta = atan2(sin(new_Theta), cos(new_Theta));  // Switch angle position between -PI and PI
      phi_m = new_Theta;

      old_s = new_s;                    // storing old data
      new_s.x = trapezoid_x(new_v.x, old_v.x, old_s.x, dT);
      x_m = x_m + new_s.x * cos(phi_m);
      y_m = y_m + new_s.x * sin(phi_m);
      new_s.x = 0;
    }
    else
    {
      //new_s = new_s;
      x_m = x_m;
      y_m = y_m;
      new_Theta = new_Theta;
      phi_m = phi_m;
    }

    //-----------------------------------------------------------------------------
    //*********************Call FUSION VALUES *********************************
    double z[10] = {x_e, x_m, y_e, y_m, phi_e, phi_m, v_e, v_m, ome_e, ome_m};
    ekf.step(z);
    x_e = z[0]; // filter value of x_e by EKF
    x_m = z[1]; // filter value of x_m by EKF
    y_e = z[2]; // filter value of y_e by EKF
    y_m = z[3]; // filter value of y_m by EKF
    phi_e = z[4]; // filter value of phi_e by EKF
    phi_m = z[5]; // filter value of phi_m by EKF
    v_e = z[6]; // filter value of v_e by EKF
    v_m = z[7]; // filter value of v_m by EKF
    ome_e = z[8]; // filter value of ome_e by EKF
    ome_m = z[9]; // filter value of ome_m by EKF
    x_fusion = ekf.getX(0);
    y_fusion = ekf.getX(1);
    phi_fusion = ekf.getX(2);
    v_fusion = ekf.getX(3);
    ome_fusion = ekf.getX(4);

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    float Phid = atan2(Yd - y_e, Xd - x_e);                             //Recalculate the desired angle in each repetition, as changes with each movement
    float deltaX = abs(x_fusion - Xd);
    float deltaY = abs(y_fusion - Yd);
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);       // Calculate value current distance: input1
    float deltaTheta = Phid - phi_e;                                // Calculate value current detaTheta: input2

    if (deltaTheta > PI)
    {
      deltaTheta -= 2 * PI;
    }
    if (deltaTheta < -PI)
    {
      deltaTheta += 2 * PI;
    }

    fuzzy->setInput(1, distance);                       // Set the distance value as the input1
    fuzzy->setInput(2, deltaTheta);                     // Set the deltaTheta value as the input2
    fuzzy->fuzzify();                                   // Running the Fuzzification
    omega_FLC_R = fuzzy->defuzzify(1);                 // Running the Defuzzification for OmegaR: output1. This is output of FLC
    omega_FLC_L = fuzzy->defuzzify(2);                 // Running the Defuzzification for OmegaL: output2. This is output of FLC
    omega_FLC_R = Kalman_Filter_Velocity.updateEstimate(omega_FLC_R);
    omega_FLC_L = Kalman_Filter_Velocity.updateEstimate(omega_FLC_L);

    error_R = omega_FLC_R - Omega_Right;
    sum_error_R = sum_error_R + error_R;
    float omega_PID_R = Kp_R * error_R + Ki_R * sum_error_R + Kd_R * (error_R - last_error_R);

    error_L = omega_FLC_L - Omega_Left;
    sum_error_L = sum_error_L + error_L;
    float omega_PID_L = Kp_L * error_L + Ki_L * sum_error_L + Kd_L * (error_L - last_error_L);

    float PWMr = map(omega_PID_R, 0, 30, 0, 255);                 // 80-255 for Body Dynamics, 0-255 for Motor dynamics go strait.
    float PWMl = map(omega_PID_L, 0, 30, 0, 255);                 // 90-255 for Body Dynamics, 0-255 for Motor dynamics go other path.

    last_error_R = error_R;
    last_error_L = error_L;


    //---------------------- Stop robot moving when it acheave destination----------------------------------
    if ( abs(x_e - Xd) < 0.1 && abs(y_e - Yd) < 0.1)
    {
      Xd = XW1;
      Yd = YW1;
    }
    if (abs(y_e - YW1) < 0.1)
    {
      Xd = XW2;
      Yd = YW2;
    }

    if ( abs(x_e - XW2) < 0.1 && abs(y_e - YW2) < 0.1)

    {
      analogWrite(ENA_MOTOR_RIGHT, 0);    // For right wheel
      analogWrite(ENA_MOTOR_LEFT, 0);     // For left wheel
    }
    else
    {
      analogWrite(ENA_MOTOR_RIGHT, PWMr); // For right wheel
      analogWrite(ENA_MOTOR_LEFT, PWMl);  // For left wheel
    }    
  }//((currentMillis_Right - previousMillis_Right > interval) && (currentMillis_Left - previousMillis_Left > interval))
  dT = (micros() - timer) / 1000000.0;         // update dT, division to convert from micro to seconds
  timer = micros(); // update timer
  Serial.print(dT);
  Serial.println();
}// loop
//---------------------------------------------------------------------------------------------------------------
//**********************************END LOOP********************************************************************
//---------------------------------------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////
float trapezoid(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data);
  return out;
}

float trapezoid_v_m(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data);
  return out;
}

float trapezoid_phi(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data);
  return out;
}

float trapezoid_x(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data); 
  return out;
}

void updateEncoder_Left()
{
  encoderValue_Left++; // Increment value for each pulse from encoder
  L_curent_tick ++;    // Increment value for each pulse from encoder
}

void updateEncoder_Right()
{
  encoderValue_Right++; // Increment value for each pulse from encoder
  R_curent_tick ++;     // Increment value for each pulse from encoder
}

void odometry() {
  deltaRtick = R_curent_tick - R_last_tick;
  Dr = PI * dia * (deltaRtick / (double)330);     // Dr = pi * diameter of wheel * (encoder counts / encoder resolution )
  deltaLtick = L_curent_tick - L_last_tick;
  Dl = PI * dia * (deltaLtick / (double)330);     // Dl & Dr are travel distance for the left and right wheel respectively
  Dc = (Dl + Dr) / 2 ;                            // Incremental linear displacement of the robot's centerpoint C
  x_e = x_e + Dc * cos(phi_e);                    // Current point X position
  y_e = y_e + Dc * sin(phi_e);                    // Current point Y position
  phi_e = phi_e + (Dr - Dl) / b;                  // The robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
  phi_e = atan2(sin(phi_e), cos(phi_e));          // Switch angle position between -PI and PI
  R_last_tick = R_curent_tick;                    // Update ticks
  L_last_tick = L_curent_tick;                    // Update ticks
}
