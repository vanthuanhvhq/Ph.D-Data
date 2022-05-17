#include <Fuzzy.h>                                // Fuzzy logic library
#include <math.h>                                 // Use for atan()
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter Kalman_Filter_Velocity(0.001, 0.001, 2);//Filter for Velocity calculated from encoders

#define PI 3.1415926535897932384626433832795      // Define the PI
#define ENC_COUNT_REV_Left 330                    // The number of tick per revolution of the motor left (For A chenal) 
#define ENC_IN_Left 2                             // Encoder output to Arduino Interrupt pin for C1 (A chenal) of motor left   
#define ENC_COUNT_REV_Right 330                   // The number of tick per revolution of the motor right (For A chenal)
#define ENC_IN_Right 3                            // Encoder output to Arduino Interrupt pin for C1 (A chenal) of motor right  

Fuzzy *fuzzy = new Fuzzy();                       // Instantiating a Fuzzy object

unsigned long t = millis();
unsigned long timer;
float dT;

int interval = 50;                                // One-second interval for measurements. Because I want to find angular velocity of the wheel in rad/s
volatile long encoderValue_Left = 0;              // Counter tick for the encoder left. This value will be reset each loop.
volatile long encoderValue_Right = 0;             // Counter tick for the encoder right. This value will be reset each loop.

int R_curent_tick = 0;                            // The current number of tick of the encoder right. Count when motor rotates. Will not be reset
int R_last_tick = 0;                              // The last number of tick of the encoder right. Will not be reset
int deltaRtick = 0;                               // deltaRtick = Rtick - RtickAnt

int L_curent_tick = 0;                            // The current number of tick of the encoder left. Count when motor rotates. Will not be reset
int L_last_tick = 0;                              // The last number of tick of the encoder left. Will not be reset
int deltaLtick = 0;                               // deltaLtick = Ltick - LtickAnt

// --------------------------COUNTERS FOR MILIISECONDS DURING INTERVAL (1000 ms)-------------------------
long previousMillis_Left = 0;
long currentMillis_Left = 0;
long previousMillis_Right = 0;
long currentMillis_Right = 0;

float error_R = 0, sum_error_R = 0, last_error_R = 0;
double Kp_R, Ki_R, Kd_R;
float error_L = 0, sum_error_L = 0, last_error_L = 0;
double Kp_L, Ki_L, Kd_L;
float omega_FLC_R, omega_FLC_L, omega_PID_R, omega_PID_L;
float PWMr, PWMl;
//------------------------------- THE ROBOT PARAMETER ---------------------------------------------------
float dia = 0.085;                                // The diameter of wheel (m)
float b = 0.2;                                    // The distance between 2 wheels (m)
float m = 1.733;                                  // The mass of robot (kg)
float I = 0.08;                               // The moment of initia of the robot (kg.m2)

// --------------------------VARIABLE FOR VELOCITY OF THE WHEEL MEASUERMENT (RPM)-------------------------
float rpm_Left = 0;
float rpm_Right = 0;
float Omega_Left = 0;
float Omega_Right = 0;
float Velocity = 0;
float Omega = 0;
float currentEnergy = 0;
float lastEnergy = 0;
float deltaEnergy = 0;
float totalEnergy = 0;

//--------------------------DEFINE PIN FOR L298 + ARDUINO MEGA 2560---------------------------------------
const int IN1_MOTOR_LEFT = 9;                   //For IN1 pin motor left
const int IN2_MOTOR_LEFT = 8;                   //For IN2 pin motor left
const int ENA_MOTOR_LEFT = 11;                  //For PWM pin motot left
const int ENA_MOTOR_RIGHT = 6;                  // For PWM pin Motor right
const int IN3_MOTOR_RIGHT = 12;                 // For IN3 pin Motor right
const int IN4_MOTOR_RIGHT = 31;                 // For IN4 pin motor right

//|------------------------------------------------------------------------------------------------------|
//|************************************  VOID SETUP () **************************************************|
//|------------------------------------------------------------------------------------------------------|
void setup()
{
  Serial.begin(115200);                          // Setup Serial Monitor
  Serial1.begin(115200);                         // Setup Serial for ESP with MEGA
  Serial1.setTimeout(10);

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

  //-----------------------***********************************--------------------------------
  // -----------------*******SETUP FOR FYZZY LOGIC CONTROLLER*****----------------------------
  //------------***********-----------INPUT 1: DISTANCE------------***************-------------
  FuzzyInput *distance = new FuzzyInput(1);           //Instantiating a Fuzzy Input_1 object (INPUT_1 là khoảng cách từ robot dến waypoint (goal)), đơn vị met
  FuzzySet *VC = new FuzzySet(0, 0, 0, 1.54148787);            // Instantiating a FuzzySet object
  distance->addFuzzySet(VC);                          // Including the FuzzySet into FuzzyInput
  FuzzySet *NC = new FuzzySet(0.06865673000000005, 1.99138378, 1.99138378, 3.96946432);            // Instantiating a FuzzySet object
  distance->addFuzzySet(NC);                          // Including the FuzzySet into FuzzyInput
  FuzzySet *C = new FuzzySet(2.42940413, 4.09933048, 4.09933048, 6.0992621499999995);             // Instantiating a FuzzySet object
  distance->addFuzzySet(C);                           // Including the FuzzySet into FuzzyInput
  FuzzySet *M = new FuzzySet(4.06097223, 4.92022555, 4.92022555, 5.61789315);             // Instantiating a FuzzySet object
  distance->addFuzzySet(M);                           // Including the FuzzySet into FuzzyInput
  FuzzySet *NF = new FuzzySet(5.06619761, 6.07322869, 6.07322869, 7.93990758);            // Instantiating a FuzzySet object
  distance->addFuzzySet(NF);                          // Including the FuzzySet into FuzzyInput
  FuzzySet *F = new FuzzySet(6.6512177900000005, 7.97345152, 7.97345152, 9.76111185);            // Instantiating a FuzzySet object
  distance->addFuzzySet(F);                           // Including the FuzzySet into FuzzyInput
  FuzzySet *VF = new FuzzySet(8.28727347, 10, 10, 10);         // Instantiating a FuzzySet object
  distance->addFuzzySet(VF);                          // Including the FuzzySet into FuzzyInput
  fuzzy->addFuzzyInput(distance);                     // Including the FuzzyInput into Fuzzy

  //------------***********-----------INPUT 2: DELTA_THETA------------***************-------------
  FuzzyInput *deltaTheta = new FuzzyInput(2);               //Instantiating a Fuzzy Input_2 object (INPUT_2 được tính (deltaTheta = Phid -Phi) với (Phid = atan2(Yd - y, Xd - x))
  FuzzySet *VBN = new FuzzySet(-3.14, -3.14, -3.14, -1.7275325300000002);     // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(VBN);                             // Including the FuzzySet into FuzzyInput
  FuzzySet *BN = new FuzzySet(-3.0895723999999998, -1.57849907, -1.57849907, -1.0688488999999999); // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(BN);                              // Including the FuzzySet into FuzzyInput
  FuzzySet *N = new FuzzySet(-1.6459925800000001, -0.70000564, -0.70000564, 0.29994580000000004); // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(N);                               // Including the FuzzySet into FuzzyInput
  FuzzySet *Z = new FuzzySet(-0.40073195000000006, 0.09982578, 0.09982578, 1.09911741);        // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(Z);                               // Including the FuzzySet into FuzzyInput
  FuzzySet *P = new FuzzySet(-0.2996345600000001, 0.70020261, 0.70020261, 1.3569127399999998);    // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(P);                               // Including the FuzzySet into FuzzyInput
  FuzzySet *BP = new FuzzySet(1.0821559299999999, 1.59584143, 1.59584143, 3.16103248);  // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(BP);                              // Including the FuzzySet into FuzzyInput
  FuzzySet *VBP = new FuzzySet(1.8381715100000002, 3.14, 3.14, 3.14);         // Instantiating a FuzzySet object
  deltaTheta->addFuzzySet(VBP);                             // Including the FuzzySet into FuzzyInput
  fuzzy->addFuzzyInput(deltaTheta);                             // Including the FuzzyInput into Fuzzy

  //----------*************---------OUTPUT 1 OMEGA_R-------------**************--------------
  FuzzyOutput *omegaR = new FuzzyOutput(1);                     // Instantiating a FuzzyOutput objects
  FuzzySet *VSR = new FuzzySet(0, 0, 0, 4.00621228);        // Instantiating a FuzzySet object
  omegaR->addFuzzySet(VSR);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *SR = new FuzzySet(1.0978643999999997, 5.09961414, 5.09961414, 9.101749250000001);        // Instantiating a FuzzySet object
  omegaR->addFuzzySet(SR);                         // Including the FuzzySet into FuzzyOutput
  FuzzySet *NMBR = new FuzzySet(3.9002215399999995, 9.90007556, 9.90007556, 13.90781947);    // Instantiating a FuzzySet object
  omegaR->addFuzzySet(NMBR);                       // Including the FuzzySet into FuzzyOutput
  FuzzySet *MBR = new FuzzySet(10.528458449999999, 14.94202322, 14.94202322, 18.957514709999998);    // Instantiating a FuzzySet object
  omegaR->addFuzzySet(MBR);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *NBR = new FuzzySet(13.91591898, 19.91559871, 19.91559871, 25.91528889);    // Instantiating a FuzzySet object
  omegaR->addFuzzySet(NBR);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *BR = new FuzzySet(18.90064177, 24.90025394, 24.90025394, 28.90061799);     // Instantiating a FuzzySet object
  omegaR->addFuzzySet(BR);                         // Including the FuzzySet into FuzzyOutput
  FuzzySet *VBR = new FuzzySet(25.99222212, 30, 30, 30);    // Instantiating a FuzzySet object
  omegaR->addFuzzySet(VBR);                        // Including the FuzzySet into FuzzyOutput
  fuzzy->addFuzzyOutput(omegaR);                   // Including the FuzzyOutput into Fuzzy

  //----------*************---------OUTPUT 2 OMEGA_L-------------**************--------------
  FuzzyOutput *omegaL = new FuzzyOutput(2);                     // Instantiating a FuzzyOutput objects
  FuzzySet *VSL = new FuzzySet(0, 0, 0, 4.00777788);        // Instantiating a FuzzySet object
  omegaL->addFuzzySet(VSL);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *SL = new FuzzySet(0.2941087600000003, 4.99986804, 4.99986804, 9.000543109999999);        // Instantiating a FuzzySet object
  omegaL->addFuzzySet(SL);                         // Including the FuzzySet into FuzzyOutput
  FuzzySet *NMBL = new FuzzySet(3.9012686299999997, 9.90051312, 9.90051312, 13.90118588);    // Instantiating a FuzzySet object
  omegaL->addFuzzySet(NMBL);                       // Including the FuzzySet into FuzzyOutput
  FuzzySet *MBL = new FuzzySet(9.19948241, 14.90000556, 14.90000556, 18.9031506);    // Instantiating a FuzzySet object
  omegaL->addFuzzySet(MBL);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *NBL = new FuzzySet(13.90432741, 19.90353512, 19.90353512, 25.902564830000003);    // Instantiating a FuzzySet object
  omegaL->addFuzzySet(NBL);                        // Including the FuzzySet into FuzzyOutput
  FuzzySet *BL = new FuzzySet(19.66998625, 24.92105317, 24.92105317, 28.92551301);     // Instantiating a FuzzySet object
  omegaL->addFuzzySet(BL);                         // Including the FuzzySet into FuzzyOutput
  FuzzySet *VBL = new FuzzySet(25.413135660000002, 30, 30, 30);    // Instantiating a FuzzySet object
  omegaL->addFuzzySet(VBL);                        // Including the FuzzySet into FuzzyOutput
  fuzzy->addFuzzyOutput(omegaL);                   // Including the FuzzyOutput into Fuzzy
  //||------------------------------------------------------------------------------------------------------------||
  //**********************************BUILDINGB FUZZY RULES FOR OMEGA RIGHT***************************************
  //||------------------------------------------------------------------------------------------------------------||
  // RULE_1R:"IF distance is VC AND deltaTheta is VBN THEN omegaR is VSR ***********RULE 1
  FuzzyRuleAntecedent *ifDistanceVC1 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC1->joinWithAND(VC, VBN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR1 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR1->addOutput(VSR);                                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifDistanceVC1, thenomegaRVSR1);                    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule1);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_2R:"IF distance is VC AND deltaTheta is BN THEN omegaR is SR ******************RULE 2
  FuzzyRuleAntecedent *ifDistanceVC2 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC2->joinWithAND(VC, BN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR2 = new FuzzyRuleConsequent();                             // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR2->addOutput(SR);                                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifDistanceVC2, thenomegaRSR2);                     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule2);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_3R:"IF distance is VC AND deltaTheta is N THEN omegaR is NMBR ******************RULE 3
  FuzzyRuleAntecedent *ifDistanceVC3 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC3->joinWithAND(VC, N);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR3 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR3->addOutput(NMBR);                                                           // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifDistanceVC3, thenomegaRNMBR3);                   // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule3);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_4R:"IF distance is VC AND deltaTheta is Z THEN omegaR is VSR ********************RULE 4
  FuzzyRuleAntecedent *ifDistanceVC4 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC4->joinWithAND(VC, Z);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR4 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR4->addOutput(VSR);                                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifDistanceVC4, thenomegaRVSR4);                    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule4);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_5R:"IF distance is VC AND deltaTheta is P THEN omegaR is NBR *************RULE 5
  FuzzyRuleAntecedent *ifDistanceVC5 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC5->joinWithAND(VC, P);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR5 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR5->addOutput(NBR);                                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifDistanceVC5, thenomegaRNBR5);                    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule5);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_6R:"IF distance is VC AND deltaTheta is BP THEN omegaR is BR *************RULE 6
  FuzzyRuleAntecedent *ifDistanceVC6 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC6->joinWithAND(VC, BP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR6 = new FuzzyRuleConsequent();                             // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR6->addOutput(BR);                                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifDistanceVC6, thenomegaRBR6);                     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule6);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_7R:"IF distance is VC AND deltaTheta is VBP THEN omegaR is VBR *************RULE 7
  FuzzyRuleAntecedent *ifDistanceVC7 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC7->joinWithAND(VC, VBP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR7 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR7->addOutput(VBR);                                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifDistanceVC7, thenomegaRVBR7);                    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule7);                                                            // Including the FuzzyRule into Fuzzy


  // RULE_8R:"IF distance is NC AND deltaTheta is VBN THEN omegaR is VSR ***************RULE 8
  FuzzyRuleAntecedent *ifDistanceNC8 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC8->joinWithAND(NC, VBN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR8 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR8->addOutput(VSR);                                                             // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifDistanceNC8, thenomegaRVSR8);                    // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule8);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_9R:"IF distance is NC AND deltaTheta is BN THEN omegaR is SR **********************RULE 9
  FuzzyRuleAntecedent *ifDistanceNC9 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC9->joinWithAND(NC, BN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR9 = new FuzzyRuleConsequent();                             // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR9->addOutput(SR);                                                               // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifDistanceNC9, thenomegaRSR9);                     // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule9);                                                            // Including the FuzzyRule into Fuzzy

  // RULE_10R:"IF distance is NC AND deltaTheta is N THEN omegaR is NMBR **************************RULE 10
  FuzzyRuleAntecedent *ifDistanceNC10 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC10->joinWithAND(NC, N);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR10 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR10->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifDistanceNC10, thenomegaRNMBR10);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule10);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_11R:"IF distance is NC AND deltaTheta is Z THEN omegaR is SR ************************RULE 11
  FuzzyRuleAntecedent *ifDistanceNC11 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC11->joinWithAND(NC, Z);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR11 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR11->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifDistanceNC11, thenomegaRSR11);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule11);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_12R:"IF distance is NC AND deltaTheta is P THEN omegaR is NBR ***************RULE 12
  FuzzyRuleAntecedent *ifDistanceNC12 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC12->joinWithAND(NC, P);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR12 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR12->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifDistanceNC12, thenomegaRNBR12);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule12);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_13R:"IF distance is NC AND deltaTheta is BP THEN omegaR is BR ************RULE 13
  FuzzyRuleAntecedent *ifDistanceNC13 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC13->joinWithAND(NC, BP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR13 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR13->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifDistanceNC13, thenomegaRBR13);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule13);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_14R:"IF distance is NC AND deltaTheta is VBP THEN omegaR is VBR *******************RULE 14
  FuzzyRuleAntecedent *ifDistanceNC14 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC14->joinWithAND(NC, VBP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR14 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR14->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifDistanceNC14, thenomegaRVBR14);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule14);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_15R:"IF distance is C AND deltaTheta is VBN THEN omegaR is VSR *******************RULE 15
  FuzzyRuleAntecedent *ifDistanceC15 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC15->joinWithAND(C, VBN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR15 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR15->addOutput(VSR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifDistanceC15, thenomegaRVSR15);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule15);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_16R:"IF distance is C AND deltaTheta is BN THEN omegaR is SR *********************RULE 16
  FuzzyRuleAntecedent *ifDistanceC16 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC16->joinWithAND(C, BN);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR16 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR16->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifDistanceC16, thenomegaRSR16);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule16);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_17R:"IF distance is C AND deltaTheta is N THEN omegaR is NMBR **************RULE 17
  FuzzyRuleAntecedent *ifDistanceC17 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC17->joinWithAND(C, N);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR17 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR17->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule17 = new FuzzyRule(15, ifDistanceC17, thenomegaRNMBR17);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule17);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_18R:"IF distance is C AND deltaTheta is Z THEN omegaR is NMBR ***************RULE 18
  FuzzyRuleAntecedent *ifDistanceC18 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC18->joinWithAND(C, Z);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR18 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR18->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifDistanceC18, thenomegaRNMBR18);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule18);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_19R:"IF distance is C19 AND deltaTheta is P THEN omegaR is NBR **********************RULE 19
  FuzzyRuleAntecedent *ifDistanceC19 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC19->joinWithAND(C, P);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR19 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR19->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifDistanceC19, thenomegaRNBR19);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule19);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_20R:"IF distance is C AND deltaTheta is BP THEN omegaR is BR ****************************RULE 20
  FuzzyRuleAntecedent *ifDistanceC20 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC20->joinWithAND(C, BP);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR20 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR20->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, ifDistanceC20, thenomegaRBR20);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule20);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_21R:"IF distance is C AND deltaTheta is VBP THEN omegaR is VBR ************************RULE 21
  FuzzyRuleAntecedent *ifDistanceC21 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC21->joinWithAND(C, VBP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR21 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR21->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, ifDistanceC21, thenomegaRVBR21);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule21);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_22R:"IF distance is M AND deltaTheta is VBN THEN omegaR is VSR *****************RULE 22
  FuzzyRuleAntecedent *ifDistanceM22 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM22->joinWithAND(M, VBN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR22 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR22->addOutput(VSR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, ifDistanceM22, thenomegaRVSR22);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule22);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_23R:"IF distance is M AND deltaTheta is BN THEN omegaR is SR ***********RULE 23
  FuzzyRuleAntecedent *ifDistanceM23 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM23->joinWithAND(M, BN);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR23 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR23->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, ifDistanceM23, thenomegaRSR23);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule23);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_24R:"IF distance is M AND deltaTheta is N THEN omegaR is NMBR ******************RULE 24
  FuzzyRuleAntecedent *ifDistanceM24 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM24->joinWithAND(M, N);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR24 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR24->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, ifDistanceM24, thenomegaRNMBR24);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule24);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_25R:"IF distance is M AND deltaTheta is Z THEN omegaR is MBR ********************RULE 25
  FuzzyRuleAntecedent *ifDistanceM25 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM25->joinWithAND(M, Z);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRMBR25 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRMBR25->addOutput(MBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, ifDistanceM25, thenomegaRMBR25);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule25);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_26R:"IF distance is M AND deltaTheta is P THEN omegaR is NBR ********************RULE 26
  FuzzyRuleAntecedent *ifDistanceM26 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM26->joinWithAND(M, P);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR26 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR26->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, ifDistanceM26, thenomegaRNBR26);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule26);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_27R:"IF distance is M AND deltaTheta is BP THEN omegaR is BR *************RULE 27
  FuzzyRuleAntecedent *ifDistanceM27 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM27->joinWithAND(M, BP);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR27 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR27->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, ifDistanceM27, thenomegaRBR27);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule27);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_28R:"IF distance is M AND deltaTheta is VBP THEN omegaR is VBR *************RULE 28
  FuzzyRuleAntecedent *ifDistanceM28 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM28->joinWithAND(M, VBP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR28 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR28->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, ifDistanceM28, thenomegaRVBR28);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule28);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_29R:"IF distance is NF AND deltaTheta is VBN THEN omegaR is VSR *************RULE 29
  FuzzyRuleAntecedent *ifDistanceNF29 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF29->joinWithAND(NF, VBN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR29 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR29->addOutput(VSR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, ifDistanceNF29, thenomegaRVSR29);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule29);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_30R:"IF distance is NF AND deltaTheta is BN THEN omegaR is SR *************RULE 30
  FuzzyRuleAntecedent *ifDistanceNF30 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF30->joinWithAND(NF, BN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR30 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR30->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, ifDistanceNF30, thenomegaRSR30);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule30);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_31R:"IF distance is NF AND deltaTheta is N THEN omegaR is NMBR *************RULE 31
  FuzzyRuleAntecedent *ifDistanceNF31 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF31->joinWithAND(NF, N);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR31 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR31->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, ifDistanceNF31, thenomegaRNMBR31);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule31);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_32R:"IF distance is NF AND deltaTheta is Z THEN omegaR is NBR *************RULE 32
  FuzzyRuleAntecedent *ifDistanceNF32 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF32->joinWithAND(NF, Z);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR32 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR32->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, ifDistanceNF32, thenomegaRNBR32);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule32);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_33R:"IF distance is NF AND deltaTheta is P THEN omegaR is NBR *************RULE 33
  FuzzyRuleAntecedent *ifDistanceNF33 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF33->joinWithAND(NF, P);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR33 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR33->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, ifDistanceNF33, thenomegaRNBR33);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule33);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_34R:"IF distance is NF AND deltaTheta is BP THEN omegaR is BR *************RULE 34
  FuzzyRuleAntecedent *ifDistanceNF34 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF34->joinWithAND(NF, BP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR34 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR34->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, ifDistanceNF34, thenomegaRBR34);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule34);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_35R:"IF distance is NF AND deltaTheta is VBP THEN omegaR is VBR *************RULE 35
  FuzzyRuleAntecedent *ifDistanceNF35 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF35->joinWithAND(NF, VBP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR35 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR35->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, ifDistanceNF35, thenomegaRVBR35);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule35);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_36R:"IF distance is F AND deltaTheta is VBN THEN omegaR is VSR *************RULE 36
  FuzzyRuleAntecedent *ifDistanceF36 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF36->joinWithAND(F, VBN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR36 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR36->addOutput(VSR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, ifDistanceF36, thenomegaRVSR36);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule36);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_37R:"IF distance is F AND deltaTheta is BN THEN omegaR is SR *************RULE 37
  FuzzyRuleAntecedent *ifDistanceF37 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF37->joinWithAND(F, BN);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR37 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR37->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, ifDistanceF37, thenomegaRSR37);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule37);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_38R:"IF distance is F AND deltaTheta is N THEN omegaR is NMBR *************RULE 38
  FuzzyRuleAntecedent *ifDistanceF38 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF38->joinWithAND(F, N);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR38 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR38->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, ifDistanceF38, thenomegaRNMBR38);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule38);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_39R:"IF distance is F AND deltaTheta is Z THEN omegaR is BR *************RULE 39
  FuzzyRuleAntecedent *ifDistanceF39 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF39->joinWithAND(F, Z);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR39 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR39->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, ifDistanceF39, thenomegaRBR39);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule39);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_40R:"IF distance is F AND deltaTheta is P THEN omegaR is NBR *************RULE 40
  FuzzyRuleAntecedent *ifDistanceF40 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF40->joinWithAND(F, P);                                                           // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR40 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR40->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, ifDistanceF40, thenomegaRNBR40);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule40);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_41R:"IF distance is F AND deltaTheta is BP THEN omegaR is BR *************RULE 41
  FuzzyRuleAntecedent *ifDistanceF41 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF41->joinWithAND(F, BP);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR41 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR41->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, ifDistanceF41, thenomegaRBR41);                  // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule41);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_42R:"IF distance is F AND deltaTheta is VBP THEN omegaR is VBR *************RULE 42
  FuzzyRuleAntecedent *ifDistanceF42 = new FuzzyRuleAntecedent();                             // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF42->joinWithAND(F, VBP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR42 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR42->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, ifDistanceF42, thenomegaRVBR42);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule42);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_43R:"IF distance is VF AND deltaTheta is VBN THEN omegaR is VSR *************RULE 43
  FuzzyRuleAntecedent *ifDistanceVF43 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF43->joinWithAND(VF, VBN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVSR43 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVSR43->addOutput(VSR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, ifDistanceVF43, thenomegaRVSR43);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule43);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_44R:"IF distance is VF AND deltaTheta is BN THEN omegaR is SR *************RULE 44
  FuzzyRuleAntecedent *ifDistanceVF44 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF44->joinWithAND(VF, BN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRSR44 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRSR44->addOutput(SR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, ifDistanceVF44, thenomegaRSR44);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule44);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_45R:"IF distance is VF AND deltaTheta is N THEN omegaR is NMBR *************RULE 45
  FuzzyRuleAntecedent *ifDistanceVF45 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF45->joinWithAND(VF, N);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNMBR45 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNMBR45->addOutput(NMBR);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, ifDistanceVF45, thenomegaRNMBR45);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule45);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_46R:"IF distance is VF AND deltaTheta is Z THEN omegaR is VBR *************RULE 46
  FuzzyRuleAntecedent *ifDistanceVF46 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF46->joinWithAND(VF, Z);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR46 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR46->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, ifDistanceVF46, thenomegaRVBR46);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule46);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_47R:"IF distance is VF AND deltaTheta is P THEN omegaR is NBR *************RULE 47
  FuzzyRuleAntecedent *ifDistanceVF47 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF47->joinWithAND(VF, P);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRNBR47 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRNBR47->addOutput(NBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, ifDistanceVF47, thenomegaRNBR47);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule47);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_48R:"IF distance is VF AND deltaTheta is BP THEN omegaR is BR *************RULE 48
  FuzzyRuleAntecedent *ifDistanceVF48 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF48->joinWithAND(VF, BP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRBR48 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaRBR48->addOutput(BR);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, ifDistanceVF48, thenomegaRBR48);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule48);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_49R:"IF distance is VF AND deltaTheta is VBP THEN omegaR is VBR *************RULE 49
  FuzzyRuleAntecedent *ifDistanceVF49 = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF49->joinWithAND(VF, VBP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaRVBR49 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaRVBR49->addOutput(VBR);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, ifDistanceVF49, thenomegaRVBR49);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule49);                                                           // Including the FuzzyRule into Fuzzy

  //|-----------------------------------------------------------------------------------------------------------|
  //**********************************BUILDINGB FUZZY RULES FOR OMEGA LEFT***************************************
  //|-----------------------------------------------------------------------------------------------------------|

  // RULE_1L:"IF distance is VC AND deltaTheta is VBN THEN omegaL is VBL ***********RULE 50
  FuzzyRuleAntecedent *ifDistanceVC1L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC1L->joinWithAND(VC, VBN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL50 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL50->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule50 = new FuzzyRule(50, ifDistanceVC1L, thenomegaLVBL50);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule50);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_2L:"IF distance is VC AND deltaTheta is BN THEN omegaL is BL ******************RULE 51
  FuzzyRuleAntecedent *ifDistanceVC2L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC2L->joinWithAND(VC, BN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL51 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL51->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule51 = new FuzzyRule(51, ifDistanceVC2L, thenomegaLBL51);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule51);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_3L:"IF distance is VC AND deltaTheta is N THEN omegaL is NBL ******************RULE 52
  FuzzyRuleAntecedent *ifDistanceVC3L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC3L->joinWithAND(VC, N);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL52 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL52->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule52 = new FuzzyRule(52, ifDistanceVC3L, thenomegaLNBL52);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule52);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_4L:"IF distance is VC AND deltaTheta is Z THEN omegaL is VSL ********************RULE 53
  FuzzyRuleAntecedent *ifDistanceVC4L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC4L->joinWithAND(VC, Z);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL53 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL53->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule53 = new FuzzyRule(53, ifDistanceVC4L, thenomegaLVSL53);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule53);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_5L:"IF distance is VC AND deltaTheta is P THEN omegaL is NMBL *************RULE 54
  FuzzyRuleAntecedent *ifDistanceVC5L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC5L->joinWithAND(VC, P);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL54 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL54->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule54 = new FuzzyRule(54, ifDistanceVC5L, thenomegaLNMBL54);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule54);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_6L:"IF distance is VC AND deltaTheta is BP THEN omegaL is SL *************RULE 55
  FuzzyRuleAntecedent *ifDistanceVC6L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC6L->joinWithAND(VC, BP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL55 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL55->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule55 = new FuzzyRule(55, ifDistanceVC6L, thenomegaLSL55);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule55);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_7L:"IF distance is VC AND deltaTheta is VBP THEN omegaL is VSL *************RULE 56
  FuzzyRuleAntecedent *ifDistanceVC7L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVC7L->joinWithAND(VC, VBP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL56 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL56->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule56 = new FuzzyRule(56, ifDistanceVC7L, thenomegaLVSL56);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule56);                                                           // Including the FuzzyRule into Fuzzy


  // RULE_8L:"IF distance is NC AND deltaTheta is VBN THEN omegaL is VBL ***************RULE 57
  FuzzyRuleAntecedent *ifDistanceNC8L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC8L->joinWithAND(NC, VBN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL57 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL57->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule57 = new FuzzyRule(57, ifDistanceNC8L, thenomegaLVBL57);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule57);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_9L:"IF distance is NC AND deltaTheta is BN THEN omegaL is BL **********************RULE 58
  FuzzyRuleAntecedent *ifDistanceNC9L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC9L->joinWithAND(NC, BN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL58 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL58->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule58 = new FuzzyRule(58, ifDistanceNC9L, thenomegaLBL58);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule58);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_10L:"IF distance is NC AND deltaTheta is N THEN omegaL is NBL **************************RULE 59
  FuzzyRuleAntecedent *ifDistanceNC10L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC10L->joinWithAND(NC, N);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL59 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL59->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule59 = new FuzzyRule(59, ifDistanceNC10L, thenomegaLNBL59);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule59);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_11L:"IF distance is NC AND deltaTheta is Z THEN omegaL is SL ************************RULE 60
  FuzzyRuleAntecedent *ifDistanceNC11L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC11L->joinWithAND(NC, Z);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL60 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL60->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule60 = new FuzzyRule(60, ifDistanceNC11L, thenomegaLSL60);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule60);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_12L:"IF distance is NC AND deltaTheta is P THEN omegaL is NMBL ***************RULE 61
  FuzzyRuleAntecedent *ifDistanceNC12L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC12L->joinWithAND(NC, P);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL61 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL61->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule61 = new FuzzyRule(61, ifDistanceNC12L, thenomegaLNMBL61);              // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule61);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_13L:"IF distance is NC AND deltaTheta is BP THEN omegaL is SL ************RULE 62
  FuzzyRuleAntecedent *ifDistanceNC13L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC13L->joinWithAND(NC, BP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL62 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL62->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule62 = new FuzzyRule(62, ifDistanceNC13L, thenomegaLSL62);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule62);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_14L:"IF distance is NC AND deltaTheta is VBP THEN omegaL is VSL *******************RULE 63
  FuzzyRuleAntecedent *ifDistanceNC14L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNC14L->joinWithAND(NC, VBP);                                                      // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL63 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL63->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule63 = new FuzzyRule(63, ifDistanceNC14L, thenomegaLVSL63);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule63);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_15L:"IF distance is C AND deltaTheta is VBN THEN omegaL is VBL *******************RULE 64
  FuzzyRuleAntecedent *ifDistanceC15L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC15L->joinWithAND(C, VBN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL64 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL64->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule64 = new FuzzyRule(64, ifDistanceC15L, thenomegaLVBL64);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule64);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_16L:"IF distance is C AND deltaTheta is BN THEN omegaL is BL *********************RULE 65
  FuzzyRuleAntecedent *ifDistanceC16L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC16L->joinWithAND(C, BN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL65 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL65->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule65 = new FuzzyRule(65, ifDistanceC16L, thenomegaLBL65);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule65);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_17L:"IF distance is C AND deltaTheta is N THEN omegaL is NBL **************RULE 66
  FuzzyRuleAntecedent *ifDistanceC17L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC17L->joinWithAND(C, N);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL66 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL66->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule66 = new FuzzyRule(66, ifDistanceC17L, thenomegaLNBL66);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule66);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_18L:"IF distance is C AND deltaTheta is Z THEN omegaL is NMBL ***************RULE 67
  FuzzyRuleAntecedent *ifDistanceC18L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC18L->joinWithAND(C, Z);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL67 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL67->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule67 = new FuzzyRule(67, ifDistanceC18L, thenomegaLNMBL67);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule67);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_19L:"IF distance is C19 AND deltaTheta is P THEN omegaL is NMBL **********************RULE 68
  FuzzyRuleAntecedent *ifDistanceC19L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC19L->joinWithAND(C, P);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL68 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL68->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule68 = new FuzzyRule(68, ifDistanceC19L, thenomegaLNMBL68);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule68);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_20L:"IF distance is C AND deltaTheta is BP THEN omegaL is SL ****************************RULE 69
  FuzzyRuleAntecedent *ifDistanceC20L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC20L->joinWithAND(C, BP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL69 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL69->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule69 = new FuzzyRule(69, ifDistanceC20L, thenomegaLSL69);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule69);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_21L:"IF distance is C AND deltaTheta is VBP THEN omegaL is VSL ************************RULE 70
  FuzzyRuleAntecedent *ifDistanceC21L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceC21L->joinWithAND(C, VBP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL70 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL70->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule70 = new FuzzyRule(70, ifDistanceC21L, thenomegaLVSL70);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule70);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_22L:"IF distance is M AND deltaTheta is VBN THEN omegaL is VBL *****************RULE 71
  FuzzyRuleAntecedent *ifDistanceM22L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM22L->joinWithAND(M, VBN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL71 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL71->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule71 = new FuzzyRule(71, ifDistanceM22L, thenomegaLVBL71);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule71);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_23L:"IF distance is M AND deltaTheta is BN THEN omegaL is BL ***********RULE 72
  FuzzyRuleAntecedent *ifDistanceM23L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM23L->joinWithAND(M, BN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL72 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL72->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule72 = new FuzzyRule(72, ifDistanceM23L, thenomegaLBL72);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule72);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_24L:"IF distance is M AND deltaTheta is N THEN omegaL is NBL ******************RULE 73
  FuzzyRuleAntecedent *ifDistanceM24L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM24L->joinWithAND(M, N);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL73 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL73->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule73 = new FuzzyRule(73, ifDistanceM24L, thenomegaLNBL73);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule73);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_25L:"IF distance is M AND deltaTheta is Z THEN omegaL is MBL ********************RULE 74
  FuzzyRuleAntecedent *ifDistanceM25L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM25L->joinWithAND(M, Z);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLMBL74 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLMBL74->addOutput(MBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule74 = new FuzzyRule(74, ifDistanceM25L, thenomegaLMBL74);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule74);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_26L:"IF distance is M AND deltaTheta is P THEN omegaL is NMBL ********************RULE 75
  FuzzyRuleAntecedent *ifDistanceM26L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM26L->joinWithAND(M, P);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL75 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL75->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule75 = new FuzzyRule(75, ifDistanceM26L, thenomegaLNMBL75);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule75);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_27L:"IF distance is M AND deltaTheta is BP THEN omegaL is SL *************RULE 76
  FuzzyRuleAntecedent *ifDistanceM27L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM27L->joinWithAND(M, BP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL76 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL76->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule76 = new FuzzyRule(76, ifDistanceM27L, thenomegaLSL76);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule76);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_28L:"IF distance is M AND deltaTheta is VBP THEN omegaL is VSL *************RULE 77
  FuzzyRuleAntecedent *ifDistanceM28L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceM28L->joinWithAND(M, VBP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL77 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL77->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule77 = new FuzzyRule(77, ifDistanceM28L, thenomegaLVSL77);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule77);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_29L:"IF distance is NF AND deltaTheta is VBN THEN omegaL is VBL *************RULE 78
  FuzzyRuleAntecedent *ifDistanceNF29L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF29L->joinWithAND(NF, VBN);                                                      // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL78 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL78->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule78 = new FuzzyRule(78, ifDistanceNF29L, thenomegaLVBL78);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule78);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_30L:"IF distance is NF AND deltaTheta is BN THEN omegaL is BL *************RULE 79
  FuzzyRuleAntecedent *ifDistanceNF30L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF30L->joinWithAND(NF, BN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL79 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL79->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule79 = new FuzzyRule(79, ifDistanceNF30L, thenomegaLBL79);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule79);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_31L:"IF distance is NF AND deltaTheta is N THEN omegaL is NBL *************RULE 80
  FuzzyRuleAntecedent *ifDistanceNF31L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF31L->joinWithAND(NF, N);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL80 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL80->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule80 = new FuzzyRule(80, ifDistanceNF31L, thenomegaLNBL80);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule80);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_32L:"IF distance is NF AND deltaTheta is Z THEN omegaL is NBL *************RULE 81
  FuzzyRuleAntecedent *ifDistanceNF32L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF32L->joinWithAND(NF, Z);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL81 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL81->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule81 = new FuzzyRule(81, ifDistanceNF32L, thenomegaLNBL81);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule81);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_33L:"IF distance is NF AND deltaTheta is P THEN omegaL is NMBL *************RULE 82
  FuzzyRuleAntecedent *ifDistanceNF33L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF33L->joinWithAND(NF, P);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL82 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL82->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule82 = new FuzzyRule(82, ifDistanceNF33L, thenomegaLNMBL82);              // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule82);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_34L:"IF distance is NF AND deltaTheta is BP THEN omegaL is SL *************RULE 83
  FuzzyRuleAntecedent *ifDistanceNF34L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF34L->joinWithAND(NF, BP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL83 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL83->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule83 = new FuzzyRule(83, ifDistanceNF34L, thenomegaLSL83);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule83);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_35L:"IF distance is NF AND deltaTheta is VBP THEN omegaL is VSL *************RULE 84
  FuzzyRuleAntecedent *ifDistanceNF35L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceNF35L->joinWithAND(NF, VBP);                                                      // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL84 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL84->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule84 = new FuzzyRule(84, ifDistanceNF35L, thenomegaLVSL84);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule84);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_36L:"IF distance is F AND deltaTheta is VBN THEN omegaL is VBL *************RULE 85
  FuzzyRuleAntecedent *ifDistanceF36L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF36L->joinWithAND(F, VBN);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL85 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL85->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule85 = new FuzzyRule(85, ifDistanceF36L, thenomegaLVBL85);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule85);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_37L:"IF distance is F AND deltaTheta is BN THEN omegaL is BL *************RULE 86
  FuzzyRuleAntecedent *ifDistanceF37L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF37L->joinWithAND(F, BN);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL86 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL86->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule86 = new FuzzyRule(86, ifDistanceF37L, thenomegaLBL86);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule86);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_38L:"IF distance is F AND deltaTheta is N THEN omegaL is NBL *************RULE 87
  FuzzyRuleAntecedent *ifDistanceF38L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF38L->joinWithAND(F, N);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL87 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL87->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule87 = new FuzzyRule(87, ifDistanceF38L, thenomegaLNBL87);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule87);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_39L:"IF distance is F AND deltaTheta is Z THEN omegaL is BL *************RULE 88
  FuzzyRuleAntecedent *ifDistanceF39L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF39L->joinWithAND(F, Z);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL88 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL88->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule88 = new FuzzyRule(88, ifDistanceF39L, thenomegaLBL88);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule88);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_40L:"IF distance is F AND deltaTheta is P THEN omegaL is NMBL *************RULE 89
  FuzzyRuleAntecedent *ifDistanceF40L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF40L->joinWithAND(F, P);                                                          // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL89 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL89->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule89 = new FuzzyRule(89, ifDistanceF40L, thenomegaLNMBL89);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule89);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_41L:"IF distance is F AND deltaTheta is BP THEN omegaL is SL *************RULE 90
  FuzzyRuleAntecedent *ifDistanceF41L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF41L->joinWithAND(F, BP);                                                         // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL90 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL90->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule90 = new FuzzyRule(90, ifDistanceF41L, thenomegaLSL90);                 // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule90);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_42L:"IF distance is F AND deltaTheta is VBP THEN omegaL is VSL *************RULE 91
  FuzzyRuleAntecedent *ifDistanceF42L = new FuzzyRuleAntecedent();                            // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceF42L->joinWithAND(F, VBP);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL91 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL91->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule91 = new FuzzyRule(91, ifDistanceF42L, thenomegaLVSL91);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule91);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_43L:"IF distance is VF AND deltaTheta is VBN THEN omegaL is VBL *************RULE 92
  FuzzyRuleAntecedent *ifDistanceVF43L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF43L->joinWithAND(VF, VBN);                                                      // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL92 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL92->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule92 = new FuzzyRule(92, ifDistanceVF43L, thenomegaLVBL92);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule92);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_44L:"IF distance is VF AND deltaTheta is BN THEN omegaL is BL *************RULE 93
  FuzzyRuleAntecedent *ifDistanceVF44L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF44L->joinWithAND(VF, BN);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLBL93 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLBL93->addOutput(BL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule93 = new FuzzyRule(93, ifDistanceVF44L, thenomegaLBL93);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule93);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_45L:"IF distance is VF AND deltaTheta is N THEN omegaL is NBL *************RULE 94
  FuzzyRuleAntecedent *ifDistanceVF45L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF45L->joinWithAND(VF, N);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNBL94 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNBL94->addOutput(NBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule94 = new FuzzyRule(94, ifDistanceVF45L, thenomegaLNBL94);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule94);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_46L:"IF distance is VF AND deltaTheta is Z THEN omegaL is VBL *************RULE 95
  FuzzyRuleAntecedent *ifDistanceVF46L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF46L->joinWithAND(VF, Z);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVBL95 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVBL95->addOutput(VBL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule95 = new FuzzyRule(95, ifDistanceVF46L, thenomegaLVBL95);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule95);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_47L:"IF distance is VF AND deltaTheta is P THEN omegaL is NMBL *************RULE 96
  FuzzyRuleAntecedent *ifDistanceVF47L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF47L->joinWithAND(VF, P);                                                        // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLNMBL96 = new FuzzyRuleConsequent();                          // Instantiating a FuzzyRuleConsequent objects
  thenomegaLNMBL96->addOutput(NMBL);                                                          // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule96 = new FuzzyRule(96, ifDistanceVF47L, thenomegaLNMBL96);              // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule96);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_48L:"IF distance is VF AND deltaTheta is BP THEN omegaL is SL *************RULE 97
  FuzzyRuleAntecedent *ifDistanceVF48L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF48L->joinWithAND(VF, BP);                                                       // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLSL97 = new FuzzyRuleConsequent();                            // Instantiating a FuzzyRuleConsequent objects
  thenomegaLSL97->addOutput(SL);                                                              // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule97 = new FuzzyRule(97, ifDistanceVF48L, thenomegaLSL97);                // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule97);                                                           // Including the FuzzyRule into Fuzzy

  // RULE_49L:"IF distance is VF AND deltaTheta is VBP THEN omegaL is VSL *************RULE 98
  FuzzyRuleAntecedent *ifDistanceVF49L = new FuzzyRuleAntecedent();                           // Instantiating a FuzzyRuleAntecedent objects
  ifDistanceVF49L->joinWithAND(VF, VBP);                                                      // Creating a FuzzyRuleAntecedent with 02 FuzzySet
  FuzzyRuleConsequent *thenomegaLVSL98 = new FuzzyRuleConsequent();                           // Instantiating a FuzzyRuleConsequent objects
  thenomegaLVSL98->addOutput(VSL);                                                            // Including a FuzzySet to this FuzzyRuleConsequent
  FuzzyRule *fuzzyRule98 = new FuzzyRule(98, ifDistanceVF49L, thenomegaLVSL98);               // Instantiating a FuzzyRule objects
  fuzzy->addFuzzyRule(fuzzyRule98);                                                           // Including the FuzzyRule into Fuzzy
  //|---------------------------------------------------------------------------------------------------------|
  // FFFFFFFFFFFFFFFFFFFFFFFFFFFFF***KẾT THÚC SETUP CHO FUZZY LOGIC CONTROLLER***FFFFFFFFFFFFFFFFFFFFFFFFFFFFF
  //|---------------------------------------------------------------------------------------------------------|
  Kp_R = 0.926;
  Ki_R = 9.276;
  Kd_R = 0.025;

  Kp_L = 0.753;
  Ki_L = 8.528;
  Kd_L = 0.015;

  timer = micros();// For start count dT
} //void setup


//--------------------------------------------------------------------------------------------------
//********************************* LOOP ******************************************
//--------------------------------------------------------------------------------------------------
void loop()
{
  // ---------------------------Rotating Motors------------------------------------
  digitalWrite(IN1_MOTOR_LEFT, HIGH);
  digitalWrite(IN2_MOTOR_LEFT, LOW);
  digitalWrite(IN3_MOTOR_RIGHT, HIGH);
  digitalWrite(IN4_MOTOR_RIGHT, LOW);

  currentMillis_Right = millis();
  currentMillis_Left = millis();

  if ((currentMillis_Right - previousMillis_Right > interval) && (currentMillis_Left - previousMillis_Left > interval))
  {
    previousMillis_Right = currentMillis_Right;
    rpm_Right = (encoderValue_Right * 1200 / ENC_COUNT_REV_Right);// Calculate RPM
    Omega_Right = (rpm_Right * 2 * 3.14) / 60;// Converting rpm_Right to radian per second
    Omega_Right = Kalman_Filter_Velocity.updateEstimate(Omega_Right);
    previousMillis_Left = currentMillis_Left;
    rpm_Left = (encoderValue_Left * 1200 / ENC_COUNT_REV_Left);// Calculate RPM
    Omega_Left = (rpm_Left * 2 * 3.14) / 60;// Converting rpm_Left to radian per second
    Omega_Left = Kalman_Filter_Velocity.updateEstimate(Omega_Left);
    encoderValue_Left = 0;
    encoderValue_Right = 0;

    String s = Serial1.readString();//reads characters from the serial buffer into a String. The function terminates if it times out
    if (s.length() != 0)
    {
      float distance = (s.substring(s.indexOf('d') + 4, s.indexOf(';'))).toFloat();
      float angle = (s.substring(s.indexOf('a') + 4, s.indexOf('/'))).toFloat();
      if (0 < distance)
      {
        fuzzy->setInput(1, distance);                                   // Set the distance value as the input1
        fuzzy->setInput(2, angle);                                 // Set the deltaTheta value as the input2
        fuzzy->fuzzify();                                               // Running the Fuzzification
        omega_FLC_R = fuzzy->defuzzify(1);                 // Running the Defuzzification for OmegaR: output1. This is output of FLC
        omega_FLC_L = fuzzy->defuzzify(2);                 // Running the Defuzzification for OmegaL: output2. This is output of FLC
        omega_FLC_R = Kalman_Filter_Velocity.updateEstimate(omega_FLC_R);
        omega_FLC_L = Kalman_Filter_Velocity.updateEstimate(omega_FLC_L);

        error_R = omega_FLC_R - Omega_Right;
        sum_error_R = sum_error_R + error_R;
        omega_PID_R = Kp_R * error_R + Ki_R * sum_error_R + Kd_R * (error_R - last_error_R);

        error_L = omega_FLC_L - Omega_Left;
        sum_error_L = sum_error_L + error_L;
        omega_PID_L = Kp_L * error_L + Ki_L * sum_error_L + Kd_L * (error_L - last_error_L);

        PWMr = map(omega_PID_R, 0, 30, 0, 255);                
        PWMl = map(omega_PID_L, 0, 30, 0, 255);                 

        last_error_R = error_R;
        last_error_L = error_L;

        analogWrite(ENA_MOTOR_RIGHT, PWMr); // For right wheel
        analogWrite(ENA_MOTOR_LEFT, PWMl);  // For left wheel
      }
      else
      {
        distance = 0;
        angle = 0;
        float omegaR = 0;                          
        float omegaL = 0;                          
        float PWMr = 0;                 
        float PWMl = 0;                 
        analogWrite(ENA_MOTOR_RIGHT, PWMr); // For right wheel
        analogWrite(ENA_MOTOR_LEFT, PWMl);  // For left wheel
      }
    }//if (s.length() != 0)
    dT = (micros() - timer) / 1000000.0;         // update dT, division to convert from micro to seconds
    timer = micros(); // update timer
    Serial.print(dT);
    Serial.println();
  }//if ((currentMillis_Right - previousMillis_Right > interval) && (currentMillis_Left - previousMillis_Left > interval))  
}//loop

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
