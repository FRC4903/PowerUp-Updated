//code too run team 4903's PowerUp robot -- updated to 2022
#include "frc/TimedRobot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <units/current.h>

using namespace frc;
using namespace std;

class Robot: public TimedRobot {
  public:

  TalonSRX RDrive1; //right drive motors
  TalonSRX RDrive2;
  TalonSRX LDrive1; //left drive motors (180 degree turn, run backwards)
  TalonSRX LDrive2;
  TalonSRX CHeight; //height of the cube elevator, runs backwards
  TalonSRX LGrabby; //grabby wheels on the cube elevator, left runs backwards
  TalonSRX RGrabby;
  TalonSRX Climb1;  //climb height motors
  TalonSRX Climb2;
  Joystick j1{0};  //main joystick

  DigitalInput climbTop;   //sensor at the top of the climb
  DigitalInput climbBottom; //sensor at the bottom of the climb

  DoubleSolenoid testsolenoid{PneumaticsModuleType::CTREPCM, 6, 1}; //arm fold out solenoid
  Compressor compressor{0, PneumaticsModuleType::CTREPCM}; //compressor

  float LSpeed = 0; //speed of the left drive motors, stored for acceleration and deceleration
  float RSpeed = 0; //speed of the rght drive motors, same deal

  //compressor variables - explain themselves
  bool compressorEnabled = compressor.Enabled();
  bool pressureSwitch = compressor.GetPressureSwitchValue();
  double compressorCurrent = units::unit_cast<double>(compressor.GetCurrent());

  /*
  1 is right drive
  2 is broken right drive
  3 is nothing
  4 is left (backwards)
  5 also left
  6 is arm height
  7 is left grabby
  8 is right grabby
  9 is climb height
  10 is climb height
  11 is nothing
  */

  Robot():
    RDrive1(1), RDrive2(2), LDrive1(4), LDrive2(5), CHeight(6), LGrabby(7), RGrabby(8), Climb1(9), Climb2(10), climbTop(2), climbBottom(3)
  {
   
  }

void RobotInit() {
  //sets motors to 0, stops anything from screwing up
  RDrive1.Set(ControlMode::PercentOutput, 0);
  RDrive2.Set(ControlMode::PercentOutput, 0);
  LDrive1.Set(ControlMode::PercentOutput, 0);
  LDrive2.Set(ControlMode::PercentOutput, 0);
  CHeight.Set(ControlMode::PercentOutput, 0);
  LGrabby.Set(ControlMode::PercentOutput, 0);
  RGrabby.Set(ControlMode::PercentOutput, 0);
  Climb1.Set(ControlMode::PercentOutput, 0);
  Climb2.Set(ControlMode::PercentOutput, 0);
}

void RobotPeriodic() {
  //updates compressor variables and outputs them
  compressorEnabled = compressor.Enabled();
  pressureSwitch = compressor.GetPressureSwitchValue();
  compressorCurrent = units::unit_cast<double>(compressor.GetCurrent());
  SmartDashboard::PutNumber("pressure", units::unit_cast<double>(compressor.GetPressure()));
  SmartDashboard::PutNumber("current", compressorCurrent);
  SmartDashboard::PutBoolean("pressure switch", pressureSwitch);
}

void AutonomousInit() {
  //run at the beginning of autonomous period just once
  //once again useful for timers and resetting motors
}

void AutonomousPeriodic() {
 //this is where you put the bulk of your auto code, I reccommend for your sanity that you make a new method for each auto and just use this to select them
}

void TeleopInit() {
  //runs once at the beginning of the teleoperated period
  //make sure auto stuff is off and that everything you want for teleop is set up
}

void TeleopPeriodic() {
  //gets joystick values
  float y = j1.GetRawAxis(1); //up and down on the left joystick 
  float x = j1.GetRawAxis(2); //left and right on the right joystick

  DrivePeriodic(y, x, 0.5); //drives at half power, so we don't blow stuff up yet

  int pneumaticDirection = j1.GetRawButton(1) - j1.GetRawButton(3); //out on 1, in on 3

  if (pneumaticDirection == 1) {                        //if the mode is 1 it runs forward
    testsolenoid.Set(DoubleSolenoid::Value::kForward);
  } else if (pneumaticDirection == -1) {                //if mode is -1 it runs reverse
    testsolenoid.Set(DoubleSolenoid::Value::kReverse);
  } else {                                              //make sure to turn it off otherwise just in case
    testsolenoid.Set(DoubleSolenoid::Value::kOff);
  }
  compressor.Disable();

  //cube elevator height control
  if (j1.GetPOV() == 0) {                           //if the direction key is up
    CHeight.Set(ControlMode::PercentOutput, -0.25); //run the motor backwards, moves the arm up
  } else if (j1.GetPOV() == 180) {                  //if the direction is down
    CHeight.Set(ControlMode::PercentOutput, 0.25);  //run motor forwards, arm goes down
  } else {                                          //any other time turn off the motor
    CHeight.Set(ControlMode::PercentOutput, 0);
  }
  
  float intakeSpeed = j1.GetRawButton(7) - j1.GetRawButton(8); //if button 7 is pressed, goes forward, if 8 is pressed goes backwards
  RGrabby.Set(ControlMode::PercentOutput, intakeSpeed);        //runs right side forwards
  LGrabby.Set(ControlMode::PercentOutput, -intakeSpeed);       //left back
  
  float climbspeed = j1.GetRawButton(6) - j1.GetRawButton(5); //climb runs backwards if button 5 is pressed, and forwards if 6 is
  climbspeed *= abs(climbspeed) * 0.2;                        //at 20% speed for safety
  
  Climb1.Set(ControlMode::PercentOutput, climbspeed);         //both run the same way
  Climb2.Set(ControlMode::PercentOutput, climbspeed);

  SmartDashboard::PutNumber("climbtop", climbTop.Get());     //gets sensor input
  SmartDashboard::PutNumber("climbbot", climbBottom.Get());
}

void DrivePeriodic(float y, float x, float speed) {         //copy/paste tank drive
  x=x*abs(x)*0.5;                                           //squares inputs and keeps the direction, x is half speed so it doesnt turn like crazy
  y=y*abs(y);

  LSpeed = max(-1.0f, min(1.0f, -1.0f*speed*((y-x)/25)+(LSpeed*24/25)));  //clamps speeds between -1 and 1
  RSpeed = max(-1.0f, min(1.0f, speed*((y+x)/25)+(RSpeed*24/25)));        //accelerates and decelerates 

  LDrive1.Set(ControlMode::PercentOutput, LSpeed);                        //literally just sets each motor to the speed for its side
  LDrive2.Set(ControlMode::PercentOutput, LSpeed);
  RDrive1.Set(ControlMode::PercentOutput, RSpeed);
  RDrive2.Set(ControlMode::PercentOutput, RSpeed);
}

void TestInit() {
  //actual useful stuff here
  //dont ignore these Im serious
  //run these like you would teleop or auto but save your working code above
}

void TestPeriodic() {
  bool compressorMode = j1.GetRawButton(1);   //when you press 1, the compressor can turn on
  if (!pressureSwitch && compressorMode) {    //if the pressure switch isnt on, run the compressor
    compressor.EnableDigital();
  } else {                                    //otherwise disable it
    compressor.Disable();
  }
}

};

//this little chunk stays always, it's what actually starts your code
#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif