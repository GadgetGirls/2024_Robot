// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

void Robot::RobotInit() {
 
  SpeedThrottlePercentage = .5;
  TurboEnabled = 0;
  inches_per_rotation = 1.91; // derived from measurement

  frc::CameraServer::StartAutomaticCapture();
  leftLeadMotor.RestoreFactoryDefaults();
  rightLeadMotor.RestoreFactoryDefaults();
  leftFollowMotor.RestoreFactoryDefaults();
  rightFollowMotor.RestoreFactoryDefaults();
  rightLeadMotor.SetInverted(true);
  followTelescopeElevatorMotor.SetInverted(true); // This should be the right motor!
  leftLeadEncoder.SetPosition(0); 
  rightLeadEncoder.SetPosition(0);
  rioGyro.Reset();
  m_chooser.AddOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.SetDefaultOption(kAutoNameCenter, kAutoNameCenter);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  


 /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     */
    leftFollowMotor.Follow(leftLeadMotor);
    rightFollowMotor.Follow(rightLeadMotor);
    followTelescopeElevatorMotor.Follow(leadTelescopeElevatorMotor);


  // WF- gyro.GetAngle() returns the yaw axis.  Since we want to measure
  // robot tilt in autonomous mode we want to set the yaw to this axis.
  // We determined through experimentation that the X axis on the gyro
  // measures our tilt.
  rioGyro.SetYawAxis(frc::ADIS16470_IMU::kZ);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  rotate_180_done = 0;
  drive_done = 0;
  initial_tilt_detected = 0;
  pid_rotation.Reset();
  pid_drive.Reset();
  rioGyro.Reset();
  pid_out = 0;
  last_pid_out = 0;
  damping_factor = 1.0;
  payload_dump_done = 0;
  timer0.Reset();
  outside_of_home = 0;
  

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  
  if (m_autoSelected == kAutoNameCenter) {
    // Center Auto goes here.  Empty for now.
  } else {
    // Default Auto goes here. Empty for now.
  }
}

void Robot::AutonomousPeriodic() {
  timer0.Start();
   m_autoSelected = m_chooser.GetSelected();
  robot_angle = double(rioGyro.GetAngle());
  // Use this for debug purposes.  Remove before competition
   frc::SmartDashboard::PutNumber("GyroYaw", robot_angle);
 x_complementary_angle =  double(rioGyro.GetXComplementaryAngle());
  y_complementary_angle =  double(rioGyro.GetYComplementaryAngle());

  frc::SmartDashboard::PutNumber("GyroXComplementary", x_complementary_angle);
  frc::SmartDashboard::PutNumber("GyroYComplementary", y_complementary_angle);
  
  leftLeadEncoderValue = leftLeadEncoder.GetPosition();
  rightLeadEncoderValue = rightLeadEncoder.GetPosition();
  // frc::SmartDashboard::PutNumber("LeftLeadEncoder", leftLeadEncoderValue);
  // frc::SmartDashboard::PutNumber("RightLeadEncoder", rightLeadEncoderValue);
if (!payload_dump_done) {
  
  if (timer0.Get() > 1_s) { // if 3 seconds have passed
    payload_dump_done=1;
    leftLeadEncoder.SetPosition(0); // Reset the encoder statt position
    rightLeadEncoder.SetPosition(0);
  } 
  else { // If 1 seconds have not passed then go forward
    robotDrive.ArcadeDrive(0.5, 0); // 50% throttle
  }
   
}

  if (m_autoSelected == kAutoNameCenter) {
  
    // This program is executed if the robot starts off in the middle position.
    // After dumping the initial payload we want to go into reverse until we detect an angle
    // Once we detect an angle we want to keep going until we are at 0 degrees. 
   frc::SmartDashboard::PutNumber("Payload_dump_done", payload_dump_done);
    leftLeadEncoderValue = leftLeadEncoder.GetPosition();
    rightLeadEncoderValue = rightLeadEncoder.GetPosition();
    // if (payload_dump_done && (abs(leftLeadEncoderValue) > 110) && !outside_of_home) {
    // outside_of_home = 1;
    // }
    if (payload_dump_done) {
      frc::SmartDashboard::PutNumber("initial_tilt_detected", initial_tilt_detected);
      frc::SmartDashboard::PutNumber("damping_factor", damping_factor);
      x_complementary_angle =  double(rioGyro.GetXComplementaryAngle());
      if (abs(y_complementary_angle) > 8) {initial_tilt_detected = 1;}
      // if (!outside_of_home) {robotDrive.ArcadeDrive(-0.5,0);} // Back up until we are on the ramp
      if (!initial_tilt_detected) {robotDrive.ArcadeDrive(-0.5,0);}
      else {
        // If there is a direction change then slowly damp the throttle
        if (((last_angle <= 0) && (y_complementary_angle > 2)) || ((last_angle >= 0) && (y_complementary_angle < -2))) direction_change_detected = 1;
        else direction_change_detected = 0;

        if (direction_change_detected) {  
          damping_factor = damping_factor - 0.2;
          direction_change_detected = 0;
          if (damping_factor < 0) damping_factor = 0;
        } 
        if (y_complementary_angle < -6) pid_out = -0.3 * damping_factor; // back up
        else if (y_complementary_angle > 6) pid_out = 0.30 * damping_factor; // go forward
        else pid_out = 0; // if less than 2 degrees stay put
        frc::SmartDashboard::PutNumber("pid_out", pid_out);
        robotDrive.ArcadeDrive(pid_out,0);}
        last_pid_out = pid_out;
        last_angle = y_complementary_angle;
      }
    }
  else {
    // If robot is not in the middle then drop the payload to score, back up
    // then turn 180 degrees and stop.  At that point we wait until autonomous 
    // period is over.
    
    if (!drive_done && payload_dump_done) {
      leftLeadEncoderValue = leftLeadEncoder.GetPosition();
      rightLeadEncoderValue = rightLeadEncoder.GetPosition();
      pid_out = pid_drive.Calculate(abs(leftLeadEncoderValue), 110);
      if (pid_out > 0.5) pid_out = 0.5; // Clamp the max throttle value
      robotDrive.ArcadeDrive(-pid_out, 0); // We want to go in reverse!
      if (abs(leftLeadEncoderValue) >= 108) drive_done = 1;
    }
    else if (!rotate_180_done && payload_dump_done) {
    pid_out = pid_rotation.Calculate(robot_angle,180); 
    if (pid_out > 0.5) pid_out = 0.5;
    robotDrive.ArcadeDrive(0, pid_out);
    if (abs(robot_angle) >= 170) {rotate_180_done = 1;}
    }
    else if (payload_dump_done) {robotDrive.ArcadeDrive(0,0);}
  }
   
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("GyroYaw", robot_angle);


  // Elevator Telescope Control
  //if (elevatorExtendLimitSwitch0.Get() && (elevatorController.GetRightY() > 0)) leadTelescopeElevatorMotor.Set(0);
  //else if (elevatorRetractLimitSwitch0.Get() && (elevatorController.GetRightY() < 0)) leadTelescopeElevatorMotor.Set(0);
  if ((elevatorController.GetRightY() < xbox_drift_threshold) && (elevatorController.GetRightY() > -xbox_drift_threshold)) leadTelescopeElevatorMotor.Set(0);
  else if((elevatorController.GetRightY() > 0.3)) leadTelescopeElevatorMotor.Set(-0.3);
  else if((elevatorController.GetRightY() < -0.3)) leadTelescopeElevatorMotor.Set(0.3);
  else leadTelescopeElevatorMotor.Set(-(elevatorController.GetRightY()));

  
  // Elevator Tilt Control
  // if (elevatorUprightLimitSwitch0.Get() && (elevatorController.GetLeftY() > 0)) leadTelescopeElevatorMotor.Set(0);
  
  if ((elevatorController.GetLeftY() < xbox_drift_threshold) && (elevatorController.GetLeftY() > -drift_tolerance)) tiltElevatorMotor.Set(0);
  else tiltElevatorMotor.Set(elevatorController.GetLeftY());
  

  // Claw Pneumatics.  Assuming Forward is closed and reverse is open
  if (elevatorController.GetLeftBumperPressed()) {
      leftSolenoid.Set(0);
      rightSolenoid.Set(0);
  } 
  else if (elevatorController.GetRightBumperPressed()) {
      leftSolenoid.Set(1);
      rightSolenoid.Set(1);
  } 
/**/
    leftSolenoid0.Set(frc::Solenoid::kReverse);
    rightSolenoid0.Set(frc::Solenoid::kReverse);

  } else  if (elevatorController.GetRightBumperPressed()) {
    leftSolenoid0.Set(frc::Solenoid::kForward);
    rightSolenoid0.Set(frc::Solenoid::kForward);
  }
*/
  // Drive Control
  TurboEnabled = driveStick.GetRawAxis(3);
  if (TurboEnabled < 0) {robotDrive.ArcadeDrive(-driveStick.GetY(), -driveStick.GetX());}
  else { robotDrive.ArcadeDrive(-driveStick.GetY() * SpeedThrottlePercentage, -driveStick.GetX() * SpeedThrottlePercentage);}

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
