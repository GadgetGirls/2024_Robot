// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Xboxcontroller.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <cameraserver/CameraServer.h>
#include <rev/CANSparkMax.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc2/command/PIDCommand.h>
#include <Math.h>
#include <frc/Timer.h>
#include <units/time.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
 
  // Smart Dashboard Chooser
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Side starting position autonomous program";
  const std::string kAutoNameCenter = "Center starting position autonomous program";
  std::string m_autoSelected;

  // Drive Controllers, montors and enoders
  rev::CANSparkMax leftLeadMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftFollowMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightLeadMotor{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightFollowMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  frc::DifferentialDrive robotDrive{leftLeadMotor, rightLeadMotor};
  rev::SparkMaxRelativeEncoder leftLeadEncoder = leftLeadMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder rightLeadEncoder = rightLeadMotor.GetEncoder();

  // Elevator Controllers
  rev::CANSparkMax tiltElevatorMotor{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leadTelescopeElevatorMotor{6, rev::CANSparkMax::MotorType::kBrushless}; // Should be the left motor
  rev::CANSparkMax followTelescopeElevatorMotor{7, rev::CANSparkMax::MotorType::kBrushless}; // Should be the right motor

  // Pneumatics
  //frc::DoubleSolenoid leftDoubleSolenoid0{frc::PneumaticsModuleType::CTREPCM, 1,2};
  //frc::DoubleSolenoid rightDoubleSolenoid0{frc::PneumaticsModuleType::CTREPCM, 3,4};
  frc::Solenoid leftSolenoid0{frc::PneumaticsModuleType::CTREPCM, 0};
  frc::Solenoid leftSolenoid0{frc::PneumaticsModuleType::CTREPCM, 1};
  static constexpr int kSolenoidButton = 1;
  static constexpr int kDoubleSolenoidForward = 2;
  static constexpr int kDoubleSolenoidReverse = 3;


  // Input Devices
  frc::XboxController elevatorController{1};
  frc::Joystick driveStick{0};

  // Sensors
  frc::ADIS16470_IMU rioGyro;
 // frc::DigitalInput elevatorExtendLimitSwitch0 {0};
 // frc::DigitalInput elevatorRetractLimitSwitch0 {1};
 // frc::DigitalInput elevatorUprightLimitSwitch0 {2};
  
  // PID
  frc2::PIDController pid_rotation{0.01,0.00125,0};
  frc2::PIDController pid_drive{0.01, 0.00125, 0};
  
  // Timer
  frc::Timer timer0;
  // Misc
  int TurboEnabled; // Mapped to the flip switch on the joystick.  Down means Turbo is off.
  double SpeedThrottlePercentage; // When turbo mode is disabled, this is the percentage we reduce the robots speed by
  double robot_angle;
  double xbox_drift_threshold;
  int rotate_180_done;
  int drive_done;
  double inches_per_rotation;
  double pid_out, last_pid_out;
  double num_revolutions;
  double leftLeadEncoderValue, rightLeadEncoderValue;
  int payload_dump_done;
  int initial_tilt_detected;
  int direction_change_detected;
  double x_complementary_angle;
  double y_complementary_angle;
  double last_angle;
  double damping_factor;
  double drift_tolerance;
  double angle_correction;
  // Added By DJ
  int count_up_forward; 
  int count_up_backward; 
  int outside_of_home;


};
