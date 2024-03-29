// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <numbers>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
//these numbers are what the motor controllers are assigned
  constexpr int kFrontLeftDriveMotorPort = 13; 
  constexpr int kRearLeftDriveMotorPort = 7; 
  constexpr int kFrontRightDriveMotorPort = 3; 
  constexpr int kRearRightDriveMotorPort = 5; 

  constexpr int kFrontLeftTurningMotorPort = 2; 
  constexpr int kRearLeftTurningMotorPort = 8; 
  constexpr int kFrontRightTurningMotorPort = 4;
  constexpr int kRearRightTurningMotorPort = 6; 

  constexpr int kFrontLeftTurningEncoderNumber = 9; 
  constexpr int kRearLeftTurningEncoderNumber = 12; 
  constexpr int kFrontRightTurningEncoderNumber = 10;
  constexpr int kRearRightTurningEncoderNumber = 11;

  constexpr bool kFrontLeftTurningEncoderReversed = false;
  constexpr bool kRearLeftTurningEncoderReversed = false;
  constexpr bool kFrontRightTurningEncoderReversed = false;
  constexpr bool kRearRightTurningEncoderReversed = false;

  constexpr bool kFrontLeftDriveEncoderReversed = false;
  constexpr bool kRearLeftDriveEncoderReversed = true;
  constexpr bool kFrontRightDriveEncoderReversed = false;
  constexpr bool kRearRightDriveEncoderReversed = true;

  constexpr int kFrontLeftDriveCPR = 42;
  constexpr int kRearLeftDriveCPR = 42;
  constexpr int kFrontRightDriveCPR = 42;
  constexpr int kRearRightDriveCPR = 42;

  constexpr int kFrontLeftTurningCPR = 1;
  constexpr int kRearLeftTurningCPR = 1;
  constexpr int kFrontRightTurningCPR = 1;
  constexpr int kRearRightTurningCPR = 1;

  constexpr rev::SparkMaxRelativeEncoder::Type m_EncoderType = rev::SparkMaxRelativeEncoder::Type::kHallSensor;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically for *your* robot's drive. The RobotPy Characterization
  // Toolsuite provides a convenient tool for obtaining these values for your
  // robot.
  constexpr auto ks = 1_V;
  constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

  // Example value only - as above, this must be tuned for your drive!
  constexpr double kPFrontLeftVel = 0.5;
  constexpr double kPRearLeftVel = 0.5;
  constexpr double kPFrontRightVel = 0.5;
  constexpr double kPRearRightVel = 0.5;
}  

namespace ModuleConstants {
  constexpr double wheelOffset = 0;
  constexpr double gearRatio = 8.14; //we measured 8.91
  constexpr double kEncoderCPR = 1;
  constexpr double kWheelDiameterMeters = 0.0977; // 0.0762

  // Assumes the encoders are directly mounted on the wheel shafts
  constexpr double kDriveEncoderDistancePerPulse =(kWheelDiameterMeters * std::numbers::pi) / (kEncoderCPR) / gearRatio;

  // Assumes the encoders are directly mounted on the wheel shafts
  constexpr double kTurningEncoderDistancePerPulse =(std::numbers::pi * 2) / (kEncoderCPR);

  constexpr double kPModuleTurningController = 0.6;//1.0; // 0.5 //0.003 // TODO: reduce this by a factor of half outside comp
  constexpr double kPModuleDriveController = 0.1; // 0.1
  // TODO Lower Value of P to 0.0001,  Change Value of p Till its the Highest Without Osilation, 
  constexpr double kFFModuleDriveController = 0.259375;
}

namespace AutoConstants {
  constexpr auto kMaxSpeed = 4.2_mps;
  constexpr auto kMaxAcceleration = 0.5_mps_sq;
  constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
  constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

  constexpr double kPXController = 0.1;
  constexpr double kPYController = 0;
  constexpr double kPThetaController = 0;

  extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  

namespace DebugConstants {
  constexpr bool debug = false; //change this to true to debug and put most things to the smartdashboard
}  