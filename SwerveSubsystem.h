// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once



#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <string>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

//Path Planner libraries to use!
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>


//Drivestation librarie(blue and red side)
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc2/command/SubsystemBase.h>

//Phoenix 5
// #include <ctre/phoenix/sensors/Pigeon2.h>

//Phoenix 6
#include <ctre/phoenix6/Pigeon2.hpp>


//Lime Subsystems
#include "subsystems/LimeLightFrontSubsystem.h"
#include "subsystems/LimeLightBackSubsystem.h"
#include "LimeLightHelpers.h"


#include "Constants.h"


#include "SwerveModule.h"
#include <thread> 
#include <iostream>

// #include <frc/NetworkTables/NetworkTableInstance.h>
// #include <frc/NetworkTables/StructArrayPublisher.h>

 #include "networktables/NetworkTableInstance.h"
 #include "networktables/NetworkTable.h"




class SwerveSubsystem : public frc2::SubsystemBase {
 
 public:
  SwerveSubsystem();
  
  inline frc::ChassisSpeeds GetSwerveSpeed()
 {
  return DriveConstants::kDriveKinematics.ToChassisSpeeds({FrontLeft.GetState(),FrontRight.GetState(),RearLeft.GetState(),RearRight.GetState()});
 }

   //Path relative settings
  void driveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds);

  //Pid autonomous parameter
  double kPThetaController = AutoConstants::kPThetaController;
  double KPXaxisController = AutoConstants::kPXController;
  double KPYaxisController = AutoConstants::kPYController;


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //The Mainly method for SwerveDrive in TeleOperated, that transform the joystick double inputs into 
  //a module state (Meters Per Second and Radians Per Second)
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative)
             {
                Drive(xSpeed, ySpeed, rot, fieldRelative, 0_m, 0_m);
             }
  //as above, but allow more specifications
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  

  //Reset the Pigeon Heading
  void ZeroHeading();

  //Reset all the 8 encoders
  void ResetEncoders();

  //Get The Pigeon Heading
  double GetHeading();
 
  //2d NavX Rotation method 
  frc::Rotation2d GetRotation2d();

  //2d Odometry method
  frc::Pose2d GetPose2d();

  //Reset the Odometry for Autonomous
  void ResetOdometry(const frc::Pose2d& Pose);

  //get the pidgeon angle in radians
  double GetRadians();

  //Stop all the 4  modules (8 Motors)
  void StopModules();

  void SetModulesState(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  void UpdateOdometry();

  double GetswerveXPose();

  double GetswerveYPose();

   //Declaring each module as a SwerveModule object  
  SwerveModule FrontLeft;
  SwerveModule RearLeft;
  SwerveModule FrontRight;
  SwerveModule RearRight;



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Declaring the 4 LimeLights as objects from LimeLightSubsystem
  LimeLightFrontSubsystem  limeFront_Subsystem;
  LimeLightBackSubsystem limeBack_Subsystem;
  
  // //The Pigeon phoenix 5 object
  // ctre::phoenix::sensors::WPI_Pigeon2 pigeon{PigeonConstants::PigeonId};

  //The Pigeon phoenix 6 object
  ctre::phoenix6::hardware::Pigeon2 pigeon{PigeonConstants::PigeonId};
 
 //The Pigeon Config Object Phoenix 6
  ctre::phoenix6::configs::Pigeon2Configuration pigeon_Config;

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  // frc::SwerveDriveOdometry<4> Odometry;

  // frc::SwerveDriveOdometry<4> Odometry;
  frc::SwerveDrivePoseEstimator<4> OdometryEstimator;

//   ArmSubsystem arm_Subsystem;

  //for the drive method
  double rotation{0.0};
  double x{0.0};
  double y{0.0};
  double heta{0.0};


};
