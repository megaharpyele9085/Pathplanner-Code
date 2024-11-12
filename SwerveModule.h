// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

//Constants
#include "Constants.h"

//Rev Driver Library
#include "rev/CanSparkMax.h"
#include "rev/CANSparkFlex.h"

//CanCoder Library
// #include "ctre/Phoenix.h"

//Phoenix 6 library 
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"

//Units
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <numbers>

//Wplib Default
#include <wpi/DataLog.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

//Swerve Library(State and Position)
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>

//PID
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>

//Shuffleboard
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule : public frc2::SubsystemBase {
 public:
 /**
   * Default Swerve Module Paramters
   *
   * @param driveMotorId            Id do motor de deslocamento.
   * @param turningMotorId          Id do motor de rotação.
   * @param driveMotorReversed      Parâmetro para reversão do motor de deslocamento.
   * @param turningMotorReversed    Parâmetro para reversão do motor de rotação.
   * @param absoluteEncoderId       Id do encoder absoluto.
   * @param absoluteEncoderOffset   Diferença entre os valores dos encoders absoluto e relativo ao iniciar.
   * @param absoluteEncoderReversed Parâmetro para cosiderar o encoder absoluto como reverso.
   */
  SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
  int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void   Periodic() override;   

  double GetMotorDriveTemp();

  double GetMotorTurningTemp();

  double GetMotorDriveCurrent();

  double GetMotorTurnigCurrent();

  double GetTurningPositionInDegree();

  double GetDrivePosition();

  double GetTurningPosition();

  double GetDriveVelocity();

  double TurningsToMeters(double rotations);

  double getTurningVelocity();

  double GetAbsoluteEncoderRad();

  double GetAbsoluteModulePostion();

  void ResetEncoders();

  frc::SwerveModuleState GetState();

  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModulePosition GetPosition();

  void ConfigureKraken(ctre::phoenix6::configs::TalonFXConfigurator &cfgs);

  ctre::phoenix6::configs::CurrentLimitsConfigs m_currentLimits{};

  // ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_VelocityFoc{0_tps};

  // ctre::phoenix6::configs::TalonFXConfiguration driveMotorConfiguration{};

  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //All the objects here is the system base for one module work nice
  //Motors,Relative Encoders, Absolute Encoders , Turning PID and OffSetAngle


    //DriveRev   Motor
    // rev::CANSparkFlex driveMotor;

    //Drive Kraken Motor Motor
    ctre::phoenix6::hardware::TalonFX driveMotor;

    ctre::phoenix6::controls::MotionMagicVelocityVoltage MotionDrive{0_tps};

    //TurningRev Motor
    rev::CANSparkMax turningMotor;

    //DriveRev RelativeEncoder
    // rev::SparkRelativeEncoder driveEncoder;

    //TurningRev RelativeEncoder 
    rev::SparkRelativeEncoder turningEncoder;

    // //Absolute CTRE Encoder  Phoenix 6
    ctre::phoenix6::hardware::CANcoder absoluteEncoder;

    //Turning PID for Teleoperated    
    frc::PIDController turningPidController{
      PIDModConstants::kPTurning, PIDModConstants::kITurning, PIDModConstants::kDTurning};

    //Reverse Condition for CTRE Absolute Encoder
    bool absoluteEncoderReversed;

    //The OffSet Angle for each module
    double absoluteEncoderOffsetRad;

    //Auto Constants...
    static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
    static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

      
};

