// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,     
                           int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed) : driveMotor(driveMotorId),
                                                                                                                turningMotor(turningMotorId, rev::CANSparkLowLevel::MotorType::kBrushless),
                                                                                                                absoluteEncoder(absoluteEncoderId),
                                                                                                                turningEncoder(turningMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42)),
                                                                                                                absoluteEncoderOffsetRad(absoluteEncoderOffset)
{
 //Default Swerve Motors config

  turningPidController.EnableContinuousInput(0, 2*std::numbers::pi);
  // toApplyCancoder.MagnetSensor.AbsoluteSensorRange = 0;

//Kraken Drive Motor Configs
  driveMotor.SetInverted(driveMotorReversed);
  driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  //Neo  Turning Motor Configs
  turningMotor.SetInverted(turningMotorReversed);
  turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  turningMotor.BurnFlash();
  turningMotor.SetSmartCurrentLimit(0, 38);

  //  driveEncoder.SetPositionConversionFactor(ConstantsMod::kDriveEncoderRot2Meter);
  //  driveEncoder.SetVelocityConversionFactor(ConstantsMod::kDriveEncoderRPM2MeterPerSec);
  //  driveMotor.BurnFlash();


  ResetEncoders();
  ConfigureKraken(driveMotor.GetConfigurator());
  // absoluteEncoder.GetConfigurator().Apply(toApplyCancoder);
}

void SwerveModule::ConfigureKraken(ctre::phoenix6::configs::TalonFXConfigurator &cfg){
  ctre::phoenix6::configs::TalonFXConfiguration toApply{};

  m_currentLimits.SupplyCurrentLimit = 40;

  toApply.MotorOutput.NeutralMode = 1;

  ctre::phoenix6::configs::MotionMagicConfigs &MotionDriveConfig = toApply.MotionMagic;

  MotionDriveConfig.MotionMagicCruiseVelocity = 100;
  MotionDriveConfig.MotionMagicAcceleration = 200;
  MotionDriveConfig.MotionMagicJerk = 50;
  MotionDriveConfig.MotionMagicExpo_kA = 0.0001;

  // ctre::phoenix6::configs::MotionMagicConfigs &mm_Drive = toApply.MotionMagic;

  // mm_Drive.MotionMagicCruiseVelocity = 90;
  // mm_Drive.MotionMagicAcceleration = 250;
  // mm_Drive.MotionMagicJerk = 200;

  ctre::phoenix6::configs::Slot0Configs &slot0_Drive = toApply.Slot0;


  slot0_Drive.kP = 0.10;
  slot0_Drive.kI = 0.0001;
  slot0_Drive.kD = 0.00001;
  slot0_Drive.kV = 0.12;
  slot0_Drive.kS = 0.05; // Approximately 0.25V to get the mechanism moving
  
  toApply.CurrentLimits = m_currentLimits;

  cfg.Apply(toApply);
}

double SwerveModule::TurningsToMeters(double rotations){
auto circumference = ConstantsMod::kWheelRadiusMeters * 2 * std::numbers::pi;
auto gearedRotations = rotations / 6.12;
return gearedRotations * circumference;
}

double SwerveModule::GetMotorDriveTemp()
{
  return driveMotor.GetDeviceTemp().GetValueAsDouble();
}

double SwerveModule::GetMotorTurningTemp()
{
  return turningMotor.GetMotorTemperature();
}

double SwerveModule::GetMotorDriveCurrent()
{
  return driveMotor.GetSupplyCurrent().GetValueAsDouble();
}

double SwerveModule::GetMotorTurnigCurrent()
{
  return turningMotor.GetOutputCurrent();
}

double SwerveModule::GetDriveVelocity()
{
  return TurningsToMeters(driveMotor.GetVelocity().GetValueAsDouble());
  // return driveMotor.GetVelocity().GetValueAsDouble();
}

double SwerveModule::getTurningVelocity()
{
  return turningEncoder.GetVelocity();
}

double SwerveModule::GetDrivePosition()
{
  return TurningsToMeters(-driveMotor.GetPosition().GetValueAsDouble());
  // return driveMotor.GetPosition().GetValueAsDouble();
}

double SwerveModule::GetTurningPosition()
{
  return turningEncoder.GetPosition();
}

double SwerveModule::GetAbsoluteEncoderRad()
{

  
  double angle = absoluteEncoder.GetAbsolutePosition().GetValueAsDouble()*(2*std::numbers::pi) - absoluteEncoderOffsetRad;
  // double angle = absoluteEncoder.GetAbsolutePosition().GetValueAsDouble()*(2*std::numbers::pi);
  
  if (angle < 0)
  {
    angle += 2*std::numbers::pi;
  }

  if (angle > 2*std::numbers::pi){
    angle -= 2*std::numbers::pi;
  }
    
  return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  
  
}

double SwerveModule::GetAbsoluteModulePostion()
{
  // return absoluteEncoder.GetAbsolutePosition().GetValueAsDouble();
  return absoluteEncoder.GetAbsolutePosition().GetValueAsDouble();
}

void SwerveModule::ResetEncoders()
{
  driveMotor.SetPosition(0_tr);
  turningEncoder.SetPosition(GetAbsoluteEncoderRad());
}

frc::SwerveModuleState SwerveModule::GetState()
{
  //  return {units::meters_per_second_t{GetDriveVelocity()},
  //          frc::Rotation2d(units::radian_t{GetTurningPosition()})};

      return {units::meters_per_second_t{GetDriveVelocity()},
            frc::Rotation2d(units::radian_t(GetAbsoluteEncoderRad()))};

}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& State)
{
  if (units::math::abs(State.speed) < units::meters_per_second_t{0.002})
  {
    Stop();
    return;
  }

  auto state = frc::SwerveModuleState::Optimize(State, GetState().angle);

    units::meters_per_second_t speed_motor = 0_mps;
    if (state.speed > DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond)
    {
      speed_motor =   DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond;
    }
    else
    {
      speed_motor = state.speed;
    }
      double speedReturn = static_cast<double>(speed_motor)*100;
    // driveMotor.SetControl(MotionDrive.WithVelocity(static_cast<units::turns_per_second_t>(speedReturn)).WithSlot(0));
    driveMotor.Set(static_cast<double>(speed_motor));
    turningMotor.Set(turningPidController.Calculate(GetAbsoluteEncoderRad(), static_cast<double>(state.angle.Radians())));

    frc::SmartDashboard::PutNumber("ROBOT drive speed", static_cast<double>(speed_motor));
    
}

  frc::SwerveModulePosition SwerveModule::GetPosition(){

    return {units::meter_t{GetDrivePosition()},

    frc::Rotation2d{units::degree_t{GetTurningPositionInDegree()}}};

  }

void SwerveModule::Stop()
{
  driveMotor.Set(0);
  turningMotor.Set(0);
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
// frc::SmartDashboard::PutNumber("robot expression test", TurningsToMeters(driveMotor.GetPosition().GetValueAsDouble()));

     

}

double SwerveModule::GetTurningPositionInDegree(){
  double returnDegrees = GetAbsoluteEncoderRad() * 180 / std::numbers::pi;
  return returnDegrees;
}