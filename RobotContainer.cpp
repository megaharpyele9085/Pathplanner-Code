// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/TeleopCommands/SwerveCommand.h"

#include "functional"
#include "algorithm"

#include <frc2/command/InstantCommand.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <frc2/command/SwerveControllerCommand.h>

RobotContainer::RobotContainer()
{

   swerve_Subsystem.SetDefaultCommand(std::move(swerve_Command));

  // Just Descomment this Command when the Giraffe angle defined and working.
  // giraffe_Subsystem.SetDefaultCommand(std::move(autoSpeakerAngleCommand));]

  pathplanner::NamedCommands::registerCommand("AutoShooter", ShooterCommand(shooter_Subsystem, false).ToPtr());
  pathplanner::NamedCommands::registerCommand("AutoGiraffe", GiraffeCommand(limeLightBack_Subsystem, giraffe_Subsystem).ToPtr());
  pathplanner::NamedCommands::registerCommand("AutoIntake", IntakeLaunchCommand(intake_Subsystem, shooter_Subsystem).ToPtr());
  pathplanner::NamedCommands::registerCommand("AutoSpecificGiraffe", EspecificAngleGiraffeCommand(giraffe_Subsystem, 140).ToPtr());
  pathplanner::NamedCommands::registerCommand("AutoIntakeCatchCommand", IntakeCommand(intake_Subsystem, shooter_Subsystem, giraffe_Subsystem).ToPtr());

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{

  //--------------------------------SHOOTER------------------------------------------------------

  // m_TopController.RightTrigger().OnTrue(frc2::InstantCommand([&]() -> void
  //                                                            { shooter_Subsystem.CathShooterCommand(CatchShooterSpeedSpeaker); },
  //                                                            {&shooter_Subsystem})
  //                                           .ToPtr());

    m_TopController.RightTrigger().OnTrue(
    frc2::SequentialCommandGroup(
    // GiraffeCommand(limeLightBack_Subsystem, giraffe_Subsystem), 
    ShooterCommand(shooter_Subsystem, false),
    IntakeLaunchCommand(intake_Subsystem, shooter_Subsystem)).ToPtr());
    // IntakeCommand(intake_Subsystem, shooter_Subsystem, giraffe_Subsystem, false)).ToPtr());

  m_TopController.RightTrigger().OnFalse(frc2::InstantCommand([&]() -> void
                                                              { shooter_Subsystem.StopShooterCommand();
                                                                // giraffe_Subsystem.SetDesiredAngle(GiraffeConstants::DefaultAngle);
                                                                intake_Subsystem.StopIntakeCommand();},
                                                              // {&shooter_Subsystem, &giraffe_Subsystem, &intake_Subsystem})
                                                              {&shooter_Subsystem, &intake_Subsystem})
                                             .ToPtr());

  //-------------------------------DEFINING_AMP SHOOTER POTS------------------------------------------
  m_TopController.A().OnTrue(
    frc2::SequentialCommandGroup(
    ShooterAmpCommand(shooter_Subsystem),
    IntakeAmpCommand(intake_Subsystem, shooter_Subsystem)).ToPtr());
  // m_TopController.A().OnTrue(frc2::SequentialCommandGroup(ShooterCommand(shooter_Subsystem)).ToPtr());
    // m_TopController.A().OnTrue(ShooterCommand(shooter_Subsystem).ToPtr());

  m_TopController.A().OnFalse(frc2::InstantCommand([&]() -> void
                                                   { shooter_Subsystem.StopShooterCommand();
                                                     intake_Subsystem.StopIntakeCommand(); },
                                                   {&shooter_Subsystem, &intake_Subsystem})
                                  .ToPtr());
  //-------------------------------------INTAKE--------------------------------------------------------------

    m_TopController.LeftTrigger().OnTrue(IntakeCommand(intake_Subsystem, shooter_Subsystem, giraffe_Subsystem).ToPtr());

  m_TopController.LeftTrigger().OnFalse(frc2::InstantCommand([&]() -> void
                                                             { intake_Subsystem.StopIntakeCommand();},
                                                             {&intake_Subsystem})
                                            .ToPtr());
  
  //--------------------------------ELEVATOR_PID/TESTE-------------------------------------------------------------

  //**********PID_ELEVATOR***************//

  // OBS all methods of elevator subsystem is alredy inverted on ElevatorSubsystem.cpp
     m_TopController.Back().OnTrue(
    frc2::SequentialCommandGroup(ElevatorCommand(elevator_Subsystem)).ToPtr());

  // Take care with this OnFalse Method Call.Just use after the Up to Chain Pid method work nicely,
  // and you discomment this OneFalse

  m_TopController.Back().OnFalse(frc2::InstantCommand([&]()-> void
                                                {elevator_Subsystem.StopElevator();},
                                                {&elevator_Subsystem}
                                                )
                                      .ToPtr());

  // m_TopController.Back().OnTrue(frc2::InstantCommand([&]()-> void
  //                                               {elevator_Subsystem.DownArmToRobot(potDownElevator);},
  //                                               {&elevator_Subsystem}
  //                                               )
  //                                     .ToPtr());

  // m_TopController.Back().OnFalse(frc2::InstantCommand([&]()-> void
  //                                               {elevator_Subsystem.StopElevator();},
  //                                               {&elevator_Subsystem}
  //                                               )
  //                                     .ToPtr());

  //-------------------------------Amp Velocity Button--------------------------------------------------------------
    m_TopController.Y().OnTrue(
    frc2::SequentialCommandGroup(
    ShooterCommand(shooter_Subsystem, true)).ToPtr());
    // IntakeCommand(intake_Subsystem, shooter_Subsystem, giraffe_Subsystem, true)).ToPtr());

//  m_TopController.Y().OnFalse(frc2::InstantCommand([&]() -> void
//                                                             {shooter_Subsystem.StopShooterCommand();
//                                                              intake_Subsystem.StopIntakeCommand();},
//                                                             {&shooter_Subsystem,
//                                                              &intake_Subsystem})
//                                            .ToPtr());                                         

}

void RobotContainer::AutonomousInit()
{
  // swerve_Command.Cancel();
  
  swerve_Subsystem.ResetEncoders();
  // swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));
  
  swerve_Subsystem.ZeroHeading();
}

void RobotContainer::TeleopInit()
{

  // swerve_Command.Schedule();
  swerve_Subsystem.ResetEncoders();
  // swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));

  elevator_Subsystem.ResetElevatorEncoders();
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
  return pathplanner::PathPlannerAuto("lets go please").ToPtr();
}

void RobotContainer::SelectAuto()
{
  frc::SmartDashboard::SetDefaultNumber("Autonomous select", autoValue);
  autoValue = frc::SmartDashboard::GetNumber("Autonomous select", autoValue);

  // Elevator Up Speeds
  frc::SmartDashboard::SetDefaultNumber("UpElevatorPot", potUpElevator);
  potUpElevator = frc::SmartDashboard::GetNumber("UpElevatorPot", potUpElevator);

  // Elevator Down Speeds
  frc::SmartDashboard::SetDefaultNumber("DownElevatorPot", potDownElevator);
  potDownElevator = frc::SmartDashboard::GetNumber("DownElevatorPot", potDownElevator);

  frc::SmartDashboard::PutData("Intake Subsystem", &intake_Subsystem);
}

void RobotContainer::RobotInit(){
  swerve_Subsystem.ResetOdometry(limeLightBack_Subsystem.GetBotPose("limelight-backl"));
}
