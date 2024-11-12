// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include<frc/Joystick.h>
#include "Constants.h"

#include "subsystems/SwerveSubsystem.h"

#include "subsystems/LimeLightFrontSubsystem.h"

#include "subsystems/LimeLightBackSubsystem.h"

#include "subsystems/ControlSubsystem.h"

#include "subsystems/TopSubsystems/ShooterSubsystem.h"

#include "subsystems/TopSubsystems/ElevatorSubsystem.h"

#include "subsystems/TopSubsystems/FlipSubsystem.h"

#include "subsystems/TopSubsystems/GiraffeSubsystem.h"

//Ultrasonic Subsystem
#include "subsystems/UltrasonicSubsystem.h"

#include "frc2/command/Command.h"

#include "frc2/command/button/JoystickButton.h"

#include <frc/XboxController.h>

//SwerveCommand Library
#include "commands/TeleopCommands/SwerveCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <pathplanner/lib/auto/NamedCommands.h>

//Subsystems commands

#include "commands/TeleopCommands/GiraffeCommand.h"

#include "commands/TeleopCommands/ShooterCommand.h"

#include "commands/TeleopCommands/IntakeCommand.h"

#include "commands/TeleopCommands/EspecificAngleGiraffeCommand.h"

#include "commands/TeleopCommands/IntakeAmpCommand.h"

#include "commands/TeleopCommands/ShooterAmpCommand.h"

#include "commands/TeleopCommands/ElevatorCommand.h"

#include "commands/TeleopCommands/IntakeLaunchCommand.h"



/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();


    void AutonomousInit();
    void TeleopInit();
    void RobotInit();
    void SelectAuto();

 private:

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  // Xbox Top Controller
  frc2::CommandXboxController m_TopController{OperatorConstants::kTopControllerPort};

  // The robot's subsystems are defined here...

  //XboxSubsystem Controller
  ControlSubsystem Control;  

  //SwerveSubsystem Object  
  SwerveSubsystem swerve_Subsystem;

  //LimeLight-Front Subsystem
  LimeLightFrontSubsystem limeLightFront_Subsystem;

  //LimeLight-Back Subsystem
  LimeLightBackSubsystem limeLightBack_Subsystem;

  //IntakeSubsystem
  IntakeSubsystem intake_Subsystem;

  //Shooter Subsystem
  ShooterSubsystem shooter_Subsystem;

  //Elevator Subsystem
  ElevatorSubsystem elevator_Subsystem;

  //Ultrasonic Subsystem
  //UltrasonicSubsystem ultrasonic_Subsystem;

  //Flip Subsystem
  //FlipSubsystem flip_Subsystem;

  //Giraffe Subsystem
  GiraffeSubsystem giraffe_Subsystem;  

  // Default Swerve Command
  SwerveCommand swerve_Command{
    &swerve_Subsystem,
    &Control,
    // &shooter_Subsystem,
    //&flip_Subsystem,
    &giraffe_Subsystem,
    &elevator_Subsystem,
    //&ultrasonic_Subsystem,
    &limeLightFront_Subsystem,
    &limeLightBack_Subsystem
  };

  //TeleopShooterCommand Shooter_Command{&shooter_Subsystem, &intake_Subsystem};

  // AutoDistCommand Dist_ShortCommand{&swerve_Subsystem, 0.5, 0};
  // AutoDistCommand Dist_LongForwardCommand{&swerve_Subsystem, 2, 0};
  // AutoDistCommand Dist_LongBackwardCommand{&swerve_Subsystem, -2, 0};
  // AutoDistCommand Dist_ShortLeftSideCommand{&swerve_Subsystem, 0, -0.5};
  // AutoDistCommand Dist_ShortRightSideCommand{&swerve_Subsystem, 0, 0.5};

  //  frc::ProfiledPIDController<units::radians> thetaController {
  //  swerve_Subsystem.kPThetaController, 0,0,
  //  AutoConstants::kThetaControllerConstraints};


  //0 = 2 notes center with lime
  //1 = 2 notes center with timer
  //2 = 3 notes center with lime
  //3 = 2 notes left with lime
  //4 = 2 notes right with lime
  //5 = 3 notes right blue with lime
  //6 = 3 notes left with lime
  int autoValue = 2;

  //Shooter speaker speeds to apply in shuffle
  double LaunchShooterSpeedSpeaker = 90;
  double CatchShooterSpeedSpeaker = 0.15;

  //Shooter amp speeds
  double LaunchShooterAmp = 17;

  //Intake speeds to aplly in shuffle  
  double rightCatchIntakeSpeed  = -0.5; //
  double leftCatchIntakeSpeed = 0.25;  //

  double rightSpitIntakeSpeed  = 0.5; //
  double leftSpitIntakeSpeed  = -0.25; //

  //The default  Elevator Pots when Start or Back button is pressed
  double potUpElevator = 0.1;
  double potDownElevator = -0.1;

  void ConfigureBindings();
};
