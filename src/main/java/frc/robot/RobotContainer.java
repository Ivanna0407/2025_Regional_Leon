// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Cmd_Alga_Teleop;
import frc.robot.commands.Cmd_Wrist_PID;
import frc.robot.commands.Cmd_Elevador_PID;
import frc.robot.commands.Cmd_Elevador_Teleop;
import frc.robot.commands.Cmd_Move_Swerve;
import frc.robot.commands.Cmd_Specific_State;
import frc.robot.commands.Cmd_Take_Coral_Auto;
import frc.robot.commands.Cmd_Vision;
import frc.robot.commands.Cmd_Wait;
import frc.robot.commands.Cmd_giro;
import frc.robot.commands.Cmd_resetheading;
import frc.robot.subsystems.Sub_Algas;
import frc.robot.subsystems.Sub_Elevador;
import frc.robot.subsystems.Sub_Swerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;





public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Sub_Swerve swerve = new  Sub_Swerve();
  private final Sub_Elevador Elevador = new Sub_Elevador();
  private final Sub_Algas Alga= new Sub_Algas();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController subdrive = new CommandXboxController(1);
  private final CommandXboxController joydrive = new CommandXboxController(0);
  
  private final SendableChooser <Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerve.setDefaultCommand(new Cmd_Move_Swerve(swerve, () -> joydrive.getLeftX(), () -> joydrive.getLeftY(), ()-> joydrive.getRightX(),() -> joydrive.rightBumper().getAsBoolean(),() -> joydrive.y().getAsBoolean()));
    Elevador.setDefaultCommand(new Cmd_Elevador_Teleop(Elevador, () -> subdrive.getRightTriggerAxis(), () -> subdrive.getLeftTriggerAxis(),() ->  subdrive.x().getAsBoolean(), () -> subdrive.b().getAsBoolean(), () -> subdrive.leftBumper().getAsBoolean(), () -> subdrive.rightBumper().getAsBoolean(),() -> subdrive.y().getAsBoolean()));
    Alga.setDefaultCommand(new Cmd_Alga_Teleop(Alga, () -> subdrive.povUp().getAsBoolean(), () -> subdrive.povDown().getAsBoolean(), () -> subdrive.povLeft().getAsBoolean(), () -> subdrive.povRight().getAsBoolean()));
    autoChooser= AutoBuilder.buildAutoChooser("Simple");
    SmartDashboard.putData(autoChooser);
   
    configureBindings();
  }

  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  joydrive.a().whileTrue(new Cmd_giro(swerve, 170*(Math.PI/180)));
  joydrive.start().whileTrue(new Cmd_resetheading(swerve));
  //subdrive.y().whileTrue(new Cmd_Wrist_PID(Elevador, 90));
  subdrive.a().whileTrue(new Cmd_Wrist_PID(Elevador, 0));
  subdrive.leftStick().whileTrue(new Cmd_Wrist_PID(Elevador, -90));
  joydrive.b().whileTrue(new Cmd_Vision(swerve, 0));
  subdrive.y().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(new Cmd_Elevador_PID(Elevador, 1.5),
  new Cmd_Wrist_PID(Elevador, 90)),new Cmd_Take_Coral_Auto(Elevador)));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Cmd_Specific_State(swerve, new ChassisSpeeds(.75,0,0), 2);
  }
}
