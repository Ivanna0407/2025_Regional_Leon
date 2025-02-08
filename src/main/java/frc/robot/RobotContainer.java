// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Cmd_Move_Brazo;
import frc.robot.commands.Cmd_Move_Swerve;
import frc.robot.subsystems.Sub_Brazo;
import frc.robot.subsystems.Sub_Swerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Sub_Swerve swerve = new  Sub_Swerve();
  private final Sub_Brazo brazo = new Sub_Brazo();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController joydrive= new XboxController(0);
  private final XboxController subdrive = new XboxController(1);
  
 // private final SendableChooser <Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerve.setDefaultCommand(new Cmd_Move_Swerve(swerve,() -> joydrive.getLeftX(),() -> joydrive.getLeftY(), () -> joydrive.getRightX(),() -> joydrive.getRightBumperButton(),3,() -> joydrive.getYButton()));
    brazo.setDefaultCommand(new Cmd_Move_Brazo(brazo, () -> joydrive.getLeftTriggerAxis(), () -> joydrive.getRightTriggerAxis(), () -> subdrive.getLeftX(),() -> joydrive.getAButton(),() -> joydrive.getBButton()));
    //autoChooser= AutoBuilder.buildAutoChooser();
    //autoChooser= AutoBuilder.buildAutoChooser("New_Auto");
   // SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Simple");
    
  }
}
