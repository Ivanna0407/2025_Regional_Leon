// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Sub_Swerve;

public class Cmd_Specific_State extends Command {
  /** Creates a new Cmd_Specific_State. */
  private final Sub_Swerve sub_Swerve;
  private final ChassisSpeeds speeds;
  private final Timer timer = new Timer();
  private final double seconds;


  public Cmd_Specific_State(Sub_Swerve sub_Swerve, ChassisSpeeds speeds, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub_Swerve=sub_Swerve;
    this.speeds = speeds;
    this.seconds = seconds;
    addRequirements(sub_Swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Inicio");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("1");
    sub_Swerve.driveRobotRelative(speeds);
    //System.out.println("2");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub_Swerve.driveRobotRelative(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("Fin");
    if (timer.get()>2.5){
      System.out.println("Fin");
      return true;
    }
    return false;
  }
}