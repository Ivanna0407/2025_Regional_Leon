// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Algas;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Alga_PID extends Command {
  /** Creates a new Cmd_Alga_PID. */
  private final Sub_Algas Alga;
  private final double setpoint;
  double kp, error;
  public Cmd_Alga_PID(Sub_Algas Alga, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Alga=Alga;
    this.setpoint=setpoint;
    //addRequirements(Alga);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kp=0.03;
    error= setpoint- Alga.getEncoderBrazo();
    double speed;
    speed= error*kp;
    Alga.set_Brazo(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <.5;
  }
}
