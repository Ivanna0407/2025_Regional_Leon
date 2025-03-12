// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Climber extends Command {
  /** Creates a new Cmd_Climber. */
  private final Sub_Climber climber;
  private final Supplier<Double> RT,LT;
  public Cmd_Climber(Sub_Climber climber, Supplier<Double> Climber_speed,Supplier<Double> LT) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber=climber;
    this.RT=Climber_speed;
    this.LT=LT;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed=RT.get()-LT.get();
    if(Math.abs(speed)<.2){
      speed=0;
    }
    climber.setClimber(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
