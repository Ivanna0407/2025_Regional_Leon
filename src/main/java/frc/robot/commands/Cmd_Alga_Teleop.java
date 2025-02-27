// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Algas;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Alga_Teleop extends Command {
  /** Creates a new Cmd_Alga_Teleop. */
  private final Sub_Algas Alga;
  private final Supplier<Boolean> up, down, left, right;
  public Cmd_Alga_Teleop(Sub_Algas algas,Supplier<Boolean> up,Supplier<Boolean> down,Supplier<Boolean> left, Supplier<Boolean> right) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Alga=algas;
    this.down=down;
    this.left=left;
    this.right=right;
    this.up=up;
    addRequirements(Alga);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up.get()){
      Alga.set_Brazo(.2, .2);
    }
    else{
      if(down.get()){
        Alga.set_Brazo(-.2, -.2);
      }
      else{
        Alga.set_Brazo(0, 0);
      }
    }
    if(left.get()){
      Alga.set_Alga(.5);
    }
    else{
      if(right.get()){
        Alga.set_Alga(-.5);
      }
      else{
        Alga.set_Alga(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Alga.set_Alga(0);
    Alga.set_Brazo(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
