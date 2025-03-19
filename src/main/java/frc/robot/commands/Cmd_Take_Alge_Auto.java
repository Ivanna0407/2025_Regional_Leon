// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Algas;
import frc.robot.subsystems.Sub_LEDs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Take_Alge_Auto extends Command {
  /** Creates a new Cmd_Take_Alge_Auto. */
  private final Sub_Algas Algas;
  private Sub_LEDs leds;
  public Cmd_Take_Alge_Auto(Sub_Algas Algas, Sub_LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Algas=Algas;
    this.leds=leds;
   // addRequirements(Algas);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {leds.set_score();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Algas.alga==false){
      Algas.set_Alga(.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {leds.setgood();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Algas.getAlgaCurrent()>=40){
      return true;
    }
    else{return false;}
  }
}
