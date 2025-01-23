// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Brazo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Move_Brazo extends Command {
  /** Creates a new Cmd_Move_Brazo. */
  private final Sub_Brazo sub_Brazo ;
  private final Supplier<Double> TriggerL, TriggerR;
  public Cmd_Move_Brazo(Sub_Brazo Sub_Brazo, Supplier<Double> triggerl,Supplier<Double> triggerr) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub_Brazo= Sub_Brazo;
    this.TriggerL= triggerl;
    this.TriggerR=triggerr;
    addRequirements(sub_Brazo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = TriggerL.get()-TriggerR.get();
    sub_Brazo.set_velocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sub_Brazo.set_velocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
