// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Elevador_PID extends Command {
  /** Creates a new Cmd_Elevador_PID. */
  private final Sub_Elevador Elevador;
  private final double setpoint;
  double kp,error,ki,last_time,error_i,integral_zone,dt;
  public Cmd_Elevador_PID(Sub_Elevador elevador, double Setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Elevador=elevador;
    this.setpoint=Setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    integral_zone=setpoint*.1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ki=.008;
    kp=.55;
    dt=Timer.getFPGATimestamp()-last_time;
    error = setpoint - Elevador.getElevatorEncoder();
    double speed;
    speed=error*kp+error_i*ki;
    if(Math.abs(error)< integral_zone){error_i+=error*dt;}
    Elevador.setElevador(speed);
    last_time=Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
      if (setpoint-Elevador.getElevatorEncoder()<.4){
        return true;
      }
      else{return false;}
  }
}
