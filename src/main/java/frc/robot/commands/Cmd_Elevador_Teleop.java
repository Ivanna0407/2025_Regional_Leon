// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Elevador_Teleop extends Command {
  /** Creates a new Cmd_Elevador_Teleop. */
  private final Sub_Elevador Elevador;
  private final Supplier<Double> RT,LT;
  private final Supplier<Boolean>X,B,LB,RB,Y;
  public Cmd_Elevador_Teleop(Sub_Elevador elevador,Supplier<Double> RT, Supplier<Double> LT,Supplier<Boolean> X,Supplier<Boolean> B, Supplier<Boolean> LB, Supplier<Boolean> RB, Supplier<Boolean> Y) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Elevador=elevador;
    this.RT=RT;
    this.RB=RB;
    this.LB=LB;
    this.LT=LT;
    this.X=X;
    this.B=B;
    this.Y=Y;
    addRequirements(elevador);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevador.setOpenLoopSElevador(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculo speed elevador
    double speed_elevador;
    speed_elevador= RT.get()-LT.get();
    if(Math.abs(speed_elevador)<.2){
      speed_elevador=0;
    }
    Elevador.setElevador(speed_elevador*.8);
    //Calculo speed intake
    if (X.get()) {
      Elevador.set_Coral(.2);
    }                                
    else{
      if (B.get()){
        Elevador.set_Coral(-.2);
      }
      else{Elevador.set_Coral(0);}
    }
    //Calculo muñeca 
    if(RB.get()){
      Elevador.set_Wrist(.2);
    }
    else{
      if (LB.get()){
        Elevador.set_Wrist(-.2);
      }
      else{
        Elevador.set_Wrist(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevador.set_Wrist(0);
    Elevador.set_Coral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
