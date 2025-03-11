// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Vision extends Command {
  /** Creates a new Cmd_Vision. */
  private final Sub_Swerve Swerve ;
  private final int side;
  double ty, tx,yaw, kpx,kpy,setpointx,setpointy,errorx,errory,error_giro,setpoint_giro,kgiro;
  public Cmd_Vision(Sub_Swerve swerve,int side ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Swerve=swerve;
    this.side=side;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kpx=.02;
    kpy=.06;
    kgiro=.0001;
    updatevalues();
    if(side==1){
      setpointx=0;
      setpointy=0;
    }
    else{
      setpointx=0;
      setpoint_giro=0;
      setpointy=.5;
    }
    errorx= (setpointx - tx);
    errory= (setpointy - ty);
    if(Math.abs(yaw)<10){
      error_giro=0;
    }
    else{
      error_giro= setpoint_giro- yaw;
    }
    double linear_speed = errory*kpy;
    double rotation_speed = errorx*kpx;
    double giro_speed = error_giro*kgiro;

    ChassisSpeeds chassisSpeeds= new ChassisSpeeds(linear_speed,rotation_speed, giro_speed);
    SwerveModuleState[] moduleStates=frc.robot.Constants.Swerve.swervekinematics.toSwerveModuleStates(chassisSpeeds);
    Swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updatevalues(){
    this.ty=Swerve.getZarray_limelight();
    this.tx=Swerve.getTx();
    this.yaw=Swerve.getarray_limelight();
  }
}
