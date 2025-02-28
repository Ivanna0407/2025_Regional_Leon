// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsitemas;

public class Sub_Elevador extends SubsystemBase {
  /** Creates a new Sub_Elevador. */
  private final TalonFX Motor_elevador = new TalonFX(16); //Motor que sube el elevador 
  private final SparkMax Coral = new SparkMax(15, MotorType.kBrushless); //Motor del intake de coral 
  private final SparkMax Wrist = new SparkMax(14, MotorType.kBrushless); //Motor Muñeca
  private final SparkMaxConfig Config_Coral = new SparkMaxConfig();//Config sparks
  private final SparkMaxConfig Config_Wrist = new SparkMaxConfig();
  private final RelativeEncoder Wrist_Encoder = Wrist.getEncoder();
  //Limitswitch
  DigitalInput Top_elevador= new DigitalInput(0);
  DigitalInput Down_elevador = new DigitalInput(1);
  public boolean pieza;
  public Sub_Elevador() {
    var motor_config = new MotorOutputConfigs();
    Motor_elevador.getConfigurator().apply(motor_config);
    Motor_elevador.setNeutralMode(NeutralModeValue.Brake);
    Motor_elevador.set(0);
    Motor_elevador.setPosition(0);
    //Motor intake coral
    Config_Coral.idleMode(IdleMode.kBrake);
    Config_Coral.inverted(false);
    Coral.configure(Config_Coral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Motor muñeca 
    Config_Wrist.idleMode(IdleMode.kBrake);
    Config_Wrist.inverted(false);
    Wrist.configure(Config_Wrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Wrist_Encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder elebador", getElevatorEncoder());
    SmartDashboard.putNumber("Encoderwrist", getEncoderWrist());
    SmartDashboard.putNumber("Corriente Coral",Coral.getOutputCurrent());
    SmartDashboard.putBoolean("Pieza",pieza);
    SmartDashboard.putBoolean("Top elvevador", Top_elevador.get());
    SmartDashboard.putBoolean("Down elvevador", Down_elevador.get());
    if(Coral.getOutputCurrent()>=50){pieza=true;}
  }

  public double getElevatorEncoder(){
    return Motor_elevador.getPosition().getValueAsDouble()*Subsitemas.conversion_elevador;
  }

  public void setElevador(double speed){
    if (getElevatorEncoder()<=0 && speed>0){
      Motor_elevador.set(0);
    }
    if (getElevatorEncoder()>=4.7 && speed<0){
      Motor_elevador.set(0);
    }
    else{
    Motor_elevador.set(speed);
    }
  }

  public void resetEncoderElevador(){
    Motor_elevador.setPosition(0);
  }

  public void set_Coral(double out){
    if(pieza==true && out>0){
    pieza=true;
    }
    Coral.set(out);
  }

  public double getEncoderWrist(){
    return Wrist_Encoder.getPosition()* Subsitemas.conversion_wrist*-1;
  }
  public void resetEncoderWrist(){
    Wrist.getEncoder().setPosition(0);
  }
  public void set_Wrist(double speed){
    Wrist.set(speed);
  }

}
