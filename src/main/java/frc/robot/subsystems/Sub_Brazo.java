// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsitemas;

public class Sub_Brazo extends SubsystemBase {
  /** Creates a new Sub_Brazo. */
  private final SparkMax Brazo_1 = new SparkMax(13, MotorType.kBrushless);
  private final SparkMax Brazo_2 = new SparkMax(14, MotorType.kBrushless);
  private final SparkMax Coral = new SparkMax(15, MotorType.kBrushless);
  //private final SparkMax Muñeca = new SparkMax(16, MotorType.kBrushless);
  //private final RelativeEncoder Enc_Muñeca = Muñeca.getEncoder();
  private final SparkMaxConfig Config_brazo = new SparkMaxConfig();
  //private final SparkMaxConfig Config_Muñeca = new SparkMaxConfig(); 
  private final SparkMaxConfig Config_Coral = new SparkMaxConfig();
  public Sub_Brazo() {
    Config_brazo.idleMode(IdleMode.kBrake);
    Config_Coral.idleMode(IdleMode.kBrake);
   // Config_Muñeca.idleMode(IdleMode.kBrake);
    Config_brazo.inverted(false);
    Config_Coral.inverted(false);
    Config_Coral.inverted(false);
    //Config_Muñeca.encoder.positionConversionFactor(Subsitemas.Conversion_muñeca);
    Brazo_1.configure(Config_brazo, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Brazo_2.configure(Config_brazo, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Coral.configure(Config_Coral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //  Muñeca.configure(Config_Muñeca, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Muñeca", Enc_Muñeca.getPosition());
  }


  public void set_velocity(double speeds){
    Brazo_1.set(speeds);
    Brazo_2.set(speeds);
  }

  public void set_Coral(double out){
    Coral.set(out);
  }

  public void set_Muneca(double speed){
    //Muñeca.set(speed);
  }

  public double get_encoder_brazo(){
    return Brazo_1.getEncoder().getPosition();
  }
}
