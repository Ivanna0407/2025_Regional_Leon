// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Brazo extends SubsystemBase {
  /** Creates a new Sub_Brazo. */
  private final SparkFlex Brazo_1 = new SparkFlex(18, MotorType.kBrushless);
  private final SparkFlex Brazo_2 = new SparkFlex(19, MotorType.kBrushless);
  private final SparkFlexConfig Config_brazo = new SparkFlexConfig();
  private final SparkFlexConfig Config_brazo_1 = new SparkFlexConfig();
  public Sub_Brazo() {
    Config_brazo.idleMode(IdleMode.kBrake);
    Config_brazo_1.idleMode(IdleMode.kBrake);
    Config_brazo.inverted(false);
    Config_brazo_1.inverted(false);
    //Config_brazo_1.follow(18);
    Brazo_1.configure(Config_brazo, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Brazo_2.configure(Config_brazo_1, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder", get_encoder_brazo());
  }


  public void set_velocity(double speed){
    Brazo_1.set(speed);
    Brazo_2.set(speed);
  }

  public double get_encoder_brazo(){
    return Brazo_1.getEncoder().getPosition();
  }
}
