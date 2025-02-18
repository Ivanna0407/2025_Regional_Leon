// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsitemas;

public class Sub_Elevador extends SubsystemBase {
  /** Creates a new Sub_Elevador. */
  private final TalonFX Motor_elevador = new TalonFX(17);
  public Sub_Elevador() {
    var motor_config = new MotorOutputConfigs();
    Motor_elevador.getConfigurator().apply(motor_config);
    Motor_elevador.setNeutralMode(NeutralModeValue.Brake);
    Motor_elevador.set(0);
    Motor_elevador.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder elebador", 0);
  }

  public double getElevatorEncoder(){
    return Motor_elevador.getPosition().getValueAsDouble()*Subsitemas.conversion_elevador;
  }

  public void setElevador(double speed){
    Motor_elevador.set(speed);
  }

  public void resetEncoders(){
    Motor_elevador.setPosition(0);
  }

}
