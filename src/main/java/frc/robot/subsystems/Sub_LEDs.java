// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_LEDs extends SubsystemBase {
  /** Creates a new Sub_LEDs. */
  private int m_handle;
  private int m_handle2;
  private int m_handle3;
  private int m_handle4;

  public Sub_LEDs() {
      m_handle = DIOJNI.initializeDIOPort(HAL.getPort((byte) 8), false);//1
      m_handle2 = DIOJNI.initializeDIOPort(HAL.getPort((byte) 7), false);//2
      m_handle3 = DIOJNI.initializeDIOPort(HAL.getPort((byte) 6), false);//3
      m_handle4 = DIOJNI.initializeDIOPort(HAL.getPort((byte) 5), false);//4
       DIOJNI.setDIO(m_handle, true);
      DIOJNI.setDIO(m_handle2, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    
    
  }

  public void setgood(){
    DIOJNI.setDIO(m_handle, false);
    DIOJNI.setDIO(m_handle2, false);
    DIOJNI.setDIO(m_handle3, true);
    DIOJNI.setDIO(m_handle4, false);
  }

  public void set_idle(){
    DIOJNI.setDIO(m_handle, true);
    DIOJNI.setDIO(m_handle2, false);
    DIOJNI.setDIO(m_handle3, true);
    DIOJNI.setDIO(m_handle4, false);
  }

  public void set_rainbow(){
    DIOJNI.setDIO(m_handle, false);
    DIOJNI.setDIO(m_handle2, true);
    DIOJNI.setDIO(m_handle3, true);
    DIOJNI.setDIO(m_handle4, false);
  }
  public void set_score(){
    DIOJNI.setDIO(m_handle, true);
    DIOJNI.setDIO(m_handle2, true);
    DIOJNI.setDIO(m_handle3, true);
    DIOJNI.setDIO(m_handle4, false);
  }
  public void set_speed(){
    DIOJNI.setDIO(m_handle, false);
    DIOJNI.setDIO(m_handle2,false);
    DIOJNI.setDIO(m_handle3, false);
    DIOJNI.setDIO(m_handle4, true);
  }

  public void set_take(){
    DIOJNI.setDIO(m_handle, true);
    DIOJNI.setDIO(m_handle2, false);
    DIOJNI.setDIO(m_handle3, false);
    DIOJNI.setDIO(m_handle4, true);
  }
  public void set_wait(){
    DIOJNI.setDIO(m_handle, false);
    DIOJNI.setDIO(m_handle2, true);
    DIOJNI.setDIO(m_handle3, false);
    DIOJNI.setDIO(m_handle4, true);
  }
  public void set_water(){
    DIOJNI.setDIO(m_handle, true);
    DIOJNI.setDIO(m_handle2, true);
    DIOJNI.setDIO(m_handle3, false);
    DIOJNI.setDIO(m_handle4, true);
  }


}