package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Elevador;
import frc.robot.subsystems.Sub_LEDs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_CeroWrist extends Command {
  /** Creates a new Cmd_Coral_PID. */
  private final Sub_Elevador Elevador;
  private double setpoint;
  private Sub_LEDs leds;
  double error_coral,kp;
  public Cmd_CeroWrist(Sub_Elevador elevador, Sub_LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Elevador=elevador;
    this.leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = 0;
    leds.set_wait();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kp=.0074;
    error_coral=setpoint-Elevador.getEncoderWrist();
    double speed;
    speed=error_coral*kp;
    Elevador.set_Wrist(-speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.set_water();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(error_coral)<4){
      System.out.println("wrist");
      return true;
    }
    else{
      return false;
    }
  }
}
