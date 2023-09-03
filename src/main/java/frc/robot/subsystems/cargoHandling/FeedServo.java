package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.FeederConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedServo extends SubsystemBase {
  private final Servo m_servo = new Servo(kFeederServoPort);

  
  private double m_servoPositionSetpoint = 0;

 
  public void engage(boolean engaged) {
    if (engaged) {
      m_servoPositionSetpoint = kServoEngagedPosition;
    } else {
      m_servoPositionSetpoint = kServoDisengagedPosition;
    }
  }

  
  public void set(double position) {
    m_servoPositionSetpoint = position;
  }

  public double getPositionSetpoint() {
    return m_servoPositionSetpoint;
  }

  @Override
  public void periodic() {
    m_servo.set(m_servoPositionSetpoint);
  }
}
