package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;



public class Hood extends SubsystemBase {

  // m = 1, 2, 3, 4, 5, 6
  private final double[] settingTable = new double[] {0.12, 0.16, 0.21, 0.26, 0.28};

  private final CANSparkMax m_spark = new CANSparkMax(kHoodSparkPort, MotorType.kBrushless);
  private SparkMaxLimitSwitch m_limitSwitch =
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kPHood, kIHood, kDHood, kHoodMotionProfileConstraints);

 
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          kSHoodVolts, kGHoodVolts, kVHoodVoltSecondsPerRadian, kAHoodVoltSecondsSquaredPerRadian);

  
  private double m_angleSetpointRadiansCurrent = kHoodBottomPositionRadians;

  private double m_angleSetpointRadiansFinal = m_angleSetpointRadiansCurrent;

  private double m_previousVelocitySetpoint = 0;

  private boolean m_isZeroed = false;

 
  private double distanceToHub = 0;


  private double ty = 0;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kHoodImmediateCurrentLimitAmps);

    m_spark.setInverted(false);

    m_spark.setIdleMode(IdleMode.kBrake);

    m_limitSwitch.enableLimitSwitch(true);

    m_controller.setTolerance(kHoodControllerPositionTolerance);

    this.resetAngleRadians(kHoodBottomPositionRadians);

    Shuffleboard.getTab("Hood").add("Hood controller", m_controller);
  }

  public void allignToHub() {
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double angleToGoal = (37 + ty) * (Math.PI / 180.0);
    // difference between actual goal height and limelight height
    double goalHeight = 2.64 - 0.96;
    distanceToHub = goalHeight / Math.tan(angleToGoal);

    if (distanceToHub >= 1 && distanceToHub < 2) {
      m_angleSetpointRadiansCurrent =
          settingTable[0] + ((distanceToHub - 1) * (settingTable[1] - settingTable[0]));
    } else if (distanceToHub >= 2 && distanceToHub < 3) {
      m_angleSetpointRadiansCurrent =
          settingTable[1] + ((distanceToHub - 2) * (settingTable[2] - settingTable[1]));
    } else if (distanceToHub >= 3 && distanceToHub < 4) {
      m_angleSetpointRadiansCurrent =
          settingTable[2] + ((distanceToHub - 3) * (settingTable[3] - settingTable[2]));
    } else if (distanceToHub >= 4 && distanceToHub < 5) {
      m_angleSetpointRadiansCurrent =
          settingTable[3] + ((distanceToHub - 4) * (settingTable[4] - settingTable[3]));
    }
  }

  /** @return current angle from horizontal */

  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kHoodRadiansPerMotorRev;
  }

 
  public void setAngleRadians(double angle) {
    m_angleSetpointRadiansCurrent =
        MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kHoodRadiansPerMotorRev);
  }

  public boolean getBottomLimitSwitch() {
    return m_limitSwitch.isPressed();
  }

  @Override
  public void periodic() {

    // setpoint ramp
    // if(m_angleSetpointRadiansCurrent<m_angleSetpointRadiansFinal){
    //   m_angleSetpointRadiansCurrent+=0.005;
    // }
    // if(m_angleSetpointRadiansCurrent>m_angleSetpointRadiansFinal){
    //   m_angleSetpointRadiansCurrent-=0.005;
    // }

    double controllerVoltage =
        m_controller.calculate(this.getAngleRadians(), m_angleSetpointRadiansCurrent);

    double feedforwardVoltage =
        m_feedforward.calculate(
            m_angleSetpointRadiansCurrent,
            m_previousVelocitySetpoint,
            m_controller.getSetpoint().velocity);

    m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;

    m_spark.set((controllerVoltage + feedforwardVoltage) / Constants.kNominalVoltage);
  }
}
