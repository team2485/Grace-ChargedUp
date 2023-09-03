package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class Intake extends SubsystemBase  {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kIntakeTalonPort);

  
  private double tx = 0;

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          kSIntakeVolts, kVIntakeVoltSecondsPerMeter, kAIntakeVoltSecondsSquaredPerMeter);

 
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;


  private double m_lastOutputVoltage = 0;

  public Intake() {
    TalonFXConfiguration intakeTalonConfig = new TalonFXConfiguration();
    intakeTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    intakeTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    intakeTalonConfig.velocityMeasurementWindow = 1;

    intakeTalonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kIntakeSupplyCurrentLimitAmps,
            kIntakeSupplyCurrentThresholdAmps,
            kIntakeSupplyCurrentThresholdTimeSecs);
    intakeTalonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kIntakeStatorCurrentLimitAmps,
            kIntakeStatorCurrentThresholdAmps,
            kIntakeStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(intakeTalonConfig);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.enableVoltageCompensation(true);
    m_talon.setInverted(true);
  }

  /** @return the current velocity in rotations per second. */

  public double getVelocityRotationsPerSecond() {
    return m_talon.getSelectedSensorVelocity() / (kIntakeGearRatio * kFalconSensorUnitsPerRotation);
  }

  /**
   * Sets the velocity setpoint for the feeeder.
   *
   * @param rotationsPerSecond velocity setpoint
   */

  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */

  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }


  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kIntakeVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    double outputVoltage = 0;
    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput =
          m_feedforward.calculate(
              m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond, kIntakeLoopTimeSeconds);

      outputVoltage = feedforwardOutput;
      m_feedforwardOutput = feedforwardOutput;
    }

    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(outputVoltage);
    }
    m_lastOutputVoltage = outputVoltage;
  }

  public void periodic() {
    this.runControlLoop();
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
}
