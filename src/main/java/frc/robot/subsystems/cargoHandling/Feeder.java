package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeederConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class Feeder extends SubsystemBase {
  private final TalonFX m_talon = new TalonFX(kFeederTalonPort);

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          kSFeederVolts, kVFeederVoltSecondsPerMeter, kAFeederVoltSecondsSquaredPerMeter);

  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  // @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;


  private double m_lastOutputVoltage = 0;

  public Feeder() {
    // TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    // talonConfig.supplyCurrLimit =
    //     new SupplyCurrentLimitConfiguration(
    //         true,
    //         kFeederSupplyCurrentLimitAmps,
    //         kFeederSupplyCurrentThresholdAmps,
    //         kFeederSupplyCurrentThresholdTimeSecs);
    // talonConfig.statorCurrLimit =
    //     new StatorCurrentLimitConfiguration(
    //         true,
    //         kFeederStatorCurrentLimitAmps,
    //         kFeederStatorCurrentThresholdAmps,
    //         kFeederStatorCurrentThresholdTimeSecs);

    // m_talon.configAllSettings(talonConfig);

    // m_talon.setInverted(true);
    // m_talon.enableVoltageCompensation(true);
    // m_talon.setNeutralMode(NeutralMode.Brake);
    TalonFXConfigurator talonFXConfiguator = m_talon.getConfigurator();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguator.apply(motorConfigs);
  }

  /** @return the current velocity in rotations per second. */

  public double getVelocityRotationsPerSecond() {
    return m_talon.getVelocity().getValue()
        / (kFeederGearRatio * kFalconSensorUnitsPerRotation * 10);
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

  // @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kFeederVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    double outputVoltage = 0;
    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput =
          m_feedforward.calculate(
              m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond, kFeederLoopTimeSeconds);

      outputVoltage = feedforwardOutput;

      m_feedforwardOutput = feedforwardOutput;
    }
    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(outputVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
