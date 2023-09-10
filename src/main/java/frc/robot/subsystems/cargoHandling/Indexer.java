package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.VoltageOut;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Indexer extends SubsystemBase {
  private TalonFX m_talon = new TalonFX(kIndexerTalonPort);

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          kSIndexerVolts, kVIndexerVoltSecondsPerMeter, kAIndexerVoltSecondsSquaredPerMeter);

 
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  private double m_lastVelocity;

  // @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  private double m_lastOutputVoltage = 0;

 
  private double outputVoltage = 0;

  public Indexer() {
    TalonFXConfigurator talonFXConfiguator = m_talon.getConfigurator();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguator.apply(motorConfigs);
  }

  /** @return the current velocity in rotations per second. */
  // @Log(name = "Current velocity (RPS)")
  
  public double getVelocityRotationsPerSecond() {
    return m_talon.getVelocity().getValue()
        / (kIndexerGearRatio * kFalconSensorUnitsPerRotation);
  }

  /**
   * Sets the velocity setpoint for the feeeder.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  // @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  // @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  // @Log(name = "Stator current")
  // public double getStatorCurrent() {
  //   return m_talon.getOutputCurrent();
  // }

  // @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kIndexerVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    outputVoltage = 0;
    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput =
          m_feedforward.calculate(
              m_lastVelocitySetpoint,
              m_velocitySetpointRotationsPerSecond,
              kIndexerLoopTimeSeconds);

      outputVoltage = feedforwardOutput;

      m_feedforwardOutput = feedforwardOutput;
    }

    // m_talon.setVoltage(outputVoltage);
    // m_talon.set(ControlMode.PercentOutput, .2);

    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(outputVoltage);
    }
    m_lastOutputVoltage = outputVoltage;
    m_lastVelocity = this.getVelocityRotationsPerSecond();
  }

  public void periodic() {
    this.runControlLoop();
  }
}
