// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  // no invert
  private final TalonFX m_shooterTalon = new TalonFX(kShooterTalonPort1);
  // invert
  private final TalonFX m_shooterTalon2 = new TalonFX(kShooterTalonPort2);

  // m = 1, 2, 3, 4, 5, 6
  private final double[] settingTable = new double[] {31, 34, 38, 43, 48};

 
  private double distanceToHub = 0;

  private double ty = 0;

  
  double newVelocitySetpointRotationsPerSecond = 34;

  private double m_shooterVelocitySetpointRotationsPerSecond = 0;

 
  private double m_kickerVelocitySetpointRotationsPerSecond = 0;

  private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);
  private VelocityVoltage shooterVelocityVoltage2 = new VelocityVoltage(0);
  private GenericEntry working = Shuffleboard.getTab("Hood").add("isWorking", false).getEntry();

  /** Creates a new Shooter. Controlled with a feedforward and a bang bang controlller. */
  public Shooter() {
    // TalonFXConfiguration shooterTalonConfig = new TalonFXConfiguration();
    // shooterTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    // shooterTalonConfig.supplyCurrLimit =
    //     new SupplyCurrentLimitConfiguration(
    //         true,
    //         kShooterSupplyCurrentLimitAmps,
    //         kShooterSupplyCurrentThresholdAmps,
    //         kShooterSupplyCurrentThresholdTimeSecs);
    // shooterTalonConfig.statorCurrLimit =
    //     new StatorCurrentLimitConfiguration(
    //         true,
    //         kShooterStatorCurrentLimitAmps,
    //         kShooterStatorCurrentThresholdAmps,
    //         kShooterStatorCurrentThresholdTimeSecs);

    // // shooterTalonConfig.peakOutputReverse = 0;
    // shooterTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    // shooterTalonConfig.velocityMeasurementWindow = 1;

    // shooterTalonConfig.slot0.kP = kPShooterOutputUnit100MsPerSensorUnit;
    // shooterTalonConfig.slot0.kF = kFShooterOutputUnit100MsPerSensorUnit * kShooterFeedforwardScale;
    // shooterTalonConfig.slot0.allowableClosedloopError =
    //     kShooterControlVelocityToleranceSensorUnitsPer100Ms;

    // m_shooterTalon.configAllSettings(shooterTalonConfig);
    // m_shooterTalon.setNeutralMode(NeutralMode.Coast);
    // m_shooterTalon.enableVoltageCompensation(true);

    // TalonFXConfiguration shooterTalon2Config = new TalonFXConfiguration();
    // shooterTalon2Config.voltageCompSaturation = Constants.kNominalVoltage;
    // shooterTalon2Config.supplyCurrLimit =
    //     new SupplyCurrentLimitConfiguration(
    //         true,
    //         kKickerSupplyCurrentLimitAmps,
    //         kKickerSupplyCurrentThresholdAmps,
    //         kKickerSupplyCurrentThresholdTimeSecs);
    // shooterTalon2Config.statorCurrLimit =
    //     new StatorCurrentLimitConfiguration(
    //         true,
    //         kKickerStatorCurrentLimitAmps,
    //         kKickerStatorCurrentThresholdAmps,
    //         kKickerStatorCurrentThresholdTimeSecs);

    // // shooterTalon2Config.peakOutputReverse = 0;
    // shooterTalon2Config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    // shooterTalon2Config.velocityMeasurementWindow = 1;

    // shooterTalon2Config.slot0.kP = kPShooterOutputUnit100MsPerSensorUnit;
    // shooterTalon2Config.slot0.kF = kFShooterOutputUnit100MsPerSensorUnit * kKickerFeedforwardScale;
    // shooterTalon2Config.slot0.allowableClosedloopError =
    //     kKickerControlVelocityToleranceSensorUnitsPer100Ms;

    // m_shooterTalon2.configAllSettings(shooterTalon2Config);
    // m_shooterTalon2.setNeutralMode(NeutralMode.Coast);
    // m_shooterTalon2.setInverted(true);
    // m_shooterTalon2.enableVoltageCompensation(true);

    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    // m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    // m_shooterTalon2.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    TalonFXConfigurator shooterConfiguator = m_shooterTalon.getConfigurator();
    TalonFXConfigurator shooterConfiguator2 = m_shooterTalon2.getConfigurator();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Coast;
    Slot0Configs shooterPIDConfigs = new Slot0Configs();
    shooterPIDConfigs.kP = .1;
    shooterPIDConfigs.kI = 0;
    shooterPIDConfigs.kD = 0;
    shooterPIDConfigs.kS = 0;

    MotorOutputConfigs motorConfigs2 = new MotorOutputConfigs();
    motorConfigs2.NeutralMode = NeutralModeValue.Coast;
    motorConfigs2.Inverted = InvertedValue.Clockwise_Positive;
    Slot0Configs shooterPIDConfigs2 = new Slot0Configs();
    shooterPIDConfigs2.kP = .1;
    shooterPIDConfigs2.kI = 0;
    shooterPIDConfigs2.kD = 0;
    shooterPIDConfigs2.kS = 0;

    shooterConfiguator.apply(motorConfigs);
    shooterConfiguator.apply(shooterPIDConfigs);
    shooterConfiguator2.apply(motorConfigs2);
    shooterConfiguator2.apply(shooterPIDConfigs2);
    // this.zeroShooter();
  }

  /** @return the current talon-reported shooter velocity in rotations per second. */
  
  public double getShooterVelocityRotationsPerSecond() {
    return m_shooterTalon.getVelocity().getValue()
        * 10
        / kShooterGearRatio
        / kFalconSensorUnitsPerRotation;
  }

  public void allignToHub() {
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double angleToGoal = (37 + ty) * (Math.PI / 180.0);
    // difference between actual goal height and limelight height
    double goalHeight = 2.64 - 0.96;
    distanceToHub = goalHeight / Math.tan(angleToGoal);

    if (distanceToHub >= 1 && distanceToHub < 2) {
      newVelocitySetpointRotationsPerSecond =
          settingTable[0] + ((distanceToHub - 1) * (settingTable[1] - settingTable[0]));
    } else if (distanceToHub >= 2 && distanceToHub < 3) {
      newVelocitySetpointRotationsPerSecond =
          settingTable[1] + ((distanceToHub - 2) * (settingTable[2] - settingTable[1]));
    } else if (distanceToHub >= 3 && distanceToHub < 4) {
      newVelocitySetpointRotationsPerSecond =
          settingTable[2] + ((distanceToHub - 3) * (settingTable[3] - settingTable[2]));
    } else if (distanceToHub >= 4 && distanceToHub < 5) {
      newVelocitySetpointRotationsPerSecond =
          settingTable[3] + ((distanceToHub - 4) * (settingTable[4] - settingTable[3]));
    }
    MathUtil.clamp(newVelocitySetpointRotationsPerSecond, 0, kShooterMaxSpeedRotationsPerSecond);
  }

  /**
   * Sets shooter and kicker wheel velocities.
   *
   * @param shooterVelocityRotationsPerSecond desired shooter velocity
   * @param tangentialVelocityRatio desired kicker wheel tangential velocity divided by shooter
   *     wheel tangential velocity (m/s) 1/3 will make angular velocities equal, >1 creates topspin,
   *     <1 creates backspin
   */
 
  public void setVelocities() {
    setShooter(10);
    // this.setShooterVelocityRotationsPerSecond();
    // this.setKickerVelocityRotationsPerSecond(
    //     shooterVelocityRotationsPerSecond
    //         * kShooterCircumferenceMeters // desired shooter tangential velocity m/s
    //         * tangentialVelocityRatio // desired kicker tangential velocity m/s
    //         / kKickerCircumferenceMeters); // desired kicker angular velocity rots/sec
  }

  /**
   * Sets shooter and kicker wheel velocities, with default tangential velocity ratio
   *
   * @param shooterVelocityRotationsPerSecond desired shooter velocity
   */
  // public void setVelocities(double shooterVelocityRotationsPerSecond) {
  //   this.setVelocities(shooterVelocityRotationsPerSecond, kDefaultTangentialVelocityRatio);
  // }

  /**
   * Sets the velocity setpoint for the shooter.
   *
   * @param rotationsPerSecond velocity setpoint
   */

  public void setShooterVelocityRotationsPerSecond() {

    // m_shooterTalon.set(
    //     ControlMode.Velocity,
    //     newVelocitySetpointRotationsPerSecond
    //         * kFalconSensorUnitsPerRotation
    //         * kShooterGearRatio
    //         * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);

    // m_shooterTalon2.set(
    //     ControlMode.Velocity,
    //     newVelocitySetpointRotationsPerSecond
    //         * kFalconSensorUnitsPerRotation
    //         * kShooterGearRatio
    //         * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);
    // m_shooterTalon.set(.2);
    // m_shooterTalon2.set(.2);

  }

  public void zeroShooter() {
    // m_shooterTalon.set(
    //     ControlMode.Velocity,
    //     0 * kFalconSensorUnitsPerRotation * kShooterGearRatio * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);

    // m_shooterTalon2.set(
    //     ControlMode.Velocity,
    //     0 * kFalconSensorUnitsPerRotation * kShooterGearRatio * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);

    m_shooterTalon.setControl(shooterVelocityVoltage.withVelocity(0));
    m_shooterTalon2.setControl(shooterVelocityVoltage2.withVelocity(0));
    // m_shooterTalon.setVoltage(0);
    // m_shooterTalon.setVoltage(0);
    // m_shooterTalon.set(0);
    // m_shooterTalon2.set(0);
  }

  public void setShooter(double velocity) {
    // m_shooterTalon.set(
    //     ControlMode.Velocity,
    //     velocity * kFalconSensorUnitsPerRotation * kShooterGearRatio * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);

    // m_shooterTalon2.set(
    //     ControlMode.Velocity,
    //     velocity * kFalconSensorUnitsPerRotation * kShooterGearRatio * 0.1,
    //     DemandType.ArbitraryFeedForward,
    //     newVelocitySetpointRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);

    m_shooterTalon.setControl(shooterVelocityVoltage.withVelocity(15));
    m_shooterTalon2.setControl(shooterVelocityVoltage2.withVelocity(15));

    // m_shooterTalon.set(.2);
    // m_shooterTalon2.set(.2);
  }

  /**
   * Applies the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
 
  public void setShooterVoltage(double voltage) {
    // m_shooterTalon.set(ControlMode.PercentOutput, voltage / kNominalVoltage);
    // m_shooterTalon2.set(ControlMode.PercentOutput, voltage / kNominalVoltage);
    // m_shooterTalon.setVoltage(voltage);
    // m_shooterTalon2.setVoltage(voltage);
  }

 
  public boolean shooterWithinTolerance() {
    return Math.abs(getShooterVelocityRotationsPerSecond() - newVelocitySetpointRotationsPerSecond)
        < 1;
  }

  public double getSetpoint() {
    return m_shooterVelocitySetpointRotationsPerSecond;
  }

  public void periodic() {}
}
