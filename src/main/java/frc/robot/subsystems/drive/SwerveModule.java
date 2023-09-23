package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.CTREConfigs;
import frc.util.Conversions;
import frc.util.CTREModuleState;
import frc.util.SwerveModuleConstants;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.Swerve.*;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.hardware.CANcoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private VelocityVoltage mDriveVelocityVoltage = new VelocityVoltage(0);
    private PositionVoltage mAnglePositionVoltage = new PositionVoltage(0);

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private CANcoderConfigurator angleEncoderConfigurator;
    private CANcoderConfiguration angleEncoderConfiguration;

    private TalonFXConfigurator mDriveConfigurator;
    private TalonFXConfigurator mAngleConfigurator;
    private MotorOutputConfigs mDriveOutputConfigs = new MotorOutputConfigs();
    private MotorOutputConfigs mAngleOutputConfigs = new MotorOutputConfigs();
    

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA); 
    private GenericEntry working;
    private GenericEntry target;
    private GenericEntry error;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        working = Shuffleboard.getTab("Swerve").add(String.valueOf(moduleNumber)+" current", 0.0).getEntry();
        target = Shuffleboard.getTab("Swerve").add(String.valueOf(moduleNumber)+" target",0.0).getEntry();
        error = Shuffleboard.getTab("Swerve").add(String.valueOf(moduleNumber)+" error", 0).getEntry();
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfigurator.apply(Robot.ctreConfigs.swerveCanCoderConfig);
        // configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleConfigurator = mAngleMotor.getConfigurator();
        mAngleConfigurator.apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleOutputConfigs.Inverted = angleMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mAngleOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        //mAngleConfigurator.apply(mAngleOutputConfigs);
        
        //configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveConfigurator = mDriveMotor.getConfigurator();
        mDriveConfigurator.apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveOutputConfigs.Inverted = moduleConstants.isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mDriveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        mDriveConfigurator.apply(mDriveOutputConfigs);
        //
        // configDriveMotor(moduleConstants.isInverted);

        lastAngle = getState().angle;

        // VoltageOut mAngleVoltageOut = new VoltageOut(6);
        // mAngleMotor.setControl(mAngleVoltageOut);
        // 
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
            // mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            // mDriveMotor.setControl(mDriveVelocityVoltage.withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)).withVelocity(velocity));
            // mDriveMotor.setControl(mDriveVelocityVoltage.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // target.setDouble(angle.getDegrees());
        // working.setDouble(getAngle().getDegrees());
        mAngleMotor.getClosedLoopIntegratedOutput().refresh();
        target.setDouble(angleOffset.getDegrees());
        // error.setDouble(mAngleMotor.getClosedLoopError().getValue());
        // mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), angleGearRatio));
        mAngleMotor.setControl(mAnglePositionVoltage.withPosition((angle.getDegrees() / 360) * angleGearRatio));
        //mAngleMotor.setVoltage(6);
        // mAngleMotor.setControl(mAnglePositionVoltage.withPosition(angle.getRadians()));
        lastAngle = angle;
    } 

    private void setAngle(double angle)
    {
        mAngleMotor.setControl(mAnglePositionVoltage.withPosition(angle));
    }

    private Rotation2d getAngle(){
        
        //working.setDouble((mAngleMotor.getPosition().getValue() / angleGearRatio) * 360);
        return Rotation2d.fromDegrees((mAngleMotor.getPosition().getValue() / angleGearRatio) * 360);
        //return Rotation2d.fromDegrees(getCanCoder().getDegrees());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    void resetToAbsolute(){
        //double absolutePosition = ((getCanCoder().getDegrees() - angleOffset.getDegrees()) * angleGearRatio) / 360;
        Rotation2d absolutePosition = getCanCoder().minus(angleOffset);

        //setAngle(absolutePosition);
        //mAngleMotor.setRotorPosition(0);
        //angleEncoder.setPosition(absolutePosition);
        //working.setDouble(absolutePosition);
        mAngleMotor.setRotorPosition(absolutePosition.getRotations());
    }

    private void configAngleEncoder(){        
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(angleMotorInvert);
        // mAngleMotor.setNeutralMode(angleNeutralMode);
        // mDriveConfigurator.apply();
        // resetToAbsolute();
    }

    // private void configDriveMotor(boolean isInverted){        
    //     // mDriveMotor.configFactoryDefault();
    //     // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    //     mDriveMotor.setInverted(isInverted);
    //     mDriveMotor.setNeutralMode(driveNeutralMode);
    //     mDriveMotor.setSelectedSensorPosition(0);
    // }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getPosition().getValue(), wheelCircumference, driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getPosition().getValue(), wheelCircumference, driveGearRatio), 
            getAngle()
        );
    }
}