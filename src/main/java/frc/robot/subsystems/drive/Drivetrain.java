package frc.robot.subsystems.drive;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import javax.swing.plaf.basic.BasicBorders.SplitPaneBorder;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.filter.MedianFilter;

public class Drivetrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    //public WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.Swerve.pigeonID);
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);
    public Pigeon2Configuration config = new Pigeon2Configuration();
    

    double yaw = gyro.getYaw().refresh().getValue();

    GenericEntry targetX, targetY, targetRot;

    private MedianFilter filter = new MedianFilter(5);

    public Drivetrain() {
        targetX = Shuffleboard.getTab("Swerve").add("movementX",0.0).getEntry();
        targetY = Shuffleboard.getTab("Swerve").add("movementY",0.0).getEntry();
        targetRot = Shuffleboard.getTab("Swerve").add("movementRot",0.0).getEntry();
        //gyro.configFactoryDefault();
        this.zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    Rotation2d.fromDegrees(gyro.getYaw().refresh().getValue() * -1)
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        //target.setDouble(translation.getX());

        targetX.setDouble(getPoseX());
        targetY.setDouble(getPoseY());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveAuto(ChassisSpeeds speeds) {
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, true, false);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    


    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public double getPoseX() {
        return swerveOdometry.getPoseMeters().getX();
    }

    public double getPoseY() {
        return swerveOdometry.getPoseMeters().getY();
    }

    public double getPoseRotation() {
        return swerveOdometry.getPoseMeters().getRotation().getDegrees();
    }

    public double getPitch(){
        //return gyro.getRoll() + 4;
        return gyro.getPitch().refresh().getValue();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsInverted() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = new SwerveModulePosition(-mod.getPosition().distanceMeters, mod.getPosition().angle);
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void autoGyro(){
        gyro.setYaw(180);
    }

    
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(filter.calculate(360 - gyro.getYaw().refresh().getValue())) : Rotation2d.fromDegrees(filter.calculate(gyro.getYaw().refresh().getValue()));
    }

    public void resetToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw().times(-1), getModulePositions());  

    //     for(SwerveModule mod : mSwerveMods){
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        
    // }
    }
}