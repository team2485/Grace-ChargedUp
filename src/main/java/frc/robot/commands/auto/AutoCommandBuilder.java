package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.CargoHandlingCommandBuilder.*;

import java.nio.file.Path;
import java.util.function.Consumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;
import frc.util.SwerveModuleConstants;

public class AutoCommandBuilder {

  public static Command get1CubeHighAuto(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    return new RunCommand(()-> hood.setAngleRadians(.3))
                            .alongWith(new RunCommand(()-> shooter.setVelocitiesHigh()))
                            .withTimeout(2)
                            .andThen(new RunCommand(()-> feeder.setVelocityRotationsPerSecond(4))
                            .alongWith(new RunCommand(()-> indexer.setVelocityRotationsPerSecond(6))))
                            .withTimeout(4)
                            .andThen(new InstantCommand(()-> indexer.setVoltage(0)),
                            new InstantCommand(()-> feeder.setVoltage(0)),
                            new InstantCommand(()-> shooter.setShooterVoltage(0)));

  }

  public static Command get1CubeMidAuto(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {


    return new RunCommand(()-> hood.setAngleRadians(.3))
                            .alongWith(new RunCommand(()-> shooter.setVelocitiesMid()))
                            .withTimeout(2)
                            .andThen(new RunCommand(()-> feeder.setVelocityRotationsPerSecond(4))
                            .alongWith(new RunCommand(()-> indexer.setVelocityRotationsPerSecond(6))))
                            .withTimeout(4)
                            .andThen(new InstantCommand(()-> indexer.setVoltage(0)),
                            new InstantCommand(()-> feeder.setVoltage(0)),
                            new InstantCommand(()-> shooter.setShooterVoltage(0)));

  }

  public static Command get2CubeAuto(
    Drivetrain drivetrain,
    Intake intake,
    IntakeArm intakeArm,
    Indexer indexer,
    Feeder feeder,
    FeedServo servo,
    Shooter shooter,
    Hood hood) {
        PathConstraints pathConstraints = new PathConstraints(1, 3);
        PathPlannerTrajectory path = PathPlanner.loadPath("BATB_1_Cube", pathConstraints);
        // SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(drivetrain::getPose, 
        //                                                             drivetrain::resetOdometry, 
        //                                                             new PIDConstants(Swerve.driveKP, Swerve.driveKI, Swerve.driveKD),
        //                                                             new PIDConstants(Swerve.angleKP, Swerve.angleKI, Swerve.angleKD), drivetrain::driveAuto, null, 
        //                                                             drivetrain);
        // drivetrain.resetOdometry(path.getInitialHolonomicPose());
        // return new PPSwerveControllerCommand(path, drivetrain::getPose, 
        //        new PIDController(Swerve.driveKP, Swerve.driveKI, Swerve.driveKD), 
        //        new PIDController(Swerve.driveKP, Swerve.driveKI, Swerve.driveKD), 
        //        new PIDController(Swerve.angleKP, Swerve.angleKI, Swerve.angleKD), drivetrain::driveAuto, drivetrain);
        // return new PPSwerveControllerCommand(path, drivetrain::getPose, Swerve.swerveKinematics, 
        //        new PIDController(Swerve.driveKP, Swerve.driveKI, Swerve.driveKD), 
        //        new PIDController(Swerve.driveKP, Swerve.driveKI, Swerve.driveKD), 
        //        new PIDController(Swerve.angleKP, Swerve.angleKI, Swerve.angleKD), drivetrain::setModuleStates, drivetrain);
        WL_SwerveControllerCommand path1 = PathCommandBuilder.getPathSlowCommand(drivetrain, "BATB_1_Cube");
        return path1;

        
  }
}
