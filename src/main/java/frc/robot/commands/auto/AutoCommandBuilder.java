package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.CargoHandlingCommandBuilder.*;

import java.nio.file.Path;

import java.util.List;
import java.util.function.Consumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveWithController;
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

  public static Command get1CubeHighMobilityAuto(
    Drivetrain drivetrain,
    Intake intake,
    IntakeArm intakeArm,
    Indexer indexer,
    Feeder feeder,
    FeedServo servo,
    Shooter shooter,
    Hood hood) {

  DriveAuto driveAuto = new DriveAuto(()->-.5, ()->0, ()->0, ()->true, drivetrain);

  return new RunCommand(()-> hood.setAngleRadians(.3))
                          .alongWith(new RunCommand(()-> shooter.setVelocitiesHigh()))
                          .withTimeout(2)
                          .andThen(new RunCommand(()-> feeder.setVelocityRotationsPerSecond(4))
                          .alongWith(new RunCommand(()-> indexer.setVelocityRotationsPerSecond(6))))
                          .withTimeout(4)
                          .andThen(new InstantCommand(()-> indexer.setVoltage(0)),
                          new InstantCommand(()-> feeder.setVoltage(0)),
                          new InstantCommand(()-> shooter.setShooterVoltage(0)),
                          new InstantCommand(()-> drivetrain.zeroGyro()),
                          driveAuto.withTimeout(1));
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

  public static Command get1CubeMidMobilityAuto(
    Drivetrain drivetrain,
    Intake intake,
    IntakeArm intakeArm,
    Indexer indexer,
    Feeder feeder,
    FeedServo servo,
    Shooter shooter,
    Hood hood) {

      DriveWithController driveWithController = new DriveWithController(()->-.5, ()->0, ()->0, ()->true, drivetrain);


  return new RunCommand(()-> hood.setAngleRadians(.3))
                          .alongWith(new RunCommand(()-> shooter.setVelocitiesMid()))
                          .withTimeout(2)
                          .andThen(new RunCommand(()-> feeder.setVelocityRotationsPerSecond(4))
                          .alongWith(new RunCommand(()-> indexer.setVelocityRotationsPerSecond(6))))
                          .withTimeout(4)
                          .andThen(new InstantCommand(()-> indexer.setVoltage(0)),
                          new InstantCommand(()-> feeder.setVoltage(0)),
                          new InstantCommand(()-> shooter.setShooterVoltage(0)),
                          new InstantCommand(()-> drivetrain.zeroGyro()),
                          driveWithController.withTimeout(2.75));

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
        // Create a list of bezier points from poses. Each pose represents one waypoint. 
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        // List<Translation2d> bezierPoints = PathPlanner.bezierFromPoses(
        //     new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0),
        //     new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0),
        //     new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)
        // );

        

        // // Create the path using the bezier points created above
        // PathPlannerPath path = new PathPlannerPath(
        //     bezierPoints,
        //     new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        //     new GoalEndState(0.0, Rotation2d.fromDegrees(-90) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        // );

        PathConstraints pathConstraints = new PathConstraints(1, 3);
        PathPlannerTrajectory path = PathPlanner.loadPath("BATB_1_Cube", pathConstraints);

        // drivetrain.resetOdometry(path.getInitialHolonomicPose());
        return new PPSwerveControllerCommand(path, drivetrain::getPose, Swerve.swerveKinematics, 
               new PIDController(1, 0, 0), 
               new PIDController(1, 0, 0), 
               new PIDController(1, 0, 0), drivetrain::setModuleStates, drivetrain);
        //WL_SwerveControllerCommand path1 = PathCommandBuilder.getPathSlowCommand(drivetrain, "BATB_1_Cube");
        //return path1;

        
  }
}
