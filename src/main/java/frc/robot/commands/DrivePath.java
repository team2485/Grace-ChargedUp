// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class DrivePath extends CommandBase {
  /** Creates a new DrivePath. */

  private Translation2d endPoint;
  private Translation2d startPoint;
  private Translation2d midPoint;
  private Translation2d currentControlPoint;
  private DoubleSupplier rotSpeedSupplier;
  private Drivetrain m_drivetrain;
  private DriveAuto m_driveAuto; 

  private PIDController xController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);

  private boolean pathFinished = false;

  private SplinePath path;

  private int controlPointIndex = 0;

  public DrivePath(Translation2d endPoint, Drivetrain drivetrain, DoubleSupplier rotSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.midPoint = new Translation2d(.5, .5);
    this.endPoint = endPoint;
    this.m_drivetrain = drivetrain;
    this.rotSpeedSupplier = rotSpeedSupplier;
    
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drivetrain.resetOdometry(new Pose2d());
    startPoint = m_drivetrain.getPose().getTranslation();
    path = new SplinePath(startPoint, midPoint, endPoint);
    path.drawPath();
    currentControlPoint = path.getControlPoint(controlPointIndex);
    controlPointIndex = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    path.drawPath();
    currentControlPoint = path.getControlPoint(controlPointIndex);

    if (Math.abs(m_drivetrain.getPoseX() - currentControlPoint.getX()) < .1 && Math.abs(m_drivetrain.getPoseY() - currentControlPoint.getY()) < .1) {
      if (controlPointIndex < 10) {
        controlPointIndex++;
      }
      // else pathFinished = true;
    }

    SmartDashboard.putNumber("ControlPoint", controlPointIndex);
    SmartDashboard.putNumber("GoalX", currentControlPoint.getX());
    SmartDashboard.putNumber("CurrentX", m_drivetrain.getPoseX());
    SmartDashboard.putNumber("GoalY", currentControlPoint.getY());
    SmartDashboard.putNumber("CurrentY", m_drivetrain.getPoseY());

    double xError = m_drivetrain.getPoseX() - currentControlPoint.getX();
    double yError = m_drivetrain.getPoseY() - currentControlPoint.getY();

    // if (Math.abs(m_drivetrain.getPoseX() - endPoint.getX()) < .1 && Math.abs(m_drivetrain.getPoseX() - endPoint.getY()) < .1) {
    //   controlPointIndex++;
    // }

    // if (controlPointIndex > 7) {
    //   pathFinished = true;
    // }

    // double xError = m_drivetrain.getPoseX() - endPoint.getX();
    // double yError = m_drivetrain.getPoseY() - endPoint.getY();

    m_driveAuto = new DriveAuto(()->xController.calculate(xError), ()->yController.calculate(yError), rotSpeedSupplier, ()->true, m_drivetrain);
    m_driveAuto.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFinished;
  }
}
