// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToTag extends CommandBase {
  /** Creates a new driveToTag. */
  Drivetrain m_drivetrain;
  Vision m_vision;

  PIDController xController = new PIDController(2, 0, 0);
  PIDController yController = new PIDController(-2, .05, 0);
  PIDController rotController = new PIDController(1, 0, 0);
  double desiredX = 0;
  double desiredY = 0;
  double desiredRot = 0;

  public DriveToTag(Drivetrain m_drivetrain, Vision m_vision) {
    this.m_drivetrain = m_drivetrain;
    this.m_vision = m_vision;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d tagPose = m_vision.getDetectedTagPose();
    Pose3d currentPose = m_vision.grabLatestEstimatedPose().estimatedPose;
    if (tagPose != null && currentPose != null) {
      xController.setSetpoint(tagPose.getX());
      yController.setSetpoint(tagPose.getY());
      rotController.setSetpoint(0);

      desiredX = xController.calculate(currentPose.getX());
      desiredY = yController.calculate(currentPose.getY());
      desiredRot = rotController.calculate(m_drivetrain.getYaw().getRotations());
      m_drivetrain.drive(new Translation2d(desiredX, 
                                           desiredY), 
                                           0, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
