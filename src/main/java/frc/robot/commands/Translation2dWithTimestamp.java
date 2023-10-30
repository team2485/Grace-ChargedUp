// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Translation2dWithTimestamp extends Translation2d{
    private double timestamp = 0;

    public Translation2dWithTimestamp(double x, double y) {
        super(x, y);
    }

    public Translation2dWithTimestamp(Translation2d t) {
        super(t.getX(), t.getY());
    }

    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
