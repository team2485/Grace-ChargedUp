// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class SplinePath {
    private ArrayList<Translation2d> pointList = new ArrayList<Translation2d>();
    private ArrayList<Translation2dWithTimestamp> interpolationList = new ArrayList<Translation2dWithTimestamp>();
    
    
    public SplinePath(Translation2d pointA, Translation2d pointB) {
        Translation2d startPoint = new Translation2d(pointA.getX() + .01, pointA.getY() + .01);
        pointList.add(startPoint);
        pointList.add(pointA);
        pointList.add(pointB);
        Translation2d endPoint = new Translation2d(pointB.getX() + .01, pointB.getY() + .01);
        pointList.add(endPoint);
    }
    
    public void drawPath() {
        interpolationList.clear();
        for (int i = 0; i < pointList.size() - 3; i+=1) {
            generate(new Translation2d[]{pointList.get(i), pointList.get(i+1), pointList.get(i+2), pointList.get(i+3)});
        }
        // for (double i = 0; i < 1; i+=.1) {
        //     double t = map(i, 0, 1, interpolationList.get(0).getTimestamp(), interpolationList.get(interpolationList.size()-1).getTimestamp());
        //     Translation2d p = betweenClosestTimestamps(t);
        // }
    }

    public Translation2d getControlPoint(int controlPointIndex) {
        double t = map(controlPointIndex, 0, 10, interpolationList.get(0).getTimestamp(), interpolationList.get(interpolationList.size()-1).getTimestamp());
        return betweenClosestTimestamps(t);
    }
    
    public void generate(Translation2d[] points) {
        for (double i = 0; i < 1; i+=.05) {
            double t0 = 0;
            double t1 = Math.sqrt(dist(points[0].getX(), points[0].getY(), points[1].getX(), points[1].getY())) + t0;
            double t2 = Math.sqrt(dist(points[1].getX(), points[1].getY(), points[2].getX(), points[2].getY())) + t1;
            double t3 = Math.sqrt(dist(points[2].getX(), points[2].getY(), points[3].getX(), points[3].getY())) + t2;
            double t = lerp(t1, t2, i);
            
            Translation2d A1 = points[0].times((t1 - t) / (t1 - t0)).plus(points[1].times((t - t0) / (t1 - t0)));
            Translation2d A2 = points[1].times((t2 - t) / (t2 - t1)).plus(points[2].times((t - t1) / (t2 - t1)));
            Translation2d A3 = points[2].times((t3 - t) / (t3 - t2)).plus(points[3].times((t - t2) / (t3 - t2)));
            
            Translation2d B1 = A1.times((t2 - t) / (t2 - t0)).plus(A2.times((t - t0) / (t2 - t0)));
            Translation2d B2 = A2.times((t3 - t) / (t3 - t1)).plus(A3.times((t - t1) / (t3 - t1)));
            
            Translation2dWithTimestamp C = new Translation2dWithTimestamp(B1.times((t2 - t) / (t2 - t1)).plus(B2.times((t - t1) / (t2 - t1))));
            
            C.setTimestamp(t);
            
            interpolationList.add(C);
        }
    }
    
    public Translation2d betweenClosestTimestamps(double t) {
        Translation2dWithTimestamp a = interpolationList.get(0);
        Translation2dWithTimestamp b = interpolationList.get(1);
        for (int i = 1; i < interpolationList.size()-1; i++) {
            if (b.getTimestamp() > t) break;
            a = interpolationList.get(i);
            b = interpolationList.get(i+1);
        }
        double percent = map(t, a.getTimestamp(), b.getTimestamp(), 0, 1);
        return new Translation2d(lerp(a.getX(), b.getX(), percent), lerp(a.getY(), b.getY(), percent));
    }

    private double dist(double x1, double y1, double x2, double y2) {
        return Math.sqrt(((y2-y1)*(y2-y1))/((x2-x1)*(x2-x1)));
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      }
    
    private double lerp(double v0, double v1, double t) {
        return (1 - t) * v0 + t * v1;
    }


      
}
