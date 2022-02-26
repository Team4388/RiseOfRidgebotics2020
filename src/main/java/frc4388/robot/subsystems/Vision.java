/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;
import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VOPConstants;

public class Vision extends SubsystemBase {

  public Vision() {
    // TODO
  }

  public ArrayList<Point> getTargetPoints() {
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) != 1)
      return null;
    
    ArrayList<Point> points = new ArrayList<>();

    double[] xCoord = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDoubleArray(new double[0]);
    double[] yCoord = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDoubleArray(new double[0]);
    double[] skew = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDoubleArray(new double[0]);

    double[] hSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDoubleArray(new double[0]);
    double[] vSide = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDoubleArray(new double[0]);

    // SmartDashboard.putNumberArray("Point array", corners);
    for(int i = 0; i < xCoord.length; i += 2*4) {
      Point center = new Point(xCoord[i], yCoord[i]);

      center.x /= VOPConstants.H_FOV;
      center.x += .5;
      center.x *= VOPConstants.LIME_HIXELS;

      center.y /= VOPConstants.V_FOV;
      center.y += .5;
      center.y *= VOPConstants.LIME_VIXELS;

      double mag = Math.hypot(hSide[i]/2, vSide[i]/2);

      Point tLeft = new Point(center.x - hSide[i]/2, center.y + vSide[i]/2);
      double lAngle = Math.atan(-(vSide[i]/2) / (hSide[i]/2));
      lAngle -= Math.toRadians(skew[i]);
      tLeft = new Point(mag * Math.cos(lAngle), mag * Math.sin(lAngle));

      Point tRight = new Point(center.x + hSide[i]/2, center.y + vSide[i]/2);
      double rAngle = Math.atan((vSide[i]/2) / (hSide[i]/2));
      rAngle -= Math.toRadians(skew[i]);
      tRight = new Point(mag * Math.cos(rAngle), mag * Math.sin(rAngle));

      points.add(tLeft);
      points.add(tRight);
    }

    return points;
  }

  public void changePipeline(int pipelineId) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
  }

  public void setLEDs(boolean on) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(on ? 0 : 1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(on ? 3 : 1);
  }
}