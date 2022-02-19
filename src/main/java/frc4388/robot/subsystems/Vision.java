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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public Vision() {
    // TODO
  }

  public ArrayList<Point> getTargetPoints() {
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) != 1)
      return null;
    
    ArrayList<Point> points = new ArrayList<>();

    double[] corners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornxy").getDoubleArray(new double[0]);
    for(int i = 0; i < corners.length; i += 2*2) {
      points.add(new Point(corners[i], corners[i+1]));
    }

    return points;
  }

  public void setLEDs(boolean on) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(on ? 0 : 1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(on ? 3 : 1);
  }
}