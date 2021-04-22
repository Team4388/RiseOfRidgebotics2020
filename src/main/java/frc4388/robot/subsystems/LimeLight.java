/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new LimeLight.
   */
  public String galacticSearchPath;

  public LimeLight() {

  }

  public void limeOff(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void limeOn(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void changePipeline(int pipelineId)
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
  }

  public double getV()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }
  public double getX()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  public double getY()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  int i = 0;
  boolean onceThrough = false;
  boolean pathFound = false;
  public void identifyPath(){

    if (!onceThrough)
    {
      changePipeline(2);
      if (withinCoords(VisionConstants.aBlue))
      {
        pathFound = true;
        galacticSearchPath = "A_BLUE";
      }

      changePipeline(4);
      if (withinCoords(VisionConstants.bBlue))
      {
        pathFound = true;
        galacticSearchPath = "B_BLUE";
      }

      changePipeline(1);
      if (withinCoords(VisionConstants.aRed))
      {
        pathFound = true;
        galacticSearchPath = "A_RED";
      }

      changePipeline(3);
      if (withinCoords(VisionConstants.bRed))
      {
        pathFound = true;
        galacticSearchPath = "B_RED";
      }

      if (pathFound == false)
      {
        galacticSearchPath = "PathNotFound";
      }
      SmartDashboard.putString("GalacticSearchPath", galacticSearchPath);
      onceThrough = true;
    }
    else{
      i++;
      SmartDashboard.putNumber("Counter", i);
    }
    if (i >= 50)
    {
      i=0;
      pathFound = false;
      onceThrough = false;
    }
  }

public boolean withinError(double angle, double goal)
{
  if(goal > (angle - VisionConstants.searchError) && goal < (angle + VisionConstants.searchError))
  {
    return true;
  }
  else
  {
    return false;
  }
}

public boolean withinCoords(double[] coords)
{
  if (withinError(getX(), coords[0]) && withinError(getY(), coords[1]))
  {
    return true;
  }
  else
  {
    return false;
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
