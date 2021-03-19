/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.auto;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.LimeLight;

public class IdentifyPath extends CommandBase {

  LimeLight m_limeLight;
  double xAngle;
  double yAngle;
  double target;
  public String path;
  boolean closeVisible;
  boolean finished;

  public IdentifyPath(LimeLight limeLight) {
    m_limeLight = limeLight;
    addRequirements(m_limeLight);
    m_limeLight.limeOff();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLight.limeOn();
    path = "";
    m_limeLight.changePipeline(1); //Dual Targetting Lowest
    //closeVisible = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = m_limeLight.getV();
    xAngle = m_limeLight.getX();
    yAngle = m_limeLight.getY();

      //Identify which of four paths
    if (withinError(yAngle, VisionConstants.bothCloseVisibleY) && !closeVisible) //BLUE PATHS
    {
      closeVisible = true;
    }
    else if (!withinError(yAngle, VisionConstants.bothCloseVisibleY)&& !closeVisible) // RED PATHS
    {
      closeVisible = false;
    }


    if (closeVisible)
    {
      m_limeLight.changePipeline(2); //Dual Targetting Highest
      if(withinError(xAngle, VisionConstants.farLeftVisibleX)) //A PATH
      {
        path = "A_BLUE";
      }
      if(withinError(xAngle, VisionConstants.farRightVisibleX)) //B PATH
      {
        path = "B_BLUE";
      }
    }
    else
    {
      //m_limeLight.changePipeline(1); //Dual Targetting Lowest
      if(withinError(yAngle, VisionConstants.closeLeftVisibleY)) //A PATH
      {
        path = "A_RED";
      }
      else if(withinError(yAngle, VisionConstants.closeRightVisibleY)) //B PATH
      {
        path = "B_RED";
      }
    }

    System.out.println(path);
    SmartDashboard.putString("GalacticSearchPath", path);
    SmartDashboard.putBoolean("CloseVisible", closeVisible);

  }

  public boolean withinError(double angle, double input)
  {
    if(input > (angle - VisionConstants.searchError) && input < (angle + VisionConstants.searchError))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (path != "")
    {
      SmartDashboard.putString("GalacticSearchPath", path);
      m_limeLight.galacticSearchPath = path;
      return true;
    }
    return false;
  }
}
