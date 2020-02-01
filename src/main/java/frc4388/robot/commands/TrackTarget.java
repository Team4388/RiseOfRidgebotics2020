/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrackTarget extends CommandBase {
    //Setup
  NetworkTableEntry xEntry;
  Drive m_drive;
  IHandController m_driverController;
    //Aiming
  double turnAmount = 0;
  double xAngle = 0;
  double yAngle = 0;
  double target = 0;
  double distance;
  
  /**
   * Uses the Limelight to track the target
   */
  public TrackTarget(Drive driveSubsystem, IHandController driverController) {
    m_drive = driveSubsystem;
    addRequirements(m_drive);
    m_driverController = driverController;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //Vision Processing Mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    xAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    yAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    if (target == 1.0){ //If target in view
        //Aiming Left/Right
      turnAmount = (xAngle/VisionConstants.FOV)*VisionConstants.TURN_P_VALUE;
      if (Math.abs(xAngle) < VisionConstants.X_ANGLE_ERROR){turnAmount = 0;} //Angle Error Zone 
        //Deadzones
      else if(turnAmount > 0 && turnAmount < VisionConstants.MOTOR_DEAD_ZONE){turnAmount = VisionConstants.MOTOR_DEAD_ZONE;} 
      else if(turnAmount < 0 && turnAmount > -VisionConstants.MOTOR_DEAD_ZONE){turnAmount = -VisionConstants.MOTOR_DEAD_ZONE;}
      m_drive.driveWithInput(m_driverController.getLeftYAxis(), turnAmount);

        //Finding Distance
      distance = VisionConstants.TARGET_HEIGHT/Math.tan((VisionConstants.LIME_ANGLE + yAngle)*(Math.PI/180));
      SmartDashboard.putNumber("Distance to Target", distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //Drive Camera Mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
