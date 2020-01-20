/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrackTarget extends CommandBase {
    //Setup Objects
  NetworkTableEntry xEntry;
  Drive m_drive;
  IHandController m_driverController;
    //Aiming Variables
  double turnAmount = 0;
  double xAngle = 0;//Angle from center
  double yAngle = 0;//Angle from center
  double target = 0;//0 or 1
  double FOV = 29.8;//Field of view
    //Distance Calc Constants
  double TARGET_HEIGHT = 82.75;
  double LIME_ANGLE = 24.11;
  double distance;
  
  /**
   * Starts tracking the target
   */
  public TrackTarget(Drive driveSubsystem, IHandController driverController) {
    m_drive = driveSubsystem;
    addRequirements(m_drive);
    m_driverController = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
      turnAmount = (xAngle/FOV)*0.65;
      if (Math.abs(xAngle) < 1.3){turnAmount = 0;}//Deadzone
      else if(turnAmount > 0 && turnAmount < 0.3){turnAmount = 0.3;}
      else if(turnAmount < 0 && turnAmount > -0.3){turnAmount = -0.3;}
      m_drive.driveWithInput(m_driverController.getLeftYAxis(), turnAmount);

        //Finding Distance
      distance = TARGET_HEIGHT/Math.tan((LIME_ANGLE + yAngle)*(Math.PI/180));
      System.err.println("Measured: " + distance);
      distance = ((distance*1.1279)-15.0684);
      System.err.println("Calc: " + distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
