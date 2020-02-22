/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Shooter;
import frc4388.utility.controller.IHandController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrackTarget extends CommandBase {
    //Setup
  NetworkTableEntry xEntry;
  Shooter m_shooter;
  IHandController m_driverController;
    //Aiming
  double turnAmount = 0;
  double xAngle = 0;
  double yAngle = 0;
  double target = 0;
  public double distance;
  public static double fireVel;
  public static double fireAngle;

  /**
   * Uses the Limelight to track the target
   */
  public TrackTarget(Shooter shooterSubsystem) {
    m_shooter = shooterSubsystem;
    addRequirements(m_shooter);
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
      m_shooter.runShooterWithInput(turnAmount/5);

        //Finding Distance
      distance = VisionConstants.TARGET_HEIGHT/Math.tan((VisionConstants.LIME_ANGLE + yAngle)*(Math.PI/180));
      SmartDashboard.putNumber("Distance to Target", distance);

      double yVel = Math.sqrt(2*VisionConstants.GRAV*VisionConstants.TARGET_HEIGHT);
      double xVel = (distance*VisionConstants.GRAV)/(yVel);

      fireVel = Math.sqrt((Math.pow(xVel, 2))+(Math.pow(yVel,2)));
      fireAngle = Math.atan(yVel/xVel);
      m_shooter.m_fireVel = fireVel;
      m_shooter.m_fireAngle = fireAngle;
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
