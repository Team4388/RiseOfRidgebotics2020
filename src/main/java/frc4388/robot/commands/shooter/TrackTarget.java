/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.LimeLight;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.utility.controller.IHandController;

public class TrackTarget extends CommandBase {
  // Setup
  ShooterAim m_shooterAim;
  Shooter m_shooter;
  ShooterHood m_shooterHood;

  NetworkTableEntry xEntry;
  IHandController m_driverController;
  // Aiming
  double turnAmount = 0;
  double xAngle = 0;
  double yAngle = 0;
  double target = 0;
  public double distance;
  public double realDistance;
  public static double fireVel;
  public static double fireAngle;

  public double m_hoodTrim;
  public double m_turretTrim;

  /**
   * Uses the Limelight to track the target
   * @param shooterSubsystem The Shooter subsystem
   * @param aimSubsystem The ShooterAim subsystem
   */
  public TrackTarget(ShooterAim aimSubsystem) {
    m_shooterAim = aimSubsystem;
    m_shooter = m_shooterAim.m_shooterSubsystem;
    m_shooterHood = m_shooter.m_shooterHoodSubsystem;
    addRequirements(m_shooterAim);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Vision Processing Mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    xAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    yAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    // Finding Distance
    distance = VisionConstants.TARGET_HEIGHT / Math.tan((VisionConstants.LIME_ANGLE + yAngle) * (Math.PI / 180));
    realDistance = (1.09 * distance) - 12.8;


    if (target == 1.0) { // If target in view
      // Aiming Left/Right
      xAngle = xAngle + m_shooter.m_shooterTable.getCenterDisplacement(realDistance);
      turnAmount = (xAngle / VisionConstants.FOV) * VisionConstants.TURN_P_VALUE;
      if (Math.abs(xAngle) < VisionConstants.X_ANGLE_ERROR) {
        turnAmount = 0;
      } // Angle Error Zone
      // Deadzones
      else if (turnAmount > 0 && turnAmount < VisionConstants.MOTOR_DEAD_ZONE) {
        turnAmount = VisionConstants.MOTOR_DEAD_ZONE;
      } else if (turnAmount < 0 && turnAmount > -VisionConstants.MOTOR_DEAD_ZONE) {
        turnAmount = -VisionConstants.MOTOR_DEAD_ZONE;
      }
      m_shooterAim.runShooterWithInput(-turnAmount);// - m_shooter.shooterTrims.m_turretTrim);

      SmartDashboard.putNumber("Distance to Target", realDistance);
      SmartDashboard.putNumber("Center Displacement", m_shooter.m_shooterTable.getCenterDisplacement(realDistance));
        //START Equation Code
      double yVel = Math.sqrt(2 * VisionConstants.GRAV * VisionConstants.TARGET_HEIGHT);
      double xVel = (distance * VisionConstants.GRAV) / (yVel);

      //fireVel = Math.sqrt((Math.pow(xVel, 2))+(Math.pow(yVel,2)));
      //fireAngle = Math.atan(yVel/xVel) * (180/Math.PI);
        //END Equation Code

        //START CSV Code
      fireVel = m_shooter.m_shooterTable.getVelocity(realDistance);
      fireAngle = m_shooter.m_shooterTable.getHood(realDistance); //Note: Ensure to follow because units are different
      //fireAngle = 33;
        //END CSV Code

      //fireVel = SmartDashboard.getNumber("Velocity Target", 0);
      //fireAngle = SmartDashboard.getNumber("Angle Target", 3);

      
      m_shooter.m_fireVel = fireVel;
      m_shooterHood.m_fireAngle = fireAngle;// + m_shooter.shooterTrims.m_hoodTrim;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xAngle < 0.5 && xAngle > -0.5 && target == 1)
    {
      m_shooterAim.m_isAimReady = true;
    } else {
      m_shooterAim.m_isAimReady = false;
    }

    return false;
  }
}
