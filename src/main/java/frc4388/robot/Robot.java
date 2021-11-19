/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.robot.Constants.Mode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  double m_initialTime;
  double m_currentTime;
  double m_deltaTime;
  
  SendableChooser<Mode> m_modeChooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Mode.set(Mode.COMPETITIVE);
    m_modeChooser.setDefaultOption(Mode.COMPETITIVE.name(), Mode.COMPETITIVE);
    m_modeChooser.addOption(Mode.CASUAL.name(), Mode.CASUAL);
    SmartDashboard.putData("Mode", m_modeChooser);
    SmartDashboard.putString("Is Auto Start?", "NAH");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setDriveNeutralMode(NeutralMode.Coast);
    /* Builds Autos */
    m_robotContainer.buildAutos();
    SmartDashboard.putString("Is Auto Start?", "NAH");
    m_robotContainer.m_robotLime.limeOff();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.resetOdometry(new Pose2d());
    m_robotContainer.idenPath();
    if (m_modeChooser.getSelected() != Mode.get())
      Mode.set(m_modeChooser.getSelected());
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_robotContainer.buildAutos();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.setDriveNeutralMode(NeutralMode.Coast);
    m_robotContainer.setDriveGearState(true);

    m_initialTime = System.currentTimeMillis();

    //m_robotContainer.resetGyroYawRobotContainer(0);

    //m_robotContainer.configDriveTrainSensors(FeedbackDevice.IntegratedSensor);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      SmartDashboard.putString("Is Auto Start?", "YEA");
    }
    else{

    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    m_currentTime = System.currentTimeMillis();
    m_deltaTime = m_currentTime - m_initialTime;
    SmartDashboard.putNumber("Auto Path Time", (m_deltaTime)/1000);
  }

  @Override
  public void teleopInit() {
    m_robotContainer.setDriveNeutralMode(NeutralMode.Brake);
    m_robotContainer.setDriveGearState(true);

    m_robotContainer.shiftClimberRatchet(false);
    //m_robotContainer.configDriveTrainSensors(FeedbackDevice.IntegratedSensor);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putString("Auto?", "NAH");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }
}
