/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc4388.robot.Constants.*;
import frc4388.robot.commands.DrivePositionMPAux;
import frc4388.robot.commands.DriveStraightAtVelocityPID;
import frc4388.robot.commands.DriveStraightToPositionMM;
import frc4388.robot.commands.DriveStraightToPositionPID;
import frc4388.robot.commands.DriveWithJoystick;
import frc4388.robot.commands.GotoCoordinates;
import frc4388.robot.commands.RunClimberWithTriggers;
import frc4388.robot.commands.RunExtenderOutIn;
import frc4388.robot.commands.RunIntakeWithTriggers;
import frc4388.robot.commands.StorageIntakeGroup;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.commands.RunLevelerWithJoystick;
import frc4388.robot.commands.TrackTarget;
import frc4388.robot.commands.TurnDegrees;
import frc4388.robot.commands.Wait;
import frc4388.robot.commands.storageOutake;
import frc4388.robot.subsystems.Camera;
import frc4388.robot.subsystems.Leveler;
import frc4388.robot.subsystems.Pneumatics;
import frc4388.robot.subsystems.Storage;
import frc4388.utility.controller.IHandController;
import frc4388.utility.controller.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystems */
    private final Drive m_robotDrive = new Drive();
    private final Pneumatics m_robotPneumatics = new Pneumatics();
    private final LED m_robotLED = new LED();
    private final Intake m_robotIntake = new Intake();
    private final Shooter m_robotShooter = new Shooter();
    private final Climber m_robotClimber = new Climber();
    private final Leveler m_robotLeveler = new Leveler();
    private final Storage m_robotStorage = new Storage();

    /* Cameras */
    private final Camera m_robotCameraFront = new Camera("front",0,160,120,40);
    private final Camera m_robotCameraBack = new Camera("back",1,160,120,40);

    /* Controllers */
    private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* Passing Drive and Pneumatics Subsystems */
        m_robotPneumatics.passRequiredSubsystem(m_robotDrive);
        m_robotDrive.passRequiredSubsystem(m_robotPneumatics);

        configureButtonBindings();

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller
        m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, getDriverController()));
        // drives intake with input from triggers on the opperator controller
        m_robotIntake.setDefaultCommand(new RunIntakeWithTriggers(m_robotIntake, getOperatorController()));
        // runs the drum shooter in idle mode
        m_robotShooter.setDefaultCommand(new RunCommand(() -> m_robotShooter.runShooterWithInput(m_operatorXbox.getLeftXAxis()), m_robotShooter));
        // drives climber with input from triggers on the opperator controller
        m_robotClimber.setDefaultCommand(new RunClimberWithTriggers(m_robotClimber, getDriverController()));
        // drives the leveler with an axis input from the driver controller
        m_robotLeveler.setDefaultCommand(new RunLevelerWithJoystick(m_robotLeveler, getDriverController()));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));
    }
      

    /**
    * Use this method to define your button->command mappings.  Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
        /* Test Buttons */
        // A driver test button
        new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
            .whenPressed(new DriveStraightToPositionMM(m_robotDrive, 240));
        
        // B driver test button
        new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
            .whileHeld(new DriveStraightAtVelocityPID(m_robotDrive, 6000));

        // Y driver test button
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .whenPressed(new Wait(m_robotDrive, 0));

        // X driver test button
        new JoystickButton(getDriverJoystick(), XboxController.X_BUTTON)
            .whenPressed(new DriveStraightToPositionPID(m_robotDrive, 60));
     
        /* Driver Buttons */
        // sets solenoids into high gear
        new JoystickButton(getDriverJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotPneumatics.setShiftState(true), m_robotDrive));

        // sets solenoids into low gear
        new JoystickButton(getDriverJoystick(), XboxController.LEFT_BUMPER_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotPneumatics.setShiftState(false), m_robotDrive));

        /* Operator Buttons */
        //TODO: Shooter Buttons
        // shoots until released
        //new JoystickButton(getOperatorJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
        //    .whileHeld(new ShootShooter(m_robotShooter, m_robotStorage, 5));

        // shoots one ball
        //new JoystickButton(getOperatorJoystick(), XboxController.LEFT_BUMPER_BUTTON)
        //    .whileHeld(new ShootShooter(m_robotShooter, m_robotStorage, 1));
            
        // aims the turret
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whileHeld(new TrackTarget(m_robotShooter));
            //.whenPressed(new RunCommand(() -> m_robotStorage.storeAim()));
        
        // extends or retracts the extender
        new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
            .whenPressed(new RunExtenderOutIn(m_robotIntake));

        // safety for climber and leveler
        new JoystickButton(getOperatorJoystick(), XboxController.BACK_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotClimber.setSafetyPressed(), m_robotClimber))
            .whenReleased(new InstantCommand(() -> m_robotClimber.setSafetyNotPressed(), m_robotClimber));

        /* Storage Neo PID Test */
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whileHeld(new TrackTarget(m_robotShooter));

        //Prepares storage for intaking
        new JoystickButton(getOperatorJoystick(), XboxController.LEFT_TRIGGER_AXIS)
            .whileHeld(new StorageIntakeGroup(m_robotIntake, m_robotStorage));
            
        //Runs storage to outtake
        new JoystickButton(getOperatorJoystick(), XboxController.RIGHT_TRIGGER_AXIS)
            .whileHeld(new storageOutake(m_robotStorage));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = getTrajectoryConfig();
        Trajectory trajectory = getTrajectory(config);
        RamseteCommand ramseteCommand = getRamseteCommand(trajectory);
        // Run path following command, then stop at the end.
        //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
       
        //Runs an Autonomous command group that would shoot our preloaded balls, pick up 3 more from the trench, and shoot those
        //This assumes that we are positioned against the right wall with our shooter facing the target.
        return new SequentialCommandGroup(new Wait(m_robotDrive, 0), 
                                    //add aim command
                                    //add shooter command
                                    //new DriveStraightToPositionMM(m_robotDrive, 48.0),
                                    /*new ParallelCommandGroup(
                                        new StorageIntakeGroup(m_robotIntake, m_robotStorage),
                                            new DriveStraightToPositionMM(m_robotDrive, 36.0)),
                                    new ParallelCommandGroup(
                                         new StorageIntakeGroup(m_robotIntake, m_robotStorage),
                                            new DriveStraightToPositionMM(m_robotDrive, 36.0)),
                                    new StorageIntakeGroup(m_robotIntake, m_robotStorage),*/
                                    //add aim command 
                                    //add shooter command
//Below this would be the picking up additional balls outside of those in the trench directly behind us

                                    
                                    new GotoCoordinates(m_robotDrive, 75, 44, -90),
                                    //Start Intake Ball 1
                                    new GotoCoordinates(m_robotDrive, 0, 12, 0),
                                    new GotoCoordinates(m_robotDrive, 0, 28, 0),
                                    //Start Intake Ball 2
                                    new GotoCoordinates(m_robotDrive, 0, 8, 0),
                                    new GotoCoordinates(m_robotDrive, 0, 28, 0),
                                     //Start Intake Ball 3
                                     new GotoCoordinates(m_robotDrive, 0, 8, 0)
                                     /*Shoot 3 Balls*/ );


                                    /*new GotoCoordinates(m_robotDrive, 0, 68.75, 0),*/
                                    //new StorageIntakeGroup(m_robotIntake, m_robotStorage),
    }
    TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            DriveConstants.MAX_SPEED_METERS_PER_SECOND,
            DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    }

    Trajectory getTrajectory(TrajectoryConfig config) {
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(10, 0)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(20, 20, new Rotation2d(0)),
            // Pass config
            config);
            // 10 = 20, 20 = 35, 30 = 53.5
            // (0,10) = (8,22)
        
        return exampleTrajectory;
    }

    RamseteCommand getRamseteCommand(Trajectory trajectory) {
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_robotDrive::getPose,
            new RamseteController(), 
            DriveConstants.kDriveKinematics,
            m_robotDrive::tankDriveVelocity,
            m_robotDrive);
        
        return ramseteCommand;
    }

    /**
     * Sets Motors to a NeutralMode.
     * @param mode NeutralMode to set motors to
     */
    public void setDriveNeutralMode(NeutralMode mode) {
        m_robotDrive.setDriveTrainNeutralMode(mode);
    }

    /**
     * Sets the gear of the drivetrain
     * @param state the gearing of the gearbox (true is high, false is low)
     */
    public void setDriveGearState(boolean state) {
        m_robotPneumatics.setShiftState(state);
    }

    /**
     * 
     */
    public void resetOdometry() {
        m_robotDrive.resetGyroAngles();
        m_robotDrive.setOdometry(new Pose2d());
    }

    /**
     * Used for analog inputs like triggers and axises.
     * @return IHandController interface for the Driver Controller.
     */
    public IHandController getDriverController() {
        return m_driverXbox;
    }
    
    /**
     * Used for analog inputs like triggers and axises.
     * @return The IHandController interface for the Operator Controller.
     */
    public IHandController getOperatorController()
    {
        return m_operatorXbox;
    }
    
    /**
     * Gets the {@link edu.wpi.first.wpilibj.GenericHID#GenericHID(int) Generic HID} for the Operator Xbox Controller.
     * Generic HIDs/Joysticks can be used to set up JoystickButtons.
     * @return The IHandController interface for the Operator Controller.
     */
    public Joystick getOperatorJoystick()
    {
        return m_operatorXbox.getJoyStick();
    }
    
    /**
     * Gets the {@link edu.wpi.first.wpilibj.GenericHID#GenericHID(int) Generic HID} for the Driver Xbox Controller.
     * Generic HIDs/Joysticks can be used to set up JoystickButtons.
     * @return The IHandController interface for the Driver Controller.
     */
    public Joystick getDriverJoystick()
    {
        return m_driverXbox.getJoyStick();
    }
}
