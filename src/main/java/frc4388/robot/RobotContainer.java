/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.commands.InterruptSubystem;
import frc4388.robot.commands.auto.AutoPath1FromCenter;
import frc4388.robot.commands.auto.Wait;
import frc4388.robot.commands.climber.DisengageRachet;
import frc4388.robot.commands.climber.RunClimberWithTriggers;
import frc4388.robot.commands.climber.RunLevelerWithJoystick;
import frc4388.robot.commands.drive.DriveStraightToPositionMM;
import frc4388.robot.commands.drive.DriveWithJoystick;
import frc4388.robot.commands.drive.PlaySongDrive;
import frc4388.robot.commands.drive.TurnDegrees;
import frc4388.robot.commands.intake.RunIntakeWithTriggers;
import frc4388.robot.commands.shooter.CalibrateShooter;
import frc4388.robot.commands.shooter.ShootPrepGroup;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.commands.shooter.TrimShooter;
import frc4388.robot.commands.storage.ManageStorage;
import frc4388.robot.commands.storage.StoragePrep;
import frc4388.robot.subsystems.Camera;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Leveler;
import frc4388.robot.subsystems.LimeLight;
import frc4388.robot.subsystems.Pneumatics;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.ShooterAim;
import frc4388.robot.subsystems.ShooterHood;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.Storage.StorageMode;
import frc4388.utility.controller.ButtonFox;
import frc4388.utility.controller.IHandController;
import frc4388.utility.controller.XboxController;
import frc4388.utility.controller.XboxTriggerButton;

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
    private final ShooterAim m_robotShooterAim = new ShooterAim();
    private final ShooterHood m_robotShooterHood = new ShooterHood();
    private final Climber m_robotClimber = new Climber();
    private final Leveler m_robotLeveler = new Leveler();
    private final Storage m_robotStorage = new Storage();

    /* Cameras */
    private final Camera m_robotCameraFront = new Camera("front", 0, 160, 120, 40);
    private final Camera m_robotCameraBack = new Camera("back", 1, 160, 120, 40);
    private final LimeLight m_robotLime = new LimeLight();

    /* Controllers */
    private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);
    private final XboxController m_buttonFox = new XboxController(OIConstants. BUTTON_FOX_ID);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* Passing Drive and Pneumatics Subsystems */
        m_robotPneumatics.passRequiredSubsystem(m_robotDrive);
        m_robotDrive.passRequiredSubsystem(m_robotPneumatics);

        m_robotShooter.passRequiredSubsystem(m_robotShooterHood, m_robotShooterAim);
        m_robotShooterHood.passRequiredSubsystem(m_robotShooter);
        m_robotShooterAim.passRequiredSubsystem(m_robotShooter);

        m_robotLeveler.passRequiredSubsystem(m_robotClimber);

        configureButtonBindings();

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller

        m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_robotPneumatics, getDriverController()));
        //m_robotDrive.setDefaultCommand(new DriveWithJoystickUsingDeadAssistPID(m_robotDrive, m_robotPneumatics, getDriverController()));

        // drives intake with input from triggers on the opperator controller
        m_robotIntake.setDefaultCommand(new RunIntakeWithTriggers(m_robotIntake, getOperatorController()));
        // runs the turret with joystick
        m_robotShooterAim.setDefaultCommand(new RunCommand(() -> m_robotShooterAim.runShooterWithInput(-m_operatorXbox.getLeftXAxis()), m_robotShooterAim));
        // moves the drum not
        m_robotShooter.setDefaultCommand(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(1500), m_robotShooter));
        // drives climber with input from triggers on the opperator controller
        m_robotClimber.setDefaultCommand(new RunClimberWithTriggers(m_robotClimber, getDriverController()));
        // drives the leveler with an axis input from the driver controller
        m_robotLeveler.setDefaultCommand(new RunLevelerWithJoystick(m_robotLeveler, getOperatorController()));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));
        // runs the storage not
        //m_robotStorage.setDefaultCommand(new RunCommand(() -> m_robotStorage.runStorage(0), m_robotStorage));
        m_robotStorage.setDefaultCommand(new InstantCommand(() -> m_robotStorage.changeStorageMode(StorageMode.IDLE)));
        //m_robotLime.setDefaultCommand(new RunCommand(() -> m_robotLime.limeOff(), m_robotLime));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Test Buttons */
        // A driver test button
        new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
            .whenPressed(new DriveStraightToPositionMM(m_robotDrive, m_robotPneumatics, 24.0));

        // B driver test button
        new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
            .whenPressed(new TurnDegrees(m_robotDrive, 90));
        // Y driver test button
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .whenPressed(new Wait(m_robotDrive, 0, 0));

        // X driver test button
        new JoystickButton(getDriverJoystick(), XboxController.X_BUTTON)
            .whenPressed(new InstantCommand());



        /* Driver Buttons */
        // sets solenoids into high gear
        new JoystickButton(getDriverJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotPneumatics.setShiftState(true), m_robotDrive));

        // sets solenoids into low gear
        new JoystickButton(getDriverJoystick(), XboxController.LEFT_BUMPER_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotPneumatics.setShiftState(false), m_robotDrive));

        // Disengages the rachet to allow for a climb
        new JoystickButton(getDriverJoystick(), XboxController.BACK_BUTTON)
            .whileHeld(new DisengageRachet(m_robotClimber));




        /* Operator Buttons */
        // shoots until released
        new JoystickButton(getOperatorJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
            //.whileHeld(new ShootFullGroup(m_robotShooter, m_robotShooterAim, m_robotShooterHood, m_robotStorage), false);
            //.whenReleased(new ManageStorage(m_robotStorage, StorageMode.RESET));
            //.whenReleased(new RunCommand(() -> m_robotLime.limeOff()));
            .whenPressed(new RunCommand(() -> m_robotStorage.runStorage(-1), m_robotStorage))
            .whenReleased(new InterruptSubystem(m_robotStorage));

        // shoots one ball
        new JoystickButton(getOperatorJoystick(), XboxController.LEFT_BUMPER_BUTTON)
            //.whenPressed(new ShootFullGroup(m_robotShooter, m_robotShooterAim, m_robotShooterHood, m_robotStorage), false);
            //.whenReleased(new ManageStorage(m_robotStorage, StorageMode.RESET));
            //.whenReleased(new RunCommand(() -> m_robotLime.limeOff()));
            .whenPressed(new RunCommand(() -> m_robotStorage.runStorage(1), m_robotStorage))
            .whenReleased(new InterruptSubystem(m_robotStorage));

        // extends or retracts the extender
        new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotIntake.runExtender(0.5)))
            .whenReleased(new InstantCommand(() -> m_robotIntake.runExtender(0)));
            
        new JoystickButton(getOperatorJoystick(), XboxController.Y_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotIntake.runExtender(-0.5)))
            .whenReleased(new InstantCommand(() -> m_robotIntake.runExtender(0)));

        // safety for climber and leveler
        new JoystickButton(getOperatorJoystick(), XboxController.BACK_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotClimber.setSafetyPressed(), m_robotClimber))
            .whenReleased(new InstantCommand(() -> m_robotClimber.setSafetyNotPressed(), m_robotClimber));

        // starts tracking target
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whileHeld(new TrackTarget(m_robotShooterAim))
            .whileHeld(new RunCommand(() -> m_robotShooterHood.runAngleAdjustPID(m_robotShooterHood.addFireAngle())))
            //.whenPressed(new StoragePrep(m_robotStorage))
            //.whenReleased(new InterruptSubystem(m_robotStorage))
            .whenReleased(new InstantCommand(() -> m_robotLime.limeOff()));
            //.whileHeld(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(13000)));
            //.whileHeld(new HoldTarget(m_robotShooter, m_robotShooterAim))
            //.whileHeld(new RunCommand(() -> m_robotShooter.runAngleAdjustPID(30)));

        //Trims shooter
        new JoystickButton(getOperatorJoystick(), XboxController.TOP_BOTTOM_DPAD_AXIS)
            .whenPressed(new TrimShooter(m_robotShooter));

        //Calibrates turret and hood
        new JoystickButton(getOperatorJoystick(), XboxController.START_BUTTON)
            .whileHeld(new CalibrateShooter(m_robotShooter, m_robotShooterAim, m_robotShooterHood));

        //Prepares storage for intaking
        //new XboxTriggerButton(m_operatorXbox, XboxTriggerButton.LEFT_TRIGGER)
            //.whileHeld(new RunCommand(() -> m_robotStorage.runStorage(0.8)))
            //.whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));
            //.whileHeld(new StorageIntake(m_robotIntake, m_robotStorage));

        //Runs storage to outtake
       //new XboxTriggerButton(m_operatorXbox, XboxTriggerButton.RIGHT_TRIGGER)
            //.whileHeld(new RunCommand(() -> m_robotStorage.runStorage(-0.8)))
            //.whenReleased(new RunCommand(() -> m_robotStorage.runStorage(0.0)));

        //Run drum
        new JoystickButton(getOperatorJoystick(), XboxController.B_BUTTON)
            .whileHeld(new ShootPrepGroup(m_robotShooter, m_robotShooterAim, m_robotShooterHood, m_robotStorage), false)
            //.whenReleased(new ManageStorage(m_robotStorage, StorageMode.RESET))
            .whenReleased(new InstantCommand(() -> m_robotLime.limeOff()));




        /* Button Fox */
        // Storage Manual
        new JoystickButton(getButtonFox(), ButtonFox.LEFT_SWITCH)
            .whenPressed(new InstantCommand(() -> m_robotStorage.changeStorageMode(StorageMode.MANUAL)))
            .whenReleased(new InstantCommand(() -> m_robotStorage.changeStorageMode(StorageMode.RESET)));

        // Meg
        new JoystickButton(getButtonFox(), ButtonFox.MIDDLE_SWITCH)
            .whileHeld(new PlaySongDrive(m_robotDrive))
            .whenReleased(new InterruptSubystem(m_robotDrive));

        // Shooter Manual
        new JoystickButton(getButtonFox(), ButtonFox.RIGHT_SWITCH)
            .whileHeld(new PlaySongDrive(m_robotDrive))
            .whenReleased(new InterruptSubystem(m_robotDrive));

        // Goal Shooter Position
        new JoystickButton(getButtonFox(), ButtonFox.LEFT_BUTTON)
            .whileHeld(new PlaySongDrive(m_robotDrive))
            .whenReleased(new InterruptSubystem(m_robotDrive));

        // Trench Shooter Position
        new JoystickButton(getButtonFox(), ButtonFox.RIGHT_BUTTON)
            .whileHeld(new PlaySongDrive(m_robotDrive))
            .whenReleased(new InterruptSubystem(m_robotDrive));
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
        return new AutoPath1FromCenter(m_robotDrive, m_robotPneumatics);
        //return new AutoPath2FromRight(m_robotDrive, m_robotPneumatics);

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
    public void shiftClimberRachet(boolean state) {
        m_robotClimber.shiftServo(state);
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

    public IHandController getButtonFoxObject()
    {
        return m_buttonFox;
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

    /**
     * Gets the {@link edu.wpi.first.wpilibj.GenericHID#GenericHID(int) Generic HID} for the Button Fox.
     * Generic HIDs/Joysticks can be used to set up JoystickButtons.
     * @return The IHandController interface for the Button Fox.
     */
    public Joystick getButtonFox()
    {
        return m_buttonFox.getJoyStick();
    }

}
