/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.DriveConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.commands.InterruptSubystem;
import frc4388.robot.commands.auto.Barrel;
import frc4388.robot.commands.auto.BarrelMany;
import frc4388.robot.commands.auto.BarrelStart;
import frc4388.robot.commands.auto.Bounce;
import frc4388.robot.commands.auto.DriveOffLineBackward;
import frc4388.robot.commands.auto.DriveOffLineForward;
import frc4388.robot.commands.auto.EightBallAutoMiddle;
import frc4388.robot.commands.auto.FiveBallAutoMiddle;
import frc4388.robot.commands.auto.GalacticSearch;
import frc4388.robot.commands.auto.IdentifyPath;
import frc4388.robot.commands.auto.SequentialTest;
import frc4388.robot.commands.auto.SixBallAutoMiddle;
import frc4388.robot.commands.auto.Slalom;
import frc4388.robot.commands.auto.TenBallAutoMiddle;
import frc4388.robot.commands.auto.Wait;
import frc4388.robot.commands.climber.DisengageRachet;
import frc4388.robot.commands.climber.RunClimberWithTriggers;
import frc4388.robot.commands.climber.RunLevelerWithJoystick;
import frc4388.robot.commands.drive.DriveStraightAtVelocityPID;
import frc4388.robot.commands.drive.DriveWithJoystick;
import frc4388.robot.commands.drive.PlaySongDrive;
import frc4388.robot.commands.drive.SkipSong;
import frc4388.robot.commands.drive.TurnDegrees;
import frc4388.robot.commands.intake.RunIntakeWithTriggers;
import frc4388.robot.commands.shooter.CalibrateShooter;
import frc4388.robot.commands.shooter.TrackTarget;
import frc4388.robot.commands.shooter.TrimShooter;
import frc4388.robot.commands.shooter.RunHoodWithJoystick;
import frc4388.robot.commands.shooter.ShootPrepGroup;
import frc4388.robot.commands.shooter.ShooterGoalPosition;
import frc4388.robot.commands.shooter.ShooterManual;
import frc4388.robot.commands.shooter.ShooterTrenchPosition;
import frc4388.robot.commands.shooter.ShooterVelocityControlPID;
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
import frc4388.utility.controller.JoystickManualButton;
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
    private final ShooterAim m_robotShooterAim = new ShooterAim();
    private final ShooterHood m_robotShooterHood = new ShooterHood();
    private final Climber m_robotClimber = new Climber();
    private final Leveler m_robotLeveler = new Leveler();
    private final Storage m_robotStorage = new Storage();

    /* Cameras */
    private final Camera m_robotCameraFront = new Camera("front", 0, 160, 120, 40);
    private final Camera m_robotCameraBack = new Camera("back", 1, 160, 120, 40);
    public final LimeLight m_robotLime = new LimeLight();

    /* Controllers */

    public boolean isGS = false;
    private static XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    private static XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);
    private static XboxController m_buttonFox = new XboxController(OIConstants. BUTTON_FOX_ID);
    private static XboxController m_manualXbox = new XboxController(3);

    /* Autos */
    double m_totalTimeAuto;

    SixBallAutoMiddle m_sixBallAutoMiddle;

    SixBallAutoMiddle m_sixBallAutoMiddle0;

    SixBallAutoMiddle m_sixBallAutoMiddle1;

    EightBallAutoMiddle m_eightBallAutoMiddle;

    DriveOffLineForward m_driveOffLineForward;

    DriveOffLineBackward m_driveOffLineBackward;

    FiveBallAutoMiddle m_fiveBallAutoMiddle;

    TenBallAutoMiddle m_tenBallAutoMiddle;

    Slalom m_slalom;

    Barrel m_barrel;

    BarrelStart m_barrelStart;

    BarrelMany m_barrelMany;

    Bounce m_bounce;

    SequentialTest m_sequentialTest;

    GalacticSearch m_galacticSearch;

    public static boolean m_isShooterManual = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* Passing Drive and Pneumatics Subsystems */
        m_robotPneumatics.passRequiredSubsystem(m_robotDrive);
        m_robotDrive.passRequiredSubsystem(m_robotPneumatics, m_robotShooter);

        m_robotShooter.passRequiredSubsystem(m_robotShooterHood, m_robotShooterAim);
        m_robotShooterHood.passRequiredSubsystem(m_robotShooter);
        m_robotShooterAim.passRequiredSubsystem(m_robotShooter, m_robotDrive);

        m_robotLeveler.passRequiredSubsystem(m_robotClimber);

        configureButtonBindings();

        /* Builds Autos */
        //buildAutos();

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller

        m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_robotPneumatics, getDriverController()));
        //m_robotDrive.setDefaultCommand(new DriveWithJoystickUsingDeadAssistPID(m_robotDrive, m_robotPneumatics, getDriverController()));

        // drives intake with input from triggers on the opperator controller
        m_robotIntake.setDefaultCommand(new RunIntakeWithTriggers(m_robotIntake, getOperatorController()));
        // runs the turret with joystick
        m_robotShooterAim.setDefaultCommand(new RunCommand(() -> m_robotShooterAim.runShooterWithInput(-m_operatorXbox.getLeftXAxis()), m_robotShooterAim));
        // runs the hood with joystick
        m_robotShooterHood.setDefaultCommand(new RunHoodWithJoystick(m_robotShooterHood, getOperatorController()));
        // moves the drum not
        m_robotShooter.setDefaultCommand(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(12000), m_robotShooter));
        // drives climber with input from triggers on the opperator controller
        m_robotClimber.setDefaultCommand(new RunClimberWithTriggers(m_robotClimber, getDriverController()));
        // drives the leveler with an axis input from the driver controller
        m_robotLeveler.setDefaultCommand(new RunLevelerWithJoystick(m_robotLeveler, getOperatorController()));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));
        // runs the storage not
        m_robotStorage.setDefaultCommand(new ManageStorage(m_robotStorage));
        m_robotLime.setDefaultCommand(new RunCommand(() -> m_robotLime.limeOff(), m_robotLime));

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
            .whileHeld(new ShooterTrenchPosition(m_robotShooter, m_robotShooterHood, m_robotShooterAim))
            .whenReleased(new InterruptSubystem(m_robotShooter))
            .whenReleased(new InterruptSubystem(m_robotShooterHood));

        // B driver test button
        new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotDrive.runDriveVelocityPID(2000, 45), m_robotDrive));

        // Y driver test button
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotDrive.runTurningPID(1000), m_robotDrive));

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
            .whenPressed(new RunCommand(() -> m_robotStorage.runStorage(1.0), m_robotStorage))
            .whenReleased(new InterruptSubystem(m_robotStorage));

        // extends or retracts the extender
        new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotIntake.runExtender(0.5)))
            .whenReleased(new InstantCommand(() -> m_robotIntake.runExtender(0)));
            //.whileHeld(new RunCommand(() -> m_robotShooterHood.runHood(0.2), m_robotShooterHood));

        new JoystickButton(getOperatorJoystick(), XboxController.Y_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotIntake.runExtender(-0.5)))
            .whenReleased(new InstantCommand(() -> m_robotIntake.runExtender(0)));
            //.whileHeld(new RunCommand(() -> m_robotShooterHood.runHood(-0.2), m_robotShooterHood));

        // safety for climber and leveler
        new JoystickButton(getOperatorJoystick(), XboxController.BACK_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotClimber.setSafetyPressed(), m_robotClimber))
            .whenReleased(new InstantCommand(() -> m_robotClimber.setSafetyNotPressed(), m_robotClimber));

        // starts tracking target
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whileHeld(new TrackTarget(m_robotShooterAim))
            .whileHeld(new RunCommand(() -> m_robotShooterHood.runAngleAdjustPID(m_robotShooterHood.addFireAngle())))
            .whenReleased(new InstantCommand(() -> m_robotLime.limeOff()));

            //.whileHeld(new RunCommand(() -> m_robotShooterAim.runshooterRotatePID()));
            //.whileHeld(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(11000)));
            //.whileHeld(new HoldTarget(m_robotShooter, m_robotShooterAim))
            //.whileHeld(new RunCommand(() -> m_robotShooter.runAngleAdjustPID(30)));

        //Trims shooter
        new JoystickButton(getOperatorJoystick(), XboxController.TOP_BOTTOM_DPAD_AXIS)
            .whenPressed(new TrimShooter(m_robotShooter));

        //Calibrates turret and hood
        new JoystickButton(getOperatorJoystick(), XboxController.START_BUTTON)
            .whileHeld(new CalibrateShooter(m_robotShooter, m_robotShooterAim, m_robotShooterHood));



        //Run drum
        new JoystickManualButton(getOperatorJoystick(), XboxController.B_BUTTON, false)
            .whileHeld(new ShootPrepGroup(m_robotShooter, m_robotShooterAim, m_robotShooterHood, m_robotStorage), false)
            //.whenReleased(new ManageStorage(m_robotStorage, StorageMode.RESET))
            .whenReleased(new InstantCommand(() -> m_robotLime.limeOff()));

        //Run drum manual
        new JoystickManualButton(getOperatorJoystick(), XboxController.B_BUTTON, true)
            .whileHeld(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(10000)))
            .whenReleased(new RunCommand(() -> m_robotShooter.runDrumShooterVelocityPID(0)));



        /* Button Fox */
        // Storage Manual
        new JoystickButton(getButtonFox(), ButtonFox.LEFT_SWITCH)
            .whenPressed(new InstantCommand(() -> m_robotStorage.changeStorageMode(StorageMode.MANUAL)))
            .whenReleased(new InstantCommand(() -> m_robotStorage.changeStorageMode(StorageMode.RESET)));

        // Meg
        new JoystickButton(getButtonFox(), ButtonFox.MIDDLE_SWITCH)
            .whileHeld(new PlaySongDrive(m_robotDrive, m_robotShooter))
            .whenReleased(new InterruptSubystem(m_robotDrive));

        // Shooter Manual
        new JoystickButton(getButtonFox(), ButtonFox.RIGHT_SWITCH)
            .whileHeld(new ShooterManual(true))
            .whenReleased(new ShooterManual(false));

        // Goal Shooter Position
        new JoystickButton(getButtonFox(), ButtonFox.RIGHT_BUTTON)
            .whileHeld(new PlaySongDrive(m_robotDrive, m_robotShooter))
            .whenReleased(new InterruptSubystem(m_robotDrive));

        // Trench Shooter Position
        new JoystickButton(getButtonFox(), ButtonFox.LEFT_BUTTON)
            .whileHeld(new ShooterTrenchPosition(m_robotShooter, m_robotShooterHood, m_robotShooterAim))
            .whenReleased(new InterruptSubystem(m_robotShooter))
            .whenReleased(new InterruptSubystem(m_robotShooterHood))
            .whenReleased(new InterruptSubystem(m_robotShooterAim));
            //.whenPressed(new SkipSong(m_robotDrive, 1));
    }

    public void buildAutos() {
        //resetOdometry(new Pose2d(0, 0, new Rotation2d(180)));

        String[] sixBallAutoMiddlePaths = new String[]{
            "SixBallMidComplete"
        };

        m_sixBallAutoMiddle = new SixBallAutoMiddle(m_robotDrive, buildPaths(sixBallAutoMiddlePaths));

        String[] sixBallAutoMiddle0Paths = new String[]{
            "SixBallMid0"
        };

        m_sixBallAutoMiddle0 = new SixBallAutoMiddle(m_robotDrive, buildPaths(sixBallAutoMiddle0Paths));

        String[] sixBallAutoMiddle1Paths = new String[]{
            "SixBallMid1"
        };

        m_sixBallAutoMiddle1 = new SixBallAutoMiddle(m_robotDrive, buildPaths(sixBallAutoMiddle1Paths));

        String[] slalom = new String[]{
            "Slalom"
        };

        m_slalom = new Slalom(m_robotDrive, buildPaths(slalom));

        String[] barrel = new String[]{
            "BarrelStart"
        };

        m_barrel = new Barrel(m_robotDrive, buildPaths(barrel));

        String[] barrelStart = new String[]{
            "Barrel"
        };

        m_barrelStart = new BarrelStart(m_robotDrive, buildPaths(barrelStart));

        String[] bounce = new String[]{
            "Bounce1",
            "Bounce2",
            "Bounce3",
            "Bounce4"
        };

        m_bounce = new Bounce(m_robotDrive, this, buildPaths(bounce));

        String[] barrelMany = new String[]{
            "BarrelManyWaypoints"
        };

        m_barrelMany = new BarrelMany(m_robotDrive, buildPaths(barrelMany));

        String[] eightBallAutoMiddlePaths = new String[]{
            "EightBallMidComplete"
        };

        m_eightBallAutoMiddle = new EightBallAutoMiddle(m_robotDrive, buildPaths(eightBallAutoMiddlePaths));

        String[] driveOffLineForwardPaths = new String[]{
            "DriveOffLineForward"
        };

        m_driveOffLineForward = new DriveOffLineForward(m_robotDrive, this, buildPaths(driveOffLineForwardPaths));

        String[] driveOffLineBackwardPaths = new String[]{
            "DriveOffLineBackward"
        };

        m_driveOffLineBackward = new DriveOffLineBackward(m_robotDrive, buildPaths(driveOffLineBackwardPaths));

        String[] fiveBallAutoMiddlePaths = new String[]{
            "FiveBallMidComplete"
        };

        m_fiveBallAutoMiddle = new FiveBallAutoMiddle(m_robotDrive, buildPaths(fiveBallAutoMiddlePaths));

        String[] tenBallAutoMiddlePaths = new String[]{
            "SixBallMidComplete",
            "TenBallMidComplete"
        };
        m_tenBallAutoMiddle = new TenBallAutoMiddle(m_robotShooterHood, m_robotStorage, m_robotIntake, m_robotShooter,
                m_robotShooterAim, m_robotDrive,buildPaths(tenBallAutoMiddlePaths));

        String[] galacticSearchPaths = new String[]{
              "GSC_ARED",
              "GSC_ABLUE",
              "GSC_BRED",
              "GSC_BBLUE"
        };

        m_galacticSearch = new GalacticSearch(m_robotLime, m_robotIntake, buildPaths(galacticSearchPaths));

    }

    public void idenPath()
    {
        m_robotLime.identifyPath();
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create custom trajectories
        //TrajectoryConfig config = getTrajectoryConfig();
        //Trajectory trajectory = getTrajectory(config);
        //RamseteCommand ramseteCommand = getRamseteCommand(trajectory);
        // Run path following command, then stop at the end.
        try {
            SmartDashboard.putNumber("Trajectory Total Time", m_totalTimeAuto);
            //return m_sixBallAutoMiddle.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_sixBallAutoMiddle1.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_eightBallAutoMiddle.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_driveOffLineBackward.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_fiveBallAutoMiddle.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_tenBallAutoMiddle.andThen(()-> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_slalom.andThen(()-> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_barrel.andThen(()-> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_barrelStart.andThen(()-> m_robotDrive.tankDriveVelocity(0, 0));
            //return m_sequentialTest.andThen(() -> m_robotDrive.tankDriveVelocity(0,0));
            return m_galacticSearch.andThen(() -> m_robotDrive.tankDriveVelocity(0,0));


        } catch (Exception e) {
            System.err.println("ERROR");
        }

        return new InstantCommand();
    }

    TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            DriveConstants.MAX_SPEED_METERS_PER_SECOND,
            DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    }

    public RamseteCommand getRamseteCommand(Trajectory trajectory) {
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_robotDrive::getPose,
            new RamseteController(),
            DriveConstants.kDriveKinematics,
            m_robotDrive::tankDriveVelocity,
            m_robotDrive);

        return ramseteCommand;
    }

    public RamseteCommand[] buildPaths(String[] paths) {
        RamseteCommand[] ramseteCommands = new RamseteCommand[paths.length];
        double[] times = new double[paths.length];
        Trajectory initialTrajectory;
        m_totalTimeAuto = 0;

        try {
            if (true) {
                String path = paths[0];
                String trajectoryJSON = "paths/" + path + ".wpilib.json";
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);

                SmartDashboard.putString("trajectoryPath Initial", trajectoryPath.toString());

                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                initialTrajectory = trajectory;
                RamseteCommand ramseteCommand = getRamseteCommand(trajectory.relativeTo(initialTrajectory.getInitialPose()));
                ramseteCommands[0] = ramseteCommand;
                times[0] = initialTrajectory.getTotalTimeSeconds();
            }

            for(int i = 1; i < paths.length; i++) {
                String path = paths[i];
                String trajectoryJSON = "paths/" + path + ".wpilib.json";
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                RamseteCommand ramseteCommand = getRamseteCommand(trajectory.relativeTo(initialTrajectory.getInitialPose()));
                ramseteCommands[i] = ramseteCommand;
                times[i] = trajectory.getTotalTimeSeconds();
            }
        } catch (Exception e) {
            DriverStation.reportError("Unable to open trajectory", e.getStackTrace());
        }

        for (int i = 0; i < times.length; i++) {
            m_totalTimeAuto += times[i];
        }

        return ramseteCommands;
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
    public void resetOdometry(Pose2d pose) {
        m_robotDrive.setOdometry(pose);
    }

    public void resetGyroYawRobotContainer(double angle) {
        m_robotDrive.resetGyroYaw(angle);
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
