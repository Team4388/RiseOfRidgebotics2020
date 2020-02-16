/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.commands.DriveStraightAtVelocityPID;
import frc4388.robot.commands.DriveStraightToPositionMM;
import frc4388.robot.commands.DriveStraightToPositionPID;
import frc4388.robot.commands.DriveWithJoystick;
import frc4388.robot.commands.DriveWithJoystickAuxPID;
import frc4388.robot.commands.RunClimberWithTriggers;
import frc4388.robot.commands.RunExtenderOutIn;
import frc4388.robot.commands.RunIntakeWithTriggers;
import frc4388.robot.commands.ShooterVelocityControlPID;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.commands.RunLevelerWithJoystick;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Leveler;
import frc4388.robot.subsystems.Storage;
import frc4388.utility.LEDPatterns;
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
    private final LED m_robotLED = new LED();
    private final Intake m_robotIntake = new Intake();
    private final Shooter m_robotShooter = new Shooter();
    private final Climber m_robotClimber = new Climber();
    private final Leveler m_robotLeveler = new Leveler();
    private final Storage m_robotStorage = new Storage();

    /* Controllers */
    private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller
        m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, getDriverController()));
        // drives intake with input from triggers on the opperator controller
        m_robotIntake.setDefaultCommand(new RunIntakeWithTriggers(m_robotIntake, getOperatorController()));
        // drives climber with input from triggers on the opperator controller
        m_robotClimber.setDefaultCommand(new RunClimberWithTriggers(m_robotClimber, getDriverController()));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));
        // runs the drum shooter in idle mode
        m_robotShooter.setDefaultCommand(new RunCommand(() -> m_robotShooter.runDrumShooter(0.15), m_robotShooter));
        // drives the leveler with an axis input from the driver controller
        m_robotLeveler.setDefaultCommand(new RunLevelerWithJoystick(m_robotLeveler, getDriverController()));
    }
      

    /**
    * Use this method to define your button->command mappings.  Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
            .whenPressed(new DriveStraightToPositionPID(m_robotDrive, 144));

        /* Operator Buttons */
        // activates "Lit Mode"
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
            .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));
      
        new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
            .whileHeld(new ShooterVelocityControlPID(m_robotShooter, 4000));
        
        new JoystickButton(getOperatorJoystick(), XboxController.LEFT_BUMPER_BUTTON)
            .whenPressed(new RunExtenderOutIn(m_robotIntake));
      
        /* PID Test Command */
        // runs velocity PID while driving straight
        new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
            .whenPressed(new DriveStraightAtVelocityPID(m_robotDrive, 500))
            .whenReleased(new InstantCommand(() -> System.out.print("Gamer"), m_robotDrive));
        
        //new JoystickButton(getDriverJoystick(), XboxController.RIGHT_BUMPER_BUTTON)
        //    .whileHeld(new DriveWithJoystickAuxPID(m_robotDrive, getDriverController()));
      
        // resets the yaw of the pigeon
        new JoystickButton(getDriverJoystick(), XboxController.X_BUTTON)
            .whenPressed(new DriveStraightToPositionMM(m_robotDrive, 72));
      
        // turn 45 degrees
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .whenPressed(new RunCommand(() -> m_robotDrive.runTurningPID(45), m_robotDrive));

        // sets solenoids into high gear
        new JoystickButton(getDriverJoystick(), XboxController.START_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotDrive.setShiftState(true), m_robotDrive));

        // sets solenoids into low gear
        new JoystickButton(getDriverJoystick(), XboxController.BACK_BUTTON)
            .whenPressed(new InstantCommand(() -> m_robotDrive.setShiftState(false), m_robotDrive));
        
        // interrupts any running command
        new JoystickButton(getDriverJoystick(), XboxController.LEFT_JOYSTICK_BUTTON)
            .whenPressed(new InstantCommand(() -> System.out.print("Gamer"), m_robotDrive));

        /* Storage Neo PID Test */
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whileHeld(new RunCommand(() -> m_robotStorage.runStoragePositionPID(0.5)));

        //Prepares storage for intaking
        new JoystickButton(getOperatorJoystick(), XboxController.LEFT_TRIGGER_AXIS)
            .whileHeld(new RunCommand(() -> m_robotStorage.storageIntake(m_robotIntake)));
            
        //Runs storage to outtake
        new JoystickButton(getOperatorJoystick(), XboxController.RIGHT_TRIGGER_AXIS)
            .whileHeld(new RunCommand(() -> m_robotStorage.storageOuttake()));
    }
      
    /**
     * Sets Motors to a NeutralMode.
     * @param mode NeutralMode to set motors to
     */
    public void setDriveNeutralMode(NeutralMode mode) {
        m_robotDrive.setDriveTrainNeutralMode(mode);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // no auto
        return new InstantCommand();
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
