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
import frc4388.robot.commands.RunIntakeWithTriggers;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.LED;
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
        // drives motor with input from triggers on the opperator controller
        m_robotIntake.setDefaultCommand(new RunIntakeWithTriggers(m_robotIntake, getOperatorController()));
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
        /* Driver Buttons */
        new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
            .whenPressed(new DriveStraightToPositionPID(m_robotDrive, 144));
        // runs velocity PID while driving straight
        new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
            .whenPressed(new DriveStraightAtVelocityPID(m_robotDrive, 500))
            .whenReleased(new InstantCommand(() -> System.out.print("Gamer"), m_robotDrive));
        // resets the yaw of the pigeon
        new JoystickButton(getDriverJoystick(), XboxController.X_BUTTON)
            .whenPressed(new DriveStraightToPositionMM(m_robotDrive, 72));
        // turn 45 degrees
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .whenPressed(new RunCommand(() -> m_robotDrive.runTurningPID(45), m_robotDrive));
        // interrupts any running command
        new JoystickButton(getDriverJoystick(), XboxController.LEFT_JOYSTICK_BUTTON)
            .whenPressed(new InstantCommand(() -> System.out.print("Gamer"), m_robotDrive));

        /* Operator Buttons */
        // activates "Lit Mode"
        new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
            .whenPressed(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW))
            .whenReleased(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN));
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
