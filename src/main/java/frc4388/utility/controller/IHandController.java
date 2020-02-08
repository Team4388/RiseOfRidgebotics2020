package frc4388.utility.controller;

/**
 * Interface for the {@link XboxController}.
 */
public interface IHandController {

	public double getLeftXAxis();

	public double getLeftYAxis();

	public double getRightXAxis();
	
	public double getRightYAxis();

	public double getLeftTriggerAxis();

	public double getRightTriggerAxis();

	public int getDpadAngle();
}
