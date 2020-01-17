/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

/**
 * Add your docs here.
 */
public class Gains {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public int kIzone;
    public double kPeakOutput;

    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput)
    {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }
}
