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
    public double m_kP;
    public double m_kI;
    public double m_kD;
    public double m_kF;
    public int m_kIzone;
    public double m_kPeakOutput;

    /**
     * Creates Gains object for PIDs
     * @param kP The P value.
     * @param kI The I value.
     * @param kD The D value.
     * @param kF The F value.
     * @param kIzone The zone of the I value.
     * @param kPeakOutput The peak output setting the motors to run the gains at.
     */
    public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput)
    {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;
        m_kIzone = kIzone;
        m_kPeakOutput = kPeakOutput;
    }
}
