/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

public class Trims{
    public double m_turretTrim;
    public double m_hoodTrim;

    public Trims(double turretTrim, double hoodTrim){
        m_turretTrim = turretTrim;
        m_hoodTrim = hoodTrim;
    }
}