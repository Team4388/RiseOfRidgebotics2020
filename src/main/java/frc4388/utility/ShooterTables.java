/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class ShooterTables {
    double[][] m_distance = new double[50][3];
    double[][] m_angle = new double[50][2];

    final int m_columnDistance = 0;
    final int m_columnHood = 1;
    final int m_columnVel = 2;
    final int m_columnAngle = 0;
    final int m_columnDisplacement = 1;

    int m_distanceLength;
    int m_angleLength;

    public ShooterTables() {
        int lineNum = 0;

        File distanceCSV = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/Robot Data - Distances.csv");
        File angleCSV = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/Robot Data - Angles.csv");
        
        try {


            BufferedReader distanceReader = new BufferedReader(new FileReader(distanceCSV));
            BufferedReader angleReader = new BufferedReader(new FileReader(angleCSV));
            String line = "";

            while ((line = distanceReader.readLine()) != null) {

                if (lineNum == 0) {
                    lineNum ++;
                } else {
                    String[] values = line.split(",");

                    m_distance[lineNum - 1][m_columnDistance] = Double.parseDouble(values[0]);
                    m_distance[lineNum - 1][m_columnHood] = Double.parseDouble(values[1]);
                    m_distance[lineNum - 1][m_columnVel] = Double.parseDouble(values[2]);
                    
                    lineNum ++;
                }

            }

            m_distanceLength = lineNum-1;
            lineNum = 0;

            while ((line = angleReader.readLine()) != null) {

                if (lineNum == 0) {
                    lineNum ++;
                } else {
                    String[] values = line.split(",");

                    m_angle[lineNum - 1][m_columnAngle] = Double.parseDouble(values[0]);
                    m_angle[lineNum - 1][m_columnDisplacement] = Double.parseDouble(values[1]);
                    
                    lineNum ++;
                }
            }

            m_angleLength = lineNum-1;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        SmartDashboard.putNumber("Row 2 Column 1", m_angle[1][0]);
        SmartDashboard.putNumber("Row 4 Column 2", m_angle[3][1]);
        SmartDashboard.putNumber("m_distanceLength", m_distanceLength);
        SmartDashboard.putNumber("Distance last row 0", m_distance[m_distanceLength-1][0]);
        SmartDashboard.putNumber("Distance last row 1", m_distance[m_distanceLength-1][1]);
        SmartDashboard.putNumber("Distance last row 2", m_distance[m_distanceLength-1][2]);
    }

    public double getHood(double distance) {
        int i = 0;
        while ((i < m_distanceLength) && (m_distance[i][m_columnDistance] < distance)) {
            i ++;
        }
        if ((i < m_distanceLength) && (m_distance[i][m_columnDistance] == distance)) {
            return m_distance[i][m_columnHood];
        } else {
            if (i >= m_distanceLength) {
                i = m_distanceLength - 1;
            }
            return linearInterpolation(i, distance, m_columnHood, m_distance);
        }
    }

    public double getVelocity(double distance) {
        int i = 0;
        while ((i < m_distanceLength) && (m_distance[i][m_columnDistance] < distance)) {
            i ++;
        }
        if ((i < m_distanceLength) && (m_distance[i][m_columnDistance] == distance)) {
            return m_distance[i][m_columnVel];
        } else {
            if (i >= m_distanceLength) {
                i = m_distanceLength - 1;
            }
            return linearInterpolation(i, distance, m_columnVel, m_distance);
        }
    }

    public double getAngleDisplacement(double angle) {
        int i = 0;
        while ((i < m_angleLength) && (m_angle[i][m_columnAngle] < angle)) {
            i ++;
        }
        if ((i < m_angleLength) && (m_angle[i][m_columnAngle] == angle)) {
            return m_angle[i][m_columnDisplacement];
        } else {
            if (i >= m_angleLength) {
                i = m_angleLength - 1;
            }
            return linearInterpolation(i, angle, m_columnDisplacement, m_angle);
        }
    }

    public double linearInterpolation(int i, double value, int column, double[][] table) {
        if (i != 0) {
            double slope = (table[i][column] - table[i-1][column]) / (table[i][0] - table[i-1][0]);
            value = slope * (value - table[i-1][0]) + table[i-1][column];
            return value;
        }
        return 0.0;
    }
}