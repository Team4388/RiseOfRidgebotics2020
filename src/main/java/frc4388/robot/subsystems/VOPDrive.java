package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VOPDrive extends SubsystemBase {
    private PigeonIMU m_pigeon;

    public VOPDrive(PigeonIMU pigeon) {
        m_pigeon = pigeon;
    }

    public double getRotation() {
        double[] ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr);
        return Math.toRadians(ypr[0]);
    }
}