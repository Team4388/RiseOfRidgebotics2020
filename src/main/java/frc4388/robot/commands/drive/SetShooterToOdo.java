package frc4388.robot.commands.drive;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.ShooterAim;

public class SetShooterToOdo extends CommandBase {
  private ShooterAim m_shooterAim;
  private Drive m_driveTrain;

  private double targetAngle;

  public SetShooterToOdo(ShooterAim shooterAim, Drive driveTrain) {
    m_shooterAim = shooterAim;
    m_driveTrain = driveTrain;
  }

  @Override
  public void initialize() {
    double xPos = m_driveTrain.getPose().getX();
    double yPos = m_driveTrain.getPose().getY();
    targetAngle = Math.atan(yPos / xPos) - m_driveTrain.getPose().getRotation().getDegrees();
  }

  @Override
  public void execute() {
    m_shooterAim.runshooterRotatePID(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_shooterAim.getShooterRotatePosition() - targetAngle) < 5;
  }
}
