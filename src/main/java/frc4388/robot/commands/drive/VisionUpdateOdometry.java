package frc4388.robot.commands.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.VOPConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.Drive;
import frc4388.robot.subsystems.LimeLight;
import frc4388.robot.subsystems.ShooterAim;

public class VisionUpdateOdometry extends CommandBase {
  private LimeLight m_limeLight;
  private ShooterAim m_shooterAim;
  private Drive m_driveTrain;

  private double xPos;
  private double yPos;

  /**
   * Uses the lime light to update odometry
   * @param limeLight  replace with Vision subsystem for integration with 2022
   * @param shooterAim replace with Turret subsystem for integration with 2022
   * @param driveTrain replace with Swerve subsystem for integration with 2022
   */
  public VisionUpdateOdometry(LimeLight limeLight, ShooterAim shooterAim, Drive driveTrain) {
    m_limeLight = limeLight;
    m_shooterAim = shooterAim;
    m_driveTrain = driveTrain;
    addRequirements(m_limeLight, m_shooterAim, m_driveTrain);

    // Turn camera on but leave LEDs off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Vision Processing Mode
    m_limeLight.limeOn();
    m_limeLight.changePipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] xPoints = m_limeLight.getXArray();
    double[] yPoints = m_limeLight.getYArray();

    double[] ellipseRadii = getEllipseRadii(xPoints, yPoints);

    // https://www.desmos.com/calculator/qu0pe4rmiv
    // https://math.stackexchange.com/questions/2388747/formula-for-ellipse-formed-from-projecting-a-tilted-circle-onto-the-xy-plane
    // PI - acos((R_y / R_x)^2)
    double viewAngle = Math.PI - Math.acos(Math.pow(ellipseRadii[1] / ellipseRadii[0], 2)); // TODO account for limelight angle of rotation

    double distance = 1.d / Math.tan(viewAngle);
    distance *= VOPConstants.TARGET_HEIGHT; // replace with VisionConstants for 2022

    double[] ypr = new double[3];
    Drive.m_pigeon.getYawPitchRoll(ypr);
    double relativeAngle = Math.toDegrees(m_shooterAim.getShooterAngleDegrees() - ypr[0]);

    xPos = Math.cos(relativeAngle) * distance;
    yPos = Math.sin(relativeAngle) * distance;
  }

  // http://www.lee-mac.com/5pointellipse.html
  // https://math.stackexchange.com/questions/163920/how-to-find-an-ellipse-given-five-points
  // https://towardsdatascience.com/understanding-singular-value-decomposition-and-its-application-in-data-science-388a54be95d
  public static double[] getEllipseRadii(double[] xPoints, double[] yPoints) {
    return null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Translation2d translate = new Translation2d(xPos, yPos);

    double[] ypr = new double[3];
    Drive.m_pigeon.getYawPitchRoll(ypr);
    Rotation2d rotation = new Rotation2d(ypr[0]);

    Pose2d pose = new Pose2d(translate, rotation);
    m_driveTrain.setOdometry(pose);

    return false;
  }
}
