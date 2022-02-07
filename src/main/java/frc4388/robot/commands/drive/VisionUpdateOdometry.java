package frc4388.robot.commands.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N5;
import edu.wpi.first.wpiutil.math.numbers.N6;
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

  private Rotation2d rotation;
  private Translation2d translate;

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
    Drive.m_pigeon.getYawPitchRoll(ypr); // Replace static reference to pigeon with SwerveDrive object for 2022
    double relativeAngle = Math.toDegrees(m_shooterAim.getShooterAngleDegrees() - ypr[0]);
    rotation = new Rotation2d(ypr[0]);

    xPos = Math.cos(relativeAngle) * distance;
    yPos = Math.sin(relativeAngle) * distance;
    translate = new Translation2d(xPos, yPos);
  }

  // http://www.lee-mac.com/5pointellipse.html
  // https://math.stackexchange.com/questions/163920/how-to-find-an-ellipse-given-five-points
  // https://towardsdatascience.com/understanding-singular-value-decomposition-and-its-application-in-data-science-388a54be95d
  // https://www.desmos.com/calculator/ounqguxjac
  // https://en.wikipedia.org/wiki/Five_points_determine_a_conic

  /* solves the determinant of the following matrix
   * | x0^2 x0y0 y0^2 x0 y0 1 |
   * | x1^2 x1y1 y1^2 x1 y1 1 |
   * | x2^2 x2y2 y2^2 x2 y2 1 | = 0
   * | x3^2 x3y3 y3^2 x3 y3 1 |
   * | x4^2 x4y4 y4^2 x4 y4 1 |
   * | x5^2 x5y5 y5^2 x5 y5 1 |
   * for conic equation
   * ax^2 - bxy + cy^2 - dx + fy - g = 0
   */
  public static double[] getEllipseRadii(double[] xPoints, double[] yPoints) {
    double[][] matrix = new double[6][5];

    // Generate matrix
    for(int i = 0; i < 5; i++) {
      matrix[i][0] = xPoints[i] * xPoints[i];
      matrix[i][1] = xPoints[i] * yPoints[i];
      matrix[i][2] = yPoints[i] * yPoints[i];
      matrix[i][3] = xPoints[i] * 1.d;
      matrix[i][4] = 1.d * yPoints[i];
      matrix[i][5] = 1.d;
    }

    double[] coeficients = new double[6];
    int pos = 1;
    for(int i = 0; i < 6; i++) {
      double[][] cofactor = cofactor(matrix, -1, i);
      coeficients[i] = pos * determinant(cofactor);
    
      pos *= -1;
    }
    
    return null;
  }

  public static double determinant(double[][] matrix) {
    if(matrix.length == 2) {
      return (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
    } else {
      double sum = 0;
      int pos = 1;

      for(int i = 0; i < matrix.length; i++) {
        double[][] cofactor = cofactor(matrix, 0, i);
        sum += pos * determinant(cofactor);

        pos *= -1;
      }

      return sum;
    }
  }

  public static double[][] cofactor(double[][] matrix, int row, int col) {
    double[][] cofactor = new double[matrix.length - 1][matrix.length - 1];

    // comments mostly for decoration

    // row count without the excluded row
    int y = 0;
    for(int r = 0; r < matrix.length; r++) {
      // column count without the excluded column
      int x = 0;
      // doesn't add excluded row
      if(r != row) {
        for(int c = 0; c < matrix.length; c++) {
          // doesn't add excluded column
          if(c != col) {
            cofactor[y][x] = matrix[r][c];
            x++;
          }
        }
        y++;
      }
    }

    return cofactor;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d pose = new Pose2d(translate, rotation);
    m_driveTrain.setOdometry(pose); // Replace with adding new pose to Kalman filter

    return false;
  }
}
