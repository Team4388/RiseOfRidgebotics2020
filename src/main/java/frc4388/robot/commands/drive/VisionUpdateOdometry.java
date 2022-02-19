package frc4388.robot.commands.drive;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N5;
import edu.wpi.first.wpiutil.math.numbers.N6;
import frc4388.robot.Constants.VOPConstants;
import frc4388.robot.Constants.VisionConstants;
// import frc4388.robot.subsystems.Drive;
// import frc4388.robot.subsystems.ShooterAim_1;
import frc4388.robot.subsystems.Vision;

public class VisionUpdateOdometry extends CommandBase {
  private Vision m_limeLight;
  // private ShooterAim_1 m_shooterAim;
  // private Drive m_driveTrain;

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
  public VisionUpdateOdometry(Vision limeLight) {
    m_limeLight = limeLight;
    // m_shooterAim = shooterAim;
    // m_driveTrain = driveTrain;
    addRequirements(m_limeLight);//, m_driveTrain);

    // // Turn camera on but leave LEDs off
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Vision Processing Mode
    m_limeLight.setLEDs(true);
    // m_limeLight.changePipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArrayList<Point> points = m_limeLight.getTargetPoints();
    if(points.size() < 5)
      System.out.println("not enough points");

    double[] xPoints = new double[5];
    double[] yPoints = new double[5];

    for(int i = 0; i < 5; i++) {
      xPoints[i] = points.get(i).x;
      yPoints[i] = points.get(i).y;
    }

    double[] ellipseRadii = getEllipseRadii(xPoints, yPoints);

    // https://www.desmos.com/calculator/qu0pe4rmiv
    // https://math.stackexchange.com/questions/2388747/formula-for-ellipse-formed-from-projecting-a-tilted-circle-onto-the-xy-plane
    // PI - acos((R_y / R_x)^2)
    double viewAngle = Math.PI - Math.acos(Math.pow(ellipseRadii[1] / ellipseRadii[0], 2)); // TODO account for limelight angle of rotation

    double distance = 1.d / Math.tan(viewAngle);
    distance *= VOPConstants.TARGET_HEIGHT; // replace with VisionConstants for 2022
    System.out.println("Never gonna give you up: " + distance);

    double[] ypr = new double[3];
    // Drive.m_pigeon.getYawPitchRoll(ypr); // Replace static reference to pigeon with SwerveDrive object for 2022
    double relativeAngle = 0;//Math.toDegrees(/*m_shooterAim.getShooterAngleDegrees()*/ - ypr[0]);
    rotation = new Rotation2d(0);

    xPos = Math.cos(relativeAngle) * distance;
    yPos = Math.sin(relativeAngle) * distance;
    translate = new Translation2d(xPos, yPos);

    Pose2d pose = new Pose2d(translate, rotation);
    // m_driveTrain.setOdometry(pose); // Replace with adding new pose to Kalman filter
    SmartDashboard.putNumber("x:", pose.getX());
    SmartDashboard.putNumber("y:", pose.getY());
    m_limeLight.setLEDs(false);
  }

  // http://www.lee-mac.com/5pointellipse.html
  // https://math.stackexchange.com/questions/163920/how-to-find-an-ellipse-given-five-points
  // https://towardsdatascience.com/understanding-singular-value-decomposition-and-its-application-in-data-science-388a54be95d
  // https://www.desmos.com/calculatoroe_points_determine_a_conic

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
    
    double[] radii = new double[2];

    // https://math.stackexchange.com/questions/280937/finding-the-angle-of-rotation-of-an-ellipse-from-its-general-equation-and-the-ot
    double angle = Math.atan(coeficients[1] / (coeficients[0] - coeficients[2]));
    angle /= 2.d;

    // A' = Acos^2(angle) + Bcos(angle)sin(angle) + Csin^2(angle)
    // B' = 0
    // C' = Asin^2(angle) - Bcos(angle)sin(angle) + Ccos^2(angle)
    // D' = Dcos(angle) + Esin(angle)
    // E' = -Dsin(angle) + Ecos(angle)
    // F' = F
    double A_prime = coeficients[0] * Math.pow(Math.cos(angle), 2) + coeficients[1] * Math.cos(angle) * Math.sin(angle) + coeficients[2] * Math.pow(Math.sin(angle), 2);
    double B_prime = 0;
    double C_prime = coeficients[0] * Math.pow(Math.sin(angle), 2) + coeficients[1] * Math.cos(angle) * Math.sin(angle) + coeficients[2] * Math.pow(Math.cos(angle), 2);
    double D_prime = coeficients[3] * Math.cos(angle) + coeficients[4] * Math.sin(angle);
    double E_prime = -coeficients[3] * Math.sin(angle) + coeficients[4] * Math.cos(angle);
    double F_prime = coeficients[5];

    // r1^2 = (-4F'A'C'+C'D'^2+A'E'^2) / (4A'^2C')
    radii[0] = -4 * F_prime * A_prime * C_prime + C_prime * Math.pow(D_prime, 2) + A_prime * Math.pow(E_prime, 2);
    radii[0] /= 4 * Math.pow(A_prime, 2) * C_prime;
    radii[0] = Math.sqrt(radii[0]);
    // r2^2 = (-4F'A'C'+C'D'^2+A'E'^2) / (4A'C'^2)
    radii[1] = -4 * F_prime * A_prime * C_prime + C_prime * Math.pow(D_prime, 2) + A_prime * Math.pow(E_prime, 2);
    radii[1] = 4 * A_prime * Math.pow(C_prime, 2);
    radii[1] = Math.sqrt(radii[1]);

    return radii;
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
    return true;
  }
}
