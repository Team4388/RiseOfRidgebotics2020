/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VOPConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.utility.VisionObscuredException;

/** Represents the limelight and odometry related functionality
 * @author Daniel Thomas McGrath
 */
public class VisionOdometry extends SubsystemBase {
  // roborio ip & port: 10.43.88.2:1735
  private PhotonCamera m_camera;

  private VOPDrive m_drive; // replace with swerve drive subsystem
  private VOPShooter m_shooter; // replace with turret subsystem

  /** Creates VisionOdometry
   * 
   * @param drive
   * @param shooter
   */
  public VisionOdometry(VOPDrive drive, VOPShooter shooter) {
    m_camera = new PhotonCamera(VOPConstants.NAME);
    m_drive = drive;
    m_shooter = shooter;
  }

  /** Gets the vision points from the limelight
   * <p>
   * Breaks down targets into 4 corners and uses the top 2 points
   * 
   * @return Vision points on the rim of the target in screen space
   */
  public ArrayList<Point> getTargetPoints() {
    PhotonPipelineResult result = m_camera.getLatestResult();
  
    if(!result.hasTargets())
      return new ArrayList<Point>();
    
    ArrayList<Point> points = new ArrayList<>();

    for(PhotonTrackedTarget target : result.getTargets()) {
      List<TargetCorner> corners = target.getCorners();

      double centerY = 0;
      for(TargetCorner corner : corners) {
        centerY += corner.y;
      }
      centerY /= corners.size();

      for(TargetCorner corner : corners) {
        if(corner.y <= centerY)
          points.add(new Point(corner.x, VOPConstants.LIME_VIXELS - corner.y));
      }
    }

    return points;
  }

  /** Sets LEDs on or off (duh)
   * 
   * @param on LED state
   */
  public void setLEDs(boolean on) {
    m_camera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  /** Gets estimated odometry based on limelight data
   * 
   * @return The estimated odometry pose, including gyro rotation
   * @throws VisionObscuredException
   */
  public Pose2d getVisionOdometry() throws VisionObscuredException {
    ArrayList<Point> screenPoints = getTargetPoints();

    if(screenPoints.size() < 3)
      throw new VisionObscuredException("Not enough vision points available");

    ArrayList<Point3> points3d = get3dPoints(screenPoints);
    ArrayList<Point> points = topView(points3d);

    Point guess = averagePoint(points);

    for(int i = 0; i < 30; i++) {
      guess = iterateGuess(guess, points);
    }

    guess = correctGuessForCenter(guess, m_shooter.getShooterRotation());
    guess = correctGuessForGyro(guess, m_drive.getRotation());

    SmartDashboard.putNumber("Vision ODO x: ", guess.x);
    SmartDashboard.putNumber("Vision ODO y: ", guess.y);

    Rotation2d rotation = new Rotation2d(Math.toDegrees(m_drive.getRotation()));
    Pose2d odometryPose = new Pose2d(guess.x, guess.y, rotation);

    return odometryPose;
  }

  /** Reverse 3d projects target points from screen coordinates to 3d space
   * <p>
   * Uses the known height of the target to project points
   * 
   * @param points2d Vision points on the rim of the target in screen space
   * @return Reverse 3d projected points    
   */
  public static final ArrayList<Point3> get3dPoints(ArrayList<Point> points2d) {
    ArrayList<Point3> points3d = new ArrayList<>();

    for(Point point2d : points2d) {
      double y_rot = point2d.y / VOPConstants.LIME_VIXELS;
      y_rot *= Math.toRadians(VOPConstants.V_FOV);
      y_rot -= Math.toRadians(VOPConstants.V_FOV) / 2;
      y_rot += Math.toRadians(VisionConstants.LIME_ANGLE);

      double x_rot = point2d.x / VOPConstants.LIME_HIXELS;
      x_rot *= Math.toRadians(VOPConstants.H_FOV);
      x_rot -= Math.toRadians(VOPConstants.H_FOV) / 2;

      double z = VOPConstants.TARGET_HEIGHT / Math.tan(y_rot);
      double x = z * Math.tan(x_rot);
      double y = VOPConstants.TARGET_HEIGHT;

      points3d.add(new Point3(x, y, z));
    }

    return points3d;
  }

  /** Flattens 3d points from above
   * 
   * @param points3d 3d points along the target rim
   * @return An array of flattened 3d points
   */
  public static final ArrayList<Point> topView(ArrayList<Point3> points3d) {
    ArrayList<Point> points = new ArrayList<>();

    for(Point3 point3d : points3d) {
      points.add(new Point(point3d.x, point3d.z));
    }

    return points;
  }

  /** Finds the average point from an array of points
   * 
   * @param points The points the average will be taken from
   * @return The average point
   */
  public static final Point averagePoint(ArrayList<Point> points) {
    Point average = new Point(0, 0);
    for(Point point : points) {
      average.x += point.x;
      average.y += point.y;
    }

    average.x /= points.size();
    average.y /= points.size();

    return average;
  }

  /** Iterates the current guess for the vision center (relative to the limelight)
   * based on points on the rim of the target
   * <p>
   * The guess is iterated by finding the current average vector error between the guess
   * and the circlePoints, assuming that the guess should be a constant radius from each point
   * 
   * @param guess The current estimate for the vision center
   * @param circlePoints Vision points along the rim of the target
   * @return The guess after iteration
   */
  public static final Point iterateGuess(Point guess, ArrayList<Point> circlePoints) {
    Point totalDiff = new Point(0, 0);

    for(Point circlePoint : circlePoints) {
      double angle = Math.atan((guess.y - circlePoint.y) / (guess.x - circlePoint.x));
      angle = correctQuadrent(angle, guess, circlePoint);

      Point estimate = new Point();
      estimate.x = VOPConstants.TARGET_RADIUS * Math.cos(angle) + guess.x;
      estimate.y = VOPConstants.TARGET_RADIUS * Math.sin(angle) + guess.y;

      Point diff = new Point(estimate.x - circlePoint.x, estimate.y - circlePoint.y);
      totalDiff.x += diff.x;
      totalDiff.y += diff.y;
    }

    totalDiff.x /= circlePoints.size();
    totalDiff.y /= circlePoints.size();

    return new Point(guess.x - totalDiff.x, guess.y - totalDiff.y);
  }

  /** Corrects odometry guess for shooter angle
   * 
   * @param guess The current guess for the vision center
   * @param shooterRotation The rotation to correct for
   * @return The corrected odometry point
   */
  public static final Point correctGuessForCenter(Point guess, double shooterRotation) {
    Point corrected = new Point(guess.x, guess.y);
    corrected.y += VOPConstants.LIMELIGHT_RADIUS;

    double dist = Math.hypot(guess.x, guess.y);
    double angle = Math.tan(corrected.y / corrected.x);
    angle += shooterRotation;

    corrected.x = dist * Math.cos(angle);
    corrected.y = dist * Math.sin(angle);

    corrected.y += VOPConstants.SHOOTER_CORRECTION;
    
    return corrected;
  }

  /** Corrects odometry guess for gyro angle
   * 
   * @param guess The current guess for the vision center
   * @param gyroRotation The rotation to correct for
   * @return The corrected odometry point
   */
  public static final Point correctGuessForGyro(Point guess, double gyroRotation) {
    Point corrected = new Point(guess.x, guess.y);

    double dist = Math.hypot(guess.x, guess.y);
    double angle = Math.tan(corrected.y / corrected.x);
    angle += gyroRotation;

    corrected.x = dist * Math.cos(angle);
    corrected.y = dist * Math.sin(angle);
    
    return corrected;
  }

  /** Corrects the angle from the current center estimate to a point on the target rim
   * for multiple quadrents
   * 
   * @param angle The angle to be corrected
   * @param guess The current guess for the vision center
   * @param circlePoint A point along the target rim
   * @return The angle corrected for the quadrent
   */
  public static final double correctQuadrent(double angle, Point guess, Point circlePoint) {
    if(circlePoint.x - guess.x < 0) {
      return angle - Math.PI;
    }

    return angle;
  }
}
