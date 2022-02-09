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
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private PhotonCamera m_camera;

  public Vision() {
    m_camera = new PhotonCamera(VisionConstants.NAME);
  }

  public ArrayList<Point> getTargetPoints() {
    PhotonPipelineResult result = m_camera.getLatestResult();

    if(!result.hasTargets())
      return null;
    
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
          points.add(new Point(corner.x, corner.y));
      }
    }

    return points;
  }

  public void setLEDs(boolean on) {
    m_camera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }
}