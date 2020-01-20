/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {
  /**
   * Creates and initiates the camera server
   */
  public Cameras() {
    CameraServer.getInstance().startAutomaticCapture("Front", 0);
    CameraServer.getInstance().startAutomaticCapture("Back", 1);
  }

  @Override
  public void periodic() {
    
  }
}
