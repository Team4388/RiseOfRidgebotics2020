/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Cameras extends SubsystemBase {
  /**
   * Creates and initiates the camera server
   */
  public Cameras(int id) {
    try{
      VideoSource cam = CameraServer.getInstance().startAutomaticCapture(id);
      cam.setResolution(220,140);
      cam.setFPS(10);
    }
    catch(Exception e){
      System.err.println("Camera broken, pls nerf");
    }
    
  }

  @Override
  public void periodic() {
  }
}
