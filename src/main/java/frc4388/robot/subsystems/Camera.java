/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Camera extends SubsystemBase {
  CameraServer camServ = CameraServer.getInstance();
  /**
   * Creates a new Camera.
   * Makes a Camera and sends the stream to a CameraServer, to be viewed in Shuffle Board.
   * @param name Name of the Camera in Shuffle Board.
   * @param id USB Id of the Camera.
   * @param width Resolution width.
   * @param height Resolution height.
   * @param brightness Percent brightness of the stream.
   */
  public Camera(String name, int id, int width, int height, int brightness) {
    try{
      UsbCamera cam = new UsbCamera(name, id);
      cam.setResolution(width, height);
      cam.setBrightness(brightness);
      cam.setFPS(10);
      VideoSource camera = cam;
      camServ.startAutomaticCapture(camera);
    } 
    catch(Exception e){
      System.err.println("Camera broken, pls nerf");
    }
  }  

  @Override
  public void periodic() {
  }
}