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

  //UsbCamera thing = new UsbCamera("front", 0);
  //UsbCamera thing1 = new UsbCamera("back", 1);
  /**
   * Creates and initiates the camera server
   */
  public Cameras(String name, int id) {
    //CameraServer.getInstance().startAutomaticCapture(thing);
    //CameraServer.getInstance().startAutomaticCapture(thing1);

    
    try{
      
      CameraServer.getInstance().startAutomaticCapture(name, id);
    }
    catch(Exception e){
      System.err.println("Camera broken, pls nerf");
    }
    
  }

  @Override
  public void periodic() {

  }
}
