// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  private UsbCamera _frontCamera = CameraServer.startAutomaticCapture(0);
  private UsbCamera _upCamera = CameraServer.startAutomaticCapture(1);
  private VideoSink _videoSink = CameraServer.getServer(); 

  private boolean _toggle = false;

  /** Creates a new Camera. */
  public Camera() {
    _videoSink.setSource(_frontCamera);
  }

  public void toggleCamera() {
    _toggle = !_toggle;
    if(_toggle) {
      _videoSink.setSource(_upCamera);
      SmartDashboard.putString("Camera", "UP");
    }
    else {
      _videoSink.setSource(_frontCamera);
      SmartDashboard.putString("Camera", "FRONT");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
