// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable limelight2 = NetworkTableInstance.getDefault().getTable("limelight2");

  public LimeLightSubsystem() {

  }

  public Pose3d getPose3dCam1(){
    Double[] poselist = limelight.getEntry("botpose").getDoubleArray(new Double[6]);
    return new Pose3d(poselist[0], poselist[1], poselist[2], new Rotation3d(poselist[3], poselist[4], poselist[5]));
  }

  public Pose3d getPose3dCam2(){
    Double[] poselist = limelight2.getEntry("botpose").getDoubleArray(new Double[6]);
    return new Pose3d(poselist[0], poselist[1], poselist[2], new Rotation3d(poselist[3], poselist[4], poselist[5]));
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
