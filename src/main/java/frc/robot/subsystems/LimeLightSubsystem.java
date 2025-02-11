// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  private NetworkTableEntry V_angle, H_angle, TwoH_angle, hasTargets, botPose, aprilID, targetPose;
  private double[] poseList;

  private NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTable limeLight2 = NetworkTableInstance.getDefault().getTable("limelight2");

  public LimeLightSubsystem() {
    
    V_angle = limeLight.getEntry("ty");
    H_angle = limeLight.getEntry("tx");
    TwoH_angle = limeLight2.getEntry("tx");
    
    hasTargets = limeLight.getEntry("tv");
    botPose = limeLight.getEntry("botpose_targetspace");
    targetPose = limeLight.getEntry("targetpose_robotspace");

    aprilID = limeLight.getEntry("tid");
 
  }

  public Pose3d getRelative3dBotPose() {
    /*
     * Its specific because it determines what type of botpose we need
     * For example, we may need the botpose, botpose_wpiblue, botpose_wpired, etc
     * in order to tell our distance from the apriltag.
     * This method should give us an x and y position to the april tag as well as a rotaiton angle to it
     */
    poseList = botPose.getDoubleArray(new double[6]);
    //position
    double x = poseList[0];
    double y = poseList[1];
    double z = poseList[2];
    //rotation
    double roll = poseList[3];
    double pitch = poseList[4];
    double yaw = poseList[5];

    Pose3d pose3d = new Pose3d(
    x,
    y,
    z,
    new Rotation3d(
      roll,
      pitch,
      yaw
    ));
    return pose3d;
  }   

  
  public double getH_angle() {
    return H_angle.getDouble(0);
  }

  public double getAprilTagID() {
    return aprilID.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
