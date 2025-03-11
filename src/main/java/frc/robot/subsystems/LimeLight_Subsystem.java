// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight_Subsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  private NetworkTableEntry V_angle, H_angle, TwoH_angle, hasTargets, botPose, aprilID, targetPose, cameraPose;
  private double[] poseList, targetPoseList, cameraPoseList;

  private NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTable limeLight2 = NetworkTableInstance.getDefault().getTable("limelight2");

  public LimeLight_Subsystem() {
    
    V_angle = limeLight.getEntry("ty");
    H_angle = limeLight.getEntry("tx");
    TwoH_angle = limeLight2.getEntry("tx");
    cameraPose = limeLight.getEntry("targetpose_cameraspace");
    
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

  public double getYaw(){
    poseList = botPose.getDoubleArray(new double[6]);

    double yaw = poseList[5];
    return yaw;
    //Negative yaw is when robot is turned to left of aprilTag
    //Positive yaw is when robot is turned to right of aprilTag
  }

  
  public double getH_angle() {
    return H_angle.getDouble(0);
  }

  public double getV_angle(){
    return V_angle.getDouble(0);
  }

  public double getAprilTagID() {
    return aprilID.getDouble(0);
  }

  public double gethvratio(){
    return H_angle.getDouble(0) / V_angle.getDouble(0);
  }

  public double getvhratio(){
    return V_angle.getDouble(0) / H_angle.getDouble(0);
  }  

  public boolean getHasTargets(){
    if (hasTargets.getDouble(0) == 0){
      return false;
    }
    return true;
  }

  public Pose3d getAprilTagPose3d(){
    targetPoseList = targetPose.getDoubleArray(new double[6]);
    //position
    double x = targetPoseList[0];
    double y = targetPoseList[1];
    double z = targetPoseList[2];
    //rotation
    double roll = targetPoseList[3];
    double pitch = targetPoseList[4];
    double yaw = targetPoseList[5];

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


  public Pose3d getCameraPose3d(){
    cameraPoseList = cameraPose.getDoubleArray(new double[6]);
    //position
    double x = cameraPoseList[0];
    double y = cameraPoseList[1];
    double z = cameraPoseList[2];
    //rotation
    double roll = cameraPoseList[3];
    double pitch = cameraPoseList[4];
    double yaw = cameraPoseList[5];

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

  public double getRelativeRoll(){
    return poseList[3];
  }
 
  public double getRelativePitch(){
    return poseList[4];
  }
  
  public double getRelativeYaw(){
    return poseList[5];
  }  

  public double simulatedYDist(){
    return poseList[2] / (Math.tan(Units.degreesToRadians(getH_angle())));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
