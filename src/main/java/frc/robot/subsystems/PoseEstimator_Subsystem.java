// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.Vector;

public class PoseEstimator_Subsystem extends SubsystemBase {
  public static Vector<N3> stateStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  public static Vector<N3> visionStandardDevs = VecBuilder.fill(0.5, 0.5, 9999999);

  private SwerveDrivePoseEstimator poseEstimator;
 public Swerve_Subsystem drivetrain = TunerConstants.createDrivetrain();
  private Field2d field2d;
  private LimelightHelpers.PoseEstimate mt1;
  private final Pigeon2 gyro = new Pigeon2(0);
  public Translation2d[] moduleTranslation2ds = {
    //in meters
    new Translation2d(0.27305, -0.27305),
    new Translation2d(0.27305, 0.27305),
    new Translation2d(-0.27305, 0.27305),
    new Translation2d(0.27305, 0.27305)
  };

  /** Creates a new PoseEstimator_Subsystem. */
  public PoseEstimator_Subsystem() {
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    this.drivetrain = drivetrain;
    this.gyro = gyro;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.getModuleTranslations()),
            gyro.getYaw(),
            drivetrain.getSwerveModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()),
            stateStandardDevs,
            visionStandardDevs);

    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
