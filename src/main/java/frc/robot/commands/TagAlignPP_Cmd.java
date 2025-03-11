// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLight_Subsystem;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagAlignPP_Cmd extends SequentialCommandGroup {
  /** Creates a new TagAlignPP_Cmd. */
  private Swerve_Subsystem drivetrain;
  private LimeLight_Subsystem limelight;
  private Transform3d tagToGoal;
  public TagAlignPP_Cmd(Swerve_Subsystem drivetrain, LimeLight_Subsystem limelight, double frontOffsetInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    double frontOffset = frontOffsetInches;
    this.tagToGoal = new Transform3d(new Translation3d(Units.inchesToMeters(frontOffset), 0, 0), new Rotation3d(0, 0, Math.PI));
    addRequirements(drivetrain, limelight);
    addCommands(new DeferredCommand(() -> getCommand(), Set.of(drivetrain, limelight)),
      new InstantCommand(() -> drivetrain.stopModules()));    
  }

        public Command getCommand() {

                Pose2d robotPose2d = drivetrain.getPose2d();

                Pose3d robotPose3d = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0,
                                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));


                if (!limelight.getHasTargets()) {
                        return new InstantCommand();
                } else {
                        try {


                                // Get the transformation from the camera to the tag
                                Transform3d camToTarget = new Transform3d(
                                  limelight.getRelative3dBotPose(), 
                                  new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));

                                // Transform the robot's pose to find the tag's pose
                                Pose3d cameraPose = robotPose3d.transformBy(Constants.robotToCam);
                                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                                // Transform the tag's pose to set our goal
                                Pose2d goalPose = targetPose.transformBy(tagToGoal).toPose2d();

                                return AutoBuilder.pathfindToPose(goalPose, new PathConstraints(
                                                3.0, 2,
                                                Units.degreesToRadians(540), Units.degreesToRadians(720)), 0);

                        } catch (NullPointerException ex) {
                                ex.printStackTrace();
                                return new InstantCommand();
                        }

                }
        }

}
