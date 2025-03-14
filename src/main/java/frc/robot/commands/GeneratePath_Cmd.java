// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GeneratePath_Cmd extends Command {

  LimeLight_Subsystem limeLightSubsystem;
  Swerve_Subsystem drivetrain;
  private double offset=0; // OLD: -0.85

  /** Creates a new GeneratePathAndNav. */
  public GeneratePath_Cmd(LimeLight_Subsystem limeLightSubsystem, Swerve_Subsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    addRequirements(limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the pose of the limelight
    Pose2d robotPose2d = limeLightSubsystem.getRelative3dBotPose().toPose2d();
    Pose2d aprilTagPose2d = limeLightSubsystem.getAprilTagPose3d().toPose2d();

    Pose2d finalAprilTagPose = new Pose2d(-aprilTagPose2d.getX() + offset, -aprilTagPose2d.getY(), aprilTagPose2d.getRotation());

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        robotPose2d , finalAprilTagPose);

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
                                                                                           // path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
    // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If
                                                           // using a differential drivetrain, the rotation will have no
                                                           // effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
