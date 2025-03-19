// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLight_Subsystem;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTag_Cmd extends Command {
  /** Creates a new AprilTag_Cmd. */
  private Swerve_Subsystem swerve;
  private LimeLight_Subsystem limelight;
  private String direction;
  private Command pathCommand;
  private PathConstraints constraints = new PathConstraints(.3, .3, 2 * Math.PI, 4 * Math.PI); // The constraints for this

  public AprilTag_Cmd(Swerve_Subsystem swerve, LimeLight_Subsystem limelight, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.limelight = limelight;
    this.direction = direction;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose2d = limelight.getRelative3dBotPose().toPose2d();

    if (direction.equals("left")){
    
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        startingPose2d,//Pose2d(current.get().getX(), current.get().getY(), new Rotation2d(0))
        new Pose2d(-.7,0, new Rotation2d(0)));

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                // be null for on-the-fly paths.
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If
                                                            // using a differential drivetrain, the rotation will have no
                                                            // effect.
      );

        path.preventFlipping = true;

        pathCommand = AutoBuilder.pathfindThenFollowPath(path, Constants.PathplannerConstants.constraints);
    
      } else if (direction.equals("right")){
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
       startingPose2d,//Pose2d(current.get().getX(), current.get().getY(), new Rotation2d(0))
       new Pose2d(-.7,0, new Rotation2d(0)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
