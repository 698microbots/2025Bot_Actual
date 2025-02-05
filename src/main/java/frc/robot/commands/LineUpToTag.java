// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpToTag extends Command {
  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  //pid and constants
  private final PIDController pidControllerX = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private LimeLightSubsystem limeLightSubsystem;
  private CommandSwerveDrivetrain drivetrain;
  public LineUpToTag(LimeLightSubsystem limeLightSubsystem, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //PID setpoint for the robot to be 0 degrees away from the apriltag
    double omegaSpeed = pidControllerOmega.calculate(limeLightSubsystem.getH_angle(), 0);

    //if sees not see apriltag (ID = -1) start a timer, else do pid calculations
    if (limeLightSubsystem.getAprilTagID() == -1){

      //counter will increment every 20ms camera does not see apriltag
      counter++;

      //once the counter has reached a certain time (.5 s), stop the robot
      if (counter > Constants.numSeconds(.5)){
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
  
    } else {
      
      //reset the counter
      counter = 0;
      
      //if the robot sees any apriltag (might have to change settings to get closest apriltag), do calculations

      //PID setpoint for robot to be 1.3 meters away from the tag in the x direction
      double xSpeed = pidControllerX.calculate(limeLightSubsystem.getRelative3dBotPose().getZ(), -1.3);
      //PID setpoint for robot to be 0 meters away from the tag in the y direction
      double ySpeed = pidControllerY.calculate(limeLightSubsystem.getRelative3dBotPose().getX(), 0);

      //set all the calculated speeds to the robot 
      drivetrain.setControl(robotCentric.withVelocityX(-xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
