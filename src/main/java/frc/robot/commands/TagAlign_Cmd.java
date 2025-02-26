// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagAlign_Cmd extends Command {
  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  //pid and constants
  private final PIDController pidControllerX = new PIDController(.3, 0.01, 0);
  private final PIDController pidControllerY = new PIDController(.03, 0.01, 0);
  private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  public TagAlign_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    addRequirements(limelight);
    addRequirements(drivetrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //PID setpoint for the robot to be 0 degrees away from the apriltag
    double omegaSpeed = pidControllerOmega.calculate(limelight.getH_angle(), 0);

    //if there are any visible targets 
    if (!limelight.getHasTargets()){

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
      double xSpeed = pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -1.3);
      // System.out.println("xSpeed " + xSpeed);
      // //PID setpoint for robot to be 0 meters away from the tag in the y direction
      double ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);
      // System.out.println("ySpeed " + ySpeed);
      System.out.println("Rotational Rate" + omegaSpeed);
      // //set all the calculated speeds to the robot 

      if (Math.abs(xSpeed) < .02) {
        xSpeed = 0;
      }

      if (Math.abs(ySpeed) < .02){
        ySpeed = 0;
      }

       ySpeed = 0;
       omegaSpeed = 0;      
      drivetrain.setControl(robotCentric.withVelocityX(xSpeed).withVelocityY(-ySpeed).withRotationalRate(omegaSpeed));
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
