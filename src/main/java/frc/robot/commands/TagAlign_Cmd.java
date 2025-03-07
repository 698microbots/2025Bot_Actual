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
  private int counter2 = 0;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  //pid and constants original
  // private final PIDController pidControllerX = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerY = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerOmega = new PIDController(.04, .01, 0);
  

  //try making I different for x and y controllers
  private final PIDController pidControllerX = new PIDController(.35, 0.1, 0); //original p: .35 i: .1 d: 0
  private final PIDController pidControllerY = new PIDController(.02, 0.1, 0); //original p: .2 i: .05 d: 0
  private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private double xErrorBound = .15;
  private double yErrorBound = .08;
  private double omegaErrorBound = .3;



  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;

  //if the bot is lined up center to the apriltag, before it moves left or right
  private boolean middleLinedUp = false;


  public TagAlign_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.direction = direction;
    addRequirements(limelight);
    addRequirements(drivetrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!middleLinedUp){
    //PID setpoint for the robot to be 0 degrees away from the apriltag
    double omegaSpeed = pidControllerOmega.calculate(limelight.getH_angle(), 0);

    //if there are any visible targets 
    if (!limelight.getHasTargets()){

      //counter will increment every 20ms camera does not see apriltag
      counter++;

      //once the counter has reached a certain time (.5 s), stop the robot
      if (counter > Constants.numSeconds(0)){
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
  
    } else {
      
      //reset the counter
      counter = 0;
      
      //if the robot sees any apriltag (might have to change settings to get closest apriltag), do calculations

      //PID setpoint for robot to be 1 meters away from the tag in the x direction
      double xSpeed = pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -.85);

      // //PID setpoint for robot to be 0 meters away from the tag in the y direction
      double ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);
      

      // Speed Monitor
      // System.out.println("xSpeed " + xSpeed);      
      // System.out.println("ySpeed " + ySpeed);
      // System.out.println("Rotational Rate" + omegaSpeed);
     
     
     


      //Error Threshold for X, Y, Omega
      if (Math.abs(pidControllerX.getError()) < xErrorBound) {
        xSpeed = 0;
      }

      if (Math.abs(pidControllerY.getError()) < yErrorBound){ //was .1
        ySpeed = 0;
      }

      if (Math.abs(pidControllerOmega.getError()) < omegaErrorBound){
        omegaSpeed = 0;
      }



      // xSpeed = 0;
      // ySpeed = 0
      // omegaSpeed = 0;
      drivetrain.setControl(robotCentric.withVelocityX(xSpeed).withVelocityY(-ySpeed).withRotationalRate(omegaSpeed));
     
      //if the bot is at the position right in front of the reef, stop running the pid (other if statement runs)
      if (Math.abs(pidControllerX.getError()) < xErrorBound && Math.abs(pidControllerY.getError()) < yErrorBound && Math.abs(pidControllerOmega.getError()) < omegaErrorBound){
        middleLinedUp = true;
        System.out.println("Middle align is true");
      }    
     
    
    } 
    
      System.out.println("Angle error " + pidControllerOmega.getError());
      System.out.println("X error " + pidControllerX.getError());
      System.out.println("Y error " + pidControllerY.getError());
      

   
   
   
    } else {
      
      
      //once the bot is center aligned with the tag, move bot either left or right directly to the reef
      if (!(Math.abs(pidControllerX.getError()) < xErrorBound && Math.abs(pidControllerY.getError()) < yErrorBound && Math.abs(pidControllerOmega.getError()) < omegaErrorBound) && limelight.getHasTargets()){
      middleLinedUp = false;
      counter2 = 0;
      System.out.println("Middle align is false, back t'o align");      
      }

      counter2++;

        if (direction == "Right" && counter2 < Constants.numSeconds(.4)){
          drivetrain.setControl(robotCentric.withVelocityX(.45).withVelocityY(-.15).withRotationalRate(0));
          // System.out.println("Running right");
        } else if (direction == "Left" && counter2 < Constants.numSeconds(.4)){
          drivetrain.setControl(robotCentric.withVelocityX(.45).withVelocityY(.15).withRotationalRate(0));
          // System.out.println("Running left");
        } else {
          drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
          // System.out.println("Stopping");
        }


    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < 4; i++){
      System.out.println(" ");   
    }
    middleLinedUp = false;
    counter2 = 0;
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
