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


  //try making I different for x and y controllers
  // private final PIDController pidControllerX = new PIDController(.35, 0.1, 0); //original p: .35 i: .1 d: 0
  // private final PIDController pidControllerY = new PIDController(.02, 0.1, 0); //original p: .2 i: .05 d: 0
  // private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private final PIDController pidControllerX = new PIDController(.35, 0.0005, .0000095); //original p: .35 i: .0005 d: 0.00005
  private final PIDController pidControllerY = new PIDController(.2, 0.0005, .0000095); //original p: .2 i: .0005 d: 0.00005
  private final PIDController pidControllerOmega = new PIDController(.06, .0005, 0.0000095); //original p: .05 i:.01 d: .0


  private double xErrorBound = .1;
  private double yErrorBound = .1;
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

      //if there are any visible targets 
      if (limelight.getHasTargets()){

        //PID setpoint for robot to be 1 meters away from the tag in the x direction
        double xSpeed = pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -.85);

        // //PID setpoint for robot to be 0 meters away from the tag in the y direction
        double ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);

          //PID setpoint for the robot to be 0 degrees away from the apriltag
        double omegaSpeed = pidControllerOmega.calculate(limelight.getH_angle(), 0);

  
      


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

        //try changing the signs here
        drivetrain.setControl(robotCentric.withVelocityX(xSpeed).withVelocityY(-ySpeed).withRotationalRate(omegaSpeed));
      
        //if the bot is at the position right in front of the reef, stop running the pid (other if statement runs)
        if (Math.abs(pidControllerX.getError()) < xErrorBound && Math.abs(pidControllerY.getError()) < yErrorBound && Math.abs(pidControllerOmega.getError()) < omegaErrorBound){
          middleLinedUp = true;
          System.out.println("Middle align is true");
        }    
     
    
    } 
    
      // System.out.println(pidControllerOmega.getError());
      // System.out.println("X error " + pidControllerX.getError());
      // System.out.println("Y error " + pidControllerY.getError());
      

   
   
   
    } else {
      
      
      //once the bot is center aligned with the tag, move bot either left or right directly to the reef
      if (!(Math.abs(pidControllerX.getError()) < xErrorBound && Math.abs(pidControllerY.getError()) < yErrorBound && Math.abs(pidControllerOmega.getError()) < omegaErrorBound) && limelight.getHasTargets()){
      middleLinedUp = false;
      counter = 0;
      System.out.println("Middle align is false, back to align");  
      }
      
      counter++;

      if (direction == "Right" && counter < Constants.numSeconds(.4)){
          drivetrain.setControl(robotCentric.withVelocityX(.45).withVelocityY(-.15).withRotationalRate(0));
          // System.out.println("Running right");
      } else if (direction == "Left" && counter < Constants.numSeconds(.4)){
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

    middleLinedUp = false;
    counter = 0;
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
