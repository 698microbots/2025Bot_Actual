// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagAlignFinal_Cmd extends Command {
  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private final ArrayList<Double> Ypositions = new ArrayList<Double>();
  private double yDirection = 1;

  private double initalPositionY = 0;
  //pid and constants original
  // private final PIDController pidControllerX = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerY = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerOmega = new PIDController(.04, .01, 0);
  
  //pid on carpet worked well
  private final PIDController pidControllerX = new PIDController(.35, 0.0005, .0000095); //original p: .35 i: .0005 d: 0.00005
  // private final PIDController pidControllerY = new PIDController(.2, 0.0005, .0000095); //original p: .2 i: .0005 d: 0.00005
  private final PIDController pidControllerOmega = new PIDController(.06, .0005, 0.0000095); //original p: .05 i:.01 d: .0

  //try making I different for x and y controllers
  // private final PIDController pidControllerX = new PIDController(.45, 0.0005, .0000095); //original p: .35 i: .0005 d: 0.00005
  private final PIDController pidControllerY = new PIDController(.3, 0.0, 0); //original p: .3 i: .000 d: 0.0000
  // private final PIDController pidControllerOmega = new PIDController(.009, .005, 0.0000095); //original p: .05 i:.01 d: .0

  // pid on carpet worked well
  // private final PIDController pidControllerX = new PIDController(.35, 0.0005,
  // .0000095); //original p: .35 i: .0005 d: 0.00005
  // private final PIDController pidControllerY = new PIDController(.2, 0.0005,
  // .0000095); //original p: .2 i: .0005 d: 0.00005
  // private final PIDController pidControllerOmega = new PIDController(.06,
  // .0005, 0.0000095); //original p: .05 i:.01 d: .0  

  //1) make the x I pid term very small
  //2) take out both I parts of the pid
  //3) make the p terms for x/y very small
  //4) make the i term for omega very small
  //5) tune tunerConstants PID on cart
  //6) try using a PD controller (use derivative gains and add i term if it doesnt reach what its supposed to) D > P >> I
  // there could be a problem with the p and i terms being so close in value (right now specifically for omegaController)
  private double xErrorBound = 0.0;
  private double yErrorBound = 0;
  private double omegaErrorBound = 0;



  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;

  //if the bot is lined up center to the apriltag, before it moves left or right
  private boolean middleLinedUp = false;


  public TagAlignFinal_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.direction = direction;
    addRequirements(limelight);
    addRequirements(drivetrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalPositionY = limelight.getRelative3dBotPose().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = 0;
      //if there are any visible targets 
      if (limelight.getHasTargets()){

        if (direction == "left" && initalPositionY > 0){
          System.out.println(initalPositionY);
          ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), -.22);
        } else if (direction == "left"){
          ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), -.1);
        }

        if (direction == "right" && initalPositionY > 0){
          ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), -.1);
        } else if (direction == "right") {
          ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);
          // System.out.println(initalPositionY);
       
        }
        // double ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);
        //PID setpoint for robot to be 1 meters away from the tag in the x direction
        // //PID setpoint for robot to be 0 meters away from the tag in the y direction

          //PID setpoint for the robot to be 0 degrees away from the apriltag
        double omegaSpeed = pidControllerOmega.calculate(limelight.getH_angle(), 0);

        // Monitor
        if (counter % 50 == 0){
          
          // System.out.println(xSpeed);
          // System.out.println("y speed" + -ySpeed);
          // System.out.println(omegaSpeed);

          // System.out.println(pidControllerOmega.getError());
          // System.out.println(pidControllerX.getError());
          // System.out.println(pidControllerY.getError());

        }

      
      
      


        //Error Threshold for X, Y, Omega

        if (Math.abs(pidControllerY.getError()) < yErrorBound){ //was .1
          ySpeed = 0;
        }

        if (Math.abs(pidControllerOmega.getError()) < omegaErrorBound){
          omegaSpeed = 0;
        }




        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(-ySpeed).withRotationalRate(0));
    
    } else {
      drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(-0).withRotationalRate(0));

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
