// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
<<<<<<< HEAD:src/main/java/frc/robot/commands/TagAlign_Cmd.java
public class TagAlign_Cmd extends Command {


=======
public class TagAlign_Command extends Command {
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/TagAlign_Command.java
  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private double ySpeed = 0;

<<<<<<< HEAD:src/main/java/frc/robot/commands/TagAlign_Cmd.java



  private final PIDController pidControllerY = new PIDController(.014, 0.00, 0); // original p: .014 i: 0.0014 d: 0.00005


  private double yErrorBound = 0.0;

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;
  private Supplier<Double> x, omega;


  public TagAlign_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction, Supplier<Double> x, Supplier<Double> omega) {
=======
  //pid and constants
  private final PIDController pidControllerX = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  public TagAlign_Command(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain) {
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/TagAlign_Command.java
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.direction = direction;
    this.x = x;
    this.omega = omega;
    addRequirements(limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //xSpeed = 0;
    if (direction.equals("left")){
      ySpeed = pidControllerY.calculate(limelight.getH_angle(), -2);
    } else if (direction.equals("right")){
      ySpeed = pidControllerY.calculate(limelight.getH_angle(), 2);
    }

    //if there are any visible targets 
<<<<<<< HEAD:src/main/java/frc/robot/commands/TagAlign_Cmd.java
      if (limelight.getHasTargets()){
=======
    if (limelight.getHasTargets()){
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/TagAlign_Command.java

        
        // if (Math.abs(pidControllerY.getError()) < yErrorBound){ //was .1
        //   ySpeed = 0;
        //   xSpeed = -.5;
        // }

        // System.out.println(ySpeed);

        //lets driver only control x direction
        drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.75));

        //lets driver control x direction and rotation
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.75).withRotationalRate(omega.get()*.8*.75*Math.PI));
       
        //automatically does x direction
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(xSpeed));
    } else {
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    } 
    

      

<<<<<<< HEAD:src/main/java/frc/robot/commands/TagAlign_Cmd.java
   
   
   
    
=======
      //PID setpoint for robot to be 1.3 meters away from the tag in the x direction
      double xSpeed = pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -1.3);
      //PID setpoint for robot to be 0 meters away from the tag in the y direction
      double ySpeed = pidControllerY.calculate(limelight.getRelative3dBotPose().getX(), 0);

      //set all the calculated speeds to the robot 
      drivetrain.setControl(robotCentric.withVelocityX(-xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
  }
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/TagAlign_Command.java
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    counter = 0;
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
