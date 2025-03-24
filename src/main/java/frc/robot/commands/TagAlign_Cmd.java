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
import frc.robot.subsystems.Gyro_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagAlign_Cmd extends Command {


  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private double ySpeed = 0;
  private double angle = 0;
  private double rotSpeed = 0;
  private double rotAngle = 0;
  private double currentID = -1;

  private final PIDController pidControllerRotation = new PIDController(.005, 0.00, 0); // original p: .014 i: 0.0014 d: 0.00005


  private final PIDController pidControllerYL = new PIDController(.03, 0.0, 0); // original p: .014 i: 0.00 d: 0.0000

  private final PIDController pidControllerYR = new PIDController(.03, 0.0, 0); // original p: .014 i: 0.00 d: 0.0000

  private double yErrorBound = 0.0;

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;
  private Supplier<Double> x, omega;
  private Gyro_Subsystem gyro;
  

  public TagAlign_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction, Supplier<Double> x, Supplier<Double> omega, Gyro_Subsystem gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.direction = direction;
    this.x = x;
    this.omega = omega;
    this.gyro = gyro;
    addRequirements(limelight, drivetrain, gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotAngle = gyro.getYaw();
    currentID = limelight.getAprilTagID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotSpeed = pidControllerRotation.calculate(gyro.getYaw(), rotAngle);

    if (direction.equals("left")){

      //derivation
      angle = Math.atan2(.5+.25, -limelight.getRelative3dBotPose().getZ() ) * (180/Math.PI) + 3; // used + 2

      System.out.println("angle is " + angle);
      ySpeed = pidControllerYL.calculate(limelight.getH_angle(), angle);
      System.out.println("left Y " + ySpeed);



    } else if (direction.equals("right")){
      angle = Math.atan2(.5-.25, -limelight.getRelative3dBotPose().getZ() ) * (180/Math.PI) + 9; // + 9

      // System.out.println("angle is " + angle);
      ySpeed = pidControllerYR.calculate(limelight.getH_angle(), angle);
      // System.out.println("right Y " + ySpeed);
    }

    //if there are any visible targets 
      if (limelight.getHasTargets() && limelight.getAprilTagID() == currentID){

        
        // if (Math.abs(pidControllerY.getError()) < yErrorBound){ //was .1
        //   ySpeed = 0;
        //   xSpeed = -.5;
        // }


        //lets driver only control x direction PUT THE X.GET OUTSIDE THE APRILTAG IF STATEMENT SO THEY CAN STILL MOVE ROBOT CENTRIC 
        drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.4).withRotationalRate(rotSpeed));

        //lets driver control x direction and rotation
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.75).withRotationalRate(omega.get()*.8*.75*Math.PI));
       
        //automatically does x direction
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(xSpeed));
    } else {
        drivetrain.setControl(robotCentric.withVelocityX(x.get()*0.4).withVelocityY(0).withRotationalRate(rotSpeed));
    } 
    
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
