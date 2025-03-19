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
public class TagAlign_Cmd extends Command {


  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private double ySpeed = 0;
  private double angle = 0;



  private final PIDController pidControllerYL = new PIDController(.01, 0.00, 0); // original p: .014 i: 0.0014 d: 0.00005

  private final PIDController pidControllerYR = new PIDController(.01, 0.00, 0); // original p: .014 i: 0.0014 d: 0.00005

  private double yErrorBound = 0.0;

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;
  private Supplier<Double> x, omega;


  public TagAlign_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction, Supplier<Double> x, Supplier<Double> omega) {
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


      //left eq based on z pose (make z pose positive): y = -68.555x + 131.1
      // angle = -68.555 * limelight.getRelative3dBotPose().getZ() * -1 + 152;

      //derivation
      angle = Math.atan2(.5+.25, -limelight.getRelative3dBotPose().getZ()) * (180/Math.PI) -2.5 ;

      System.out.println("angle is " + angle);
      ySpeed = pidControllerYL.calculate(limelight.getH_angle(), angle);
      System.out.println("left Y " + ySpeed);



    } else if (direction.equals("right")){
      angle = Math.atan2(.5-.25, -limelight.getRelative3dBotPose().getZ()) * (180/Math.PI) -1;

      System.out.println("angle is " + angle);
      ySpeed = pidControllerYR.calculate(limelight.getH_angle(), angle);
      System.out.println("right Y " + ySpeed);    }

    //if there are any visible targets 
      if (limelight.getHasTargets()){

        
        // if (Math.abs(pidControllerY.getError()) < yErrorBound){ //was .1
        //   ySpeed = 0;
        //   xSpeed = -.5;
        // }

        System.out.println(x.get());

        //lets driver only control x direction PUT THE X.GET OUTSIDE THE APRILTAG IF STATEMENT SO THEY CAN STILL MOVE ROBOT CENTRIC 
        drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.12));

        //lets driver control x direction and rotation
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(x.get()*0.75).withRotationalRate(omega.get()*.8*.75*Math.PI));
       
        //automatically does x direction
        // drivetrain.setControl(robotCentric.withVelocityY(ySpeed).withVelocityX(xSpeed));
    } else {
        drivetrain.setControl(robotCentric.withVelocityX(x.get()*0.12).withVelocityY(0).withRotationalRate(0));
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
