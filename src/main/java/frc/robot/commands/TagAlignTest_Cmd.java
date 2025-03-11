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
public class TagAlignTest_Cmd extends Command {

  private final CommandXboxController joystick_1 = new CommandXboxController(Constants.joystick_1);

  /** Creates a new LineUpToTag. */
  private int counter = 0;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  // pid and constants original
  // private final PIDController pidControllerX = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerY = new PIDController(.55, 0.1, 0);
  // private final PIDController pidControllerOmega = new PIDController(.04, .01,
  // 0);

  // pid on carpet worked well
  // private final PIDController pidControllerX = new PIDController(.35, 0.0005,
  // .0000095); //original p: .35 i: .0005 d: 0.00005
  // private final PIDController pidControllerY = new PIDController(.2, 0.0005,
  // .0000095); //original p: .2 i: .0005 d: 0.00005
  // private final PIDController pidControllerOmega = new PIDController(.06,
  // .0005, 0.0000095); //original p: .05 i:.01 d: .0

  // try making I different for x and y controllers
  private final PIDController pidControllerX = new PIDController(.35, 0.0, 0.1); // original p: .35 i: .0005 d: 0.00005
  private final PIDController pidControllerY = new PIDController(.01, 0.0, 0); // original p: .2 i: .0005 d: 0.00005
  // ORIGINAL2: .2, 0, .1
  private final PIDController pidControllerOmega = new PIDController(.06, .0005, 0.0); // original p: .05 i:.01 d: .0

  // 1) make the x I pid term very small
  // 2) take out both I parts of the pid
  // 3) make the p terms for x/y very small
  // 4) make the i term for omega very small
  // 5) tune tunerConstants PID on cart
  // 6) try using a PD controller (use derivative gains and add i term if it
  // doesnt reach what its supposed to) D > P >> I
  // there could be a problem with the p and i terms being so close in value
  // (right now specifically for omegaController)
  private double xErrorBound = 0.0;
  private double yErrorBound = 0.0;
  private double omegaErrorBound = 0;

  private LimeLight_Subsystem limelight;
  private Swerve_Subsystem drivetrain;
  private String direction;

  // if the bot is lined up center to the apriltag, before it moves left or right
  private boolean middleLinedUp = false;

  public TagAlignTest_Cmd(LimeLight_Subsystem limelight, Swerve_Subsystem drivetrain, String direction) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if there are any visible targets
    if (limelight.getHasTargets()) {

      // PID setpoint for robot to be 1 meters away from the tag in the x direction
      // double xSpeed =
      // pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -.85);
      double xSpeed = pidControllerX.calculate(limelight.getRelative3dBotPose().getZ(), -.85);

      // //PID setpoint for robot to be 0 meters away from the tag in the y direction
      double ySpeed = pidControllerY.calculate(limelight.getH_angle(), 0);

      // PID setpoint for the robot to be 0 degrees away from the apriltag
      double omegaSpeed = pidControllerOmega.calculate(limelight.getH_angle(), 0);

      // Monitor
      if (counter % 50 == 0) {
        // System.out.println(xSpeed);
        // System.out.println(ySpeed);
        // System.out.println(omegaSpeed);

        // System.out.println(pidControllerOmega.getError());
        // System.out.println(pidControllerX.getError());
        // System.out.println(pidControllerY.getError());

      }

      // Error Threshold for X, Y, Omega
      if (Math.abs(pidControllerX.getError()) < xErrorBound) {
        xSpeed = 0;
      }

      if (Math.abs(pidControllerY.getError()) < yErrorBound) { // was .1
        ySpeed = 0;
      }

      if (Math.abs(pidControllerOmega.getError()) < omegaErrorBound) {
        omegaSpeed = 0;
      }

      // xSpeed = 0;
      // ySpeed = 0
      // omegaSpeed = 0;

      Supplier<Double> xspeed = () -> joystick_1.getLeftX();
      Supplier<Double> yspeed = () -> joystick_1.getLeftY();

      drivetrain.setControl(
          robotCentric.withVelocityX(xspeed.get()).withVelocityY(yspeed.get()).withRotationalRate(omegaSpeed));
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
