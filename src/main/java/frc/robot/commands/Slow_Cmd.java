// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Slow_Cmd extends Command {
  private Swerve_Subsystem drivetrain;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private Supplier<Double> xspeed;
  private Supplier<Double> yspeed;
  private Supplier<Double> rotateSpeed;

  /** Creates a new Slow_Cmd. */
  public Slow_Cmd(Swerve_Subsystem drivetrain, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotateSpeed) {
    this.drivetrain = drivetrain;
    this.xspeed = xspeed;
    this.yspeed = yspeed;
    this.rotateSpeed = rotateSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(fieldCentric.withVelocityX(xspeed.get()*.4).withVelocityY(yspeed.get()*.4).withRotationalRate(rotateSpeed.get()*Math.PI*.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
