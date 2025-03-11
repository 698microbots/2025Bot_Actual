// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Slow_Cmd extends Command {
  private Supplier<Double> x;
  private Supplier<Double> y;
  private Supplier<Double> rotationalRate;
  private Swerve_Subsystem drivetrain;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  /** Creates a new Slow_Cmd. */
  public Slow_Cmd(Swerve_Subsystem drivetrain, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rotationalRate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.rotationalRate = rotationalRate;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(fieldCentric.withVelocityX(x.get() * 0.1).withVelocityY(y.get() * 0.1).withRotationalRate(rotationalRate.get() * 1.0));
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
