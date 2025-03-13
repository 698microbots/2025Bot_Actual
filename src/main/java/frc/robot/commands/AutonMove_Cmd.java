// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonMove_Cmd extends Command {
  /** Creates a new AutonMove_Cmd. */
  private double counter = 0;
  private double seconds, xSpeed, ySpeed, rotSpeed;
  private Swerve_Subsystem swerve;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  public AutonMove_Cmd(Swerve_Subsystem swerve, double seconds, double xSpeed, double ySpeed, double rotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.seconds = seconds;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    swerve.setControl(fieldCentric.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > Constants.numSeconds(seconds)){
      return true;
    } else {
      return false;
    }
  }
}
