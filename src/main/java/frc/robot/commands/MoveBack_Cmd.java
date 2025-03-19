// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveBack_Cmd extends Command {
  private Swerve_Subsystem drivetrain;
  private int counter = 0;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  /** Creates a new MoveBack_Cmd. */
  public MoveBack_Cmd(Swerve_Subsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    if(counter >= Constants.numSeconds(1)){
      drivetrain.setControl(robotCentric.withVelocityY(0).withVelocityX(-0.1));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= Constants.numSeconds(2)){
     return true;   
    }
    return false;
  }
}
