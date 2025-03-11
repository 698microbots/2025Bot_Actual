// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.Whisker_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Whisker_Cmd extends Command {
  // create 2 limit switch objects
  Whisker_Subsystem whisker;
  private Swerve_Subsystem drivetrain;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();


  /** Creates a new Whisker_Cmd. */
  public Whisker_Cmd(Whisker_Subsystem whisker, Swerve_Subsystem drivetrain) {
    this.whisker = whisker;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(whisker);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (whisker.getDirection().equals("Right")) {
      // get the right LS
      if (whisker.getRightWhiskerClicked()) {
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
    } else {
      // get the left LS
      if (whisker.getLeftWhiskerClicked()) {
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
    }
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
