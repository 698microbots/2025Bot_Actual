// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotClimber extends Command {
  /** Creates a new RobotClimber. */
private double speed; 
  ClimberSubsystem newRobotClimberSubsystem = new ClimberSubsystem();//making an object of subsystem 
  public RobotClimber(double speed) {
   this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//running the subsystem
    newRobotClimberSubsystem.motorSpeed(speed);// when a is pressed, it runs this code
  }
  
  public void climberDown() {

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
