// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drop_Cmd extends Command {
  /** Creates a new DropCoral. */
  private Dropper_Subsystem dropperSubsystem;
  private int counter = 0;
  public Drop_Cmd(Dropper_Subsystem dropperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.dropperSubsystem = dropperSubsystem;
    addRequirements(dropperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    if (counter < Constants.numSeconds(.75)){
      dropperSubsystem.dropCoral();

    } else {
      dropperSubsystem.stopDrive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dropperSubsystem.stopDrive();    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
