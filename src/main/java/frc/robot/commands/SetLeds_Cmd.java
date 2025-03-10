// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.channels.ReadPendingException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReactedLED_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLeds_Cmd extends Command {
  /** Creates a new SetLeds_Cmd. */
  private int counter = 0;
  private final ReactedLED_Subsystem reactedLeds;
  public SetLeds_Cmd(ReactedLED_Subsystem reactedLeds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.reactedLeds = reactedLeds;
    addRequirements(reactedLeds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reactedLeds.turnOnLEDs(true);
    // reactedLeds.clearLEDAnimation();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    reactedLeds.rainbowLED();
    reactedLeds.setColor(255, 0, 20);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reactedLeds.turnOnLEDs(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
