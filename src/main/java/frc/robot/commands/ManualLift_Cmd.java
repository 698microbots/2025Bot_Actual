// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualLift_Cmd extends Command {
  /** Creates a new ManualLift_Cmd. */
  private Elevator_subsystem elevator_subsystem;
  private Supplier<Double> x;
  public ManualLift_Cmd(Elevator_subsystem elevator_subsystem, Supplier<Double> x) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator_subsystem = elevator_subsystem;
    this.x = x;
    addRequirements(elevator_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator_subsystem.setspeed(x.get() * .25);
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
