// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper;
import frc.robot.subsystems.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftCommand extends Command {
  private PIDController pidcontroller = new PIDController(.5, 0, 0);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.rotation_sensor);
  private Elevator_subsystem elevator = new Elevator_subsystem();
  double output = 0;
  private double level = 0;
  /** Creates a new l1_lift_command. */
  public LiftCommand(Elevator_subsystem elevator, Dropper dropper, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.level = level;
    addRequirements(elevator,dropper);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = pidcontroller.calculate(encoder.get(), level);
    elevator.setspeed(output);
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
