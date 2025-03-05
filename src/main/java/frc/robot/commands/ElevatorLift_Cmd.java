// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorLift_Cmd extends Command {
  private final PIDController pidcontroller = new PIDController(.06, 0.003, 0);
  private final Elevator_subsystem elevator;
  private double level = 0;
  private double output = 0;
  /** Creates a new l1_lift_command. */
  public ElevatorLift_Cmd(Elevator_subsystem elevator, Dropper_Subsystem dropper, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.level = level;
    addRequirements(elevator,dropper);
  }

  /*
  Encoder Values
  
  L2: 2.913
  L3: 4.74
  L4: 7.750
  
  */  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level == 2){
      output = pidcontroller.calculate(elevator.getPosition(), 2.913);

    } else if (level == 3){
      output = pidcontroller.calculate(elevator.getPosition(), 4.74);

    } else if (level == 4){
     output = pidcontroller.calculate(elevator.getPosition(), 7.750);

    }
    // System.out.println(output);
    elevator.setspeed(output);
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
