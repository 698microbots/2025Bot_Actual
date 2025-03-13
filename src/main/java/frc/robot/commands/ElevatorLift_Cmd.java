// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorLift_Cmd extends Command {
  private final PIDController pidController = new PIDController(.04, 0.00, .0);
  private final SlewRateLimiter filter = new SlewRateLimiter(.01);
  // private final ProfiledPIDController pidController = new ProfiledPIDController(0.04, 0.003, 0, new Constraints(0.5, 2));
  private final Elevator_subsystem elevator;
  private final Dropper_Subsystem dropper;
  private double level = 0;
  private double output = 0;
  private int counter = 0;

  private double L4Limit = 8.05;
  private double L3Limit = 4.9;
  private double L2Limit = 3.1;
  /** Creates a new l1_lift_command. */
  public ElevatorLift_Cmd(Elevator_subsystem elevator, Dropper_Subsystem dropper, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.level = level;
    this.dropper = dropper;
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
    counter++;

    if (counter < Constants.numSeconds(.75)){
      dropper.driveUp();
    } else {
      dropper.stopDrive();
    }


    if (level == 2){
      // output = pidController.calculate(elevator.getPosition(), 3);
      // System.out.println(output);
      elevator.setspeed(.1, L2Limit);
    } else if (level == 3){
      // output = pidController.calculate(elevator.getPosition(), 4.85);
      // System.out.println(output);
      elevator.setspeed(.1, L3Limit);

    } else if (level == 4){
      elevator.setspeed(.1, L4Limit);

      // output = pidController.calculate(elevator.getPosition(), 7.85);
      // System.out.println(filter.calculate(output));
    }
    
    // if (Math.abs(output) > .1){
    //   output = Math.signum(output) * .1;
    // }

    // System.out.println(filter.calculate(output));
    // System.out.println(output);
    
    // elevator.setspeed(filter.calculate(output));
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    dropper.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (level == 2 && elevator.getPosition() >= L2Limit){
      return true;
    } else if (level == 3 && elevator.getPosition() >= L3Limit){
      return true;
    } else if (level == 4 && elevator.getPosition() >= L4Limit){
      return true;
    } else {
      return false;
    }
    // return false;
  }
}