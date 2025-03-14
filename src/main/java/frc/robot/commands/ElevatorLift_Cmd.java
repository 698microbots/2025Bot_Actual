// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;
import frc.robot.subsystems.Swerve_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorLift_Cmd extends Command {
  // private final PIDController pidController = new PIDController(.04, 0.00, .0);
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.06);
  // private final ProfiledPIDController pidController = new ProfiledPIDController(0.04, 0.003, 0, new Constraints(0.5, 2));
  private final Elevator_subsystem elevator;
  private final Dropper_Subsystem dropper;
  private double level = 0;
  private int counter = 0;
  private boolean auto;
  private double speed = 0;

  private double L4Limit = 7.9;
  private double L3Limit = 4.7;
  private double L2Limit = 2.7;
  /** Creates a new l1_lift_command. */
  public ElevatorLift_Cmd(Elevator_subsystem elevator, Dropper_Subsystem dropper, double level, boolean auto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.level = level;
    this.auto = auto;
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
    
    if (counter < Constants.numSeconds(1.5)){
      speed = slewRateLimiter.calculate(.3);
      if (level == 2){
        elevator.setspeed(speed, L2Limit);
      } else if (level == 3){
        elevator.setspeed(speed, L3Limit);
      } else if (level == 4){
        elevator.setspeed(speed, L4Limit);
      }      
    } else {
      if (level == 2){
        elevator.setspeed(.3, L2Limit);
      } else if (level == 3){
        elevator.setspeed(.3, L3Limit);
      } else if (level == 4){
        elevator.setspeed(.3, L4Limit);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    dropper.stopDrive();
    elevator.setspeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (auto){
      if (level == 2 && elevator.getPosition() >= L2Limit){
        return true;
      } else if (level == 3 && elevator.getPosition() >= L3Limit){
        return true;
      } else if (level == 4 && elevator.getPosition() >= L4Limit){
        return true;
      } else {
        return false;
      }
    } else {
    return false;
    }
  }
}