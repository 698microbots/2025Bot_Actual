// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualLift_Cmd extends Command {
  /** Creates a new ManualLift_Cmd. */
  private Elevator_subsystem elevator_subsystem;
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.06);
  private Supplier<Double> x;
  private double speed = 0;    
  private Dropper_Subsystem dropper;

  private int counter = 0;
  public ManualLift_Cmd(Elevator_subsystem elevator_subsystem, Supplier<Double> x, Dropper_Subsystem dropper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator_subsystem = elevator_subsystem;
    this.dropper = dropper;
    this.x = x;
    addRequirements(elevator_subsystem, dropper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    
    //this breaks things maybe need to comment
    if (counter < Constants.numSeconds(.75)){
      dropper.driveUp();  
    } else {
      dropper.stopDrive();
    }

    if (counter > Constants.numSeconds(1.5)){
      speed = slewRateLimiter.calculate(x.get() * .3);
    } else {
      speed = x.get() * .3;
    }
    elevator_subsystem.setspeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    elevator_subsystem.setspeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
