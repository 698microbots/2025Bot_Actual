// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.dropper.Dropper_Subsystem;
import frc.robot.subsystems.elevator.Elevator_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualLift_Cmd extends Command {
  /** Creates a new ManualLift_Cmd. */
  private Elevator_subsystem elevator_subsystem;
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.13);
  private Supplier<Double> x;
  private double speed = 0;    

  private int counter = 0;
  public ManualLift_Cmd(Elevator_subsystem elevator_subsystem, Supplier<Double> x) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator_subsystem = elevator_subsystem;
    this.x = x;
    addRequirements(elevator_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;

    if (counter < Constants.numSeconds(1.25)){
      // System.out.println("slew rate running");
      speed = slewRateLimiter.calculate(x.get() * .41);
    }  else {
      speed = x.get() * .41;      
    }

    if (x.get() < 0 && speed > 0){
      speed = -speed;
    } 

    elevator_subsystem.setspeed(speed);
    // System.out.println(speed);

    //WORKING FOR SURE
    // elevator_subsystem.setspeed(x.get()*.15);

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
    if (x.get() == 0){
      return true;
    }
    return false;
  }
}
