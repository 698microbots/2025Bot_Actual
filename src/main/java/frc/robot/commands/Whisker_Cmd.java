// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator_subsystem;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.Whisker_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Whisker_Cmd extends Command {
  // create 2 limit switch objects
  private Whisker_Subsystem whisker;
  private Swerve_Subsystem drivetrain;
  private Supplier<Double> xspeed;
  private Supplier<Double> yspeed;
  private Supplier<Double> rotateSpeed;
  private Elevator_subsystem elevator;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private String direction;
  /** Creates a new Whisker_Cmd. */
  public Whisker_Cmd(Whisker_Subsystem whisker, Swerve_Subsystem drivetrain, Supplier<Double> xspeed,
      Supplier<Double> yspeed, Supplier<Double> rotateSpeed, String direction, Elevator_subsystem elevator) {
    this.whisker = whisker;
    this.drivetrain = drivetrain;
    this.drivetrain = drivetrain;
    this.xspeed = xspeed;
    this.yspeed = yspeed;
    this.direction = direction;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(whisker,drivetrain,elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    elevator.setspeed(.09, 1);
    drivetrain.setControl(fieldCentric.withVelocityX(xspeed.get()*.1).withVelocityY(yspeed.get()*.1).withRotationalRate(rotateSpeed.get()*.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction.equals("left") && whisker.getLeftWhiskerClicked()){
      return true;
    } else {
      return false;
    }
  }
}
