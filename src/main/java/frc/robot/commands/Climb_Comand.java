// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb_Comand extends Command {
  /** Creates a new RobotClimber. */
  private Climber_Subsystem climber = new Climber_Subsystem();//making an object of subsystem 
  public Climb_Comand(Climber_Subsystem climber) {
      this.climber = climber;
      addRequirements(climber);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.motorSpeed(.1);// when a is pressed, it runs this code
  }
  
  public void climberDown() {

  }

      // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.motorSpeed(0); // when a is pressed, it runs this code

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
