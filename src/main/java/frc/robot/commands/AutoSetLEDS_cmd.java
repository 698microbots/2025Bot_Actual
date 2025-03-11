// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Light_Subsystem;

public class AutoSetLEDS_cmd extends Command {
  /** Creates a new AutoSetLEDS. */
  // Animation animation = new Animation(0,2.00,0,8);
  private final Light_Subsystem lightSubsystem;
  public AutoSetLEDS_cmd(Light_Subsystem lightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lightSubsystem = lightSubsystem;
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("dem lights on");
    lightSubsystem.setLights(0, 255, 255);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // lightSubsystem.setLights(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
