// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.CANrange;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  //example
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    public double returnDistance(){
      return CANrange.getDistance().getValueAsDouble();
    }

    public boolean visibleTarget(){
      if (limelight.getEntry("tv").getDouble(0) == 1){
        return true;
      } else{
        return false;
      }
    }

    public void setColorWithString(String chosenColor) {
      if (chosenColor == "Yellow") {
        CANdle.setLEDs(255,255,0, 0, 0, 50);
      }else if (chosenColor == "Green") {
        CANdle.setLEDs(0,255,0, 0, 0, 50);
      }else if (chosenColor == "Red"){
        CANdle.setLEDs(255,0,0, 0, 0, 50);
      }
    }

    if (returnDistance() < .25 && visibleTarget()) {
      setColorWithString("Red");
    } else if (returnDistance() < .25 ){
      setColorWithString("Yellow");
    } else {
      setColorWithString("Green");
    }

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
