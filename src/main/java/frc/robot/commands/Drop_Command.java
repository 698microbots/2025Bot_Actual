// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dropper_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drop_Command extends Command {
  /** Creates a new DropCoral. */
  private Dropper_Subsystem dropperSubsystem;
  private int counter = 0;
<<<<<<< HEAD:src/main/java/frc/robot/commands/Drop_Cmd.java
  public Drop_Cmd(Dropper_Subsystem dropperSubsystem) {
=======
  public Drop_Command(Dropper_Subsystem dropper) {
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/Drop_Command.java
    // Use addRequirements() here to declare subsystem dependencies.\
    this.dropperSubsystem = dropper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/Drop_Cmd.java
    counter++;
    // if (counter < Constants.numSeconds(2.5)){
    //   dropperSubsystem.testDropCoral(-1);
    // } else {
    //   dropperSubsystem.stopDrive();
    // }

    dropperSubsystem.testDropCoral(1);

=======
    dropperSubsystem.dropCoral();
    counter++;
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/Drop_Command.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dropperSubsystem.stopDrive();
    counter = 0;    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/Drop_Cmd.java
    if(counter > Constants.numSeconds(1)){
      return true;
    }
    return false;
=======
    if (counter >= 5){
      return true;
    } else {
      return false;
    }
    
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd:src/main/java/frc/robot/commands/Drop_Command.java
  }
}
