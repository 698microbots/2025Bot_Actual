// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Whisker_Subsystem extends SubsystemBase {
  private DigitalInput leftWhisker = new DigitalInput(3);
  private String direction;

  /** Creates a new Whisker_Subsystem. */
  public Whisker_Subsystem(String direction) {
    this.direction = direction;
  }

  public Whisker_Subsystem() {
    direction = null;
  }

  public String getDirection() {
    return direction;
  }

  public boolean getLeftWhiskerClicked() {
    return leftWhisker.get();
  }

  // public boolean getRightWhiskerClicked() {
  //   return rightWhisker.get();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
