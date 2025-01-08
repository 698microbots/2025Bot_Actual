// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class dropperFunnel extends SubsystemBase {
  public TalonFX dropper_motor = new TalonFX(Constants.dropper_id);
  /** Creates a new dropperFunnel. */
  public dropperFunnel() {}

  public void speed(double speed) {
    dropper_motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
