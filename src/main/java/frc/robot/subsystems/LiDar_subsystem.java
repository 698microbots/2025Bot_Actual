// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiDar_subsystem extends SubsystemBase {
  /** Creates a new LiDar_subsystem. */
  private CANrange lidar_sensor = new CANrange(Constants.lidar_id);

  public LiDar_subsystem() {} 

  public boolean blocked() {
    boolean ifBlocked = false;
  StatusSignal<Double> signal = lidar_sensor.getAmbientSignal();
  if (signal.getValue() < 10000) {
    ifBlocked = true;
  }
    return ifBlocked;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
