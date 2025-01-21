// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber_subsystem extends SubsystemBase {
  
  /** Creates a new climber_subsystem. */
  public climber_subsystem() {}
  public TalonFX motor1 = new TalonFX(Constants.elevator_motor_1);
  public TalonFX motor2 = new TalonFX(Constants.elevator_motor_2);
  
  public void moveMotors(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
