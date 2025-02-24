// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator_subsystem extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(Constants.elevator_motor_1);
  private TalonFX motor2 = new TalonFX(Constants.elevator_motor_2);
  private DutyCycleEncoder revEncoder = new DutyCycleEncoder(Constants.boreEncoderId);
  /** Creates a new slevator. */
  public Elevator_subsystem() {}

  public void setspeed(double speed) {
    motor1.set(-speed);
    motor2.set(-speed); // without direction changes, pushing up on the joystick goes down
  }

  public double getPosition(){
    return revEncoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
