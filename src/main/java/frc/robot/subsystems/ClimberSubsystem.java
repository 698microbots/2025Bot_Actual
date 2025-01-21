// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor = new TalonFX (Constants.climberMotorID);//setting the ID for motor
  //private double speed;
  public ClimberSubsystem() {
  
  }
public void motorSpeed(double speed) {//creating a method to control the motor 
  climberMotor.set(speed);// setting the speed of the motor
}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}