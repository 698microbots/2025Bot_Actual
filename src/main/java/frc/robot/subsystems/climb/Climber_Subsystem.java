// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber_Subsystem extends SubsystemBase {
  private final SparkMax climberMotor = new SparkMax(Constants.climberMotorID, MotorType.kBrushed); 
  //private double speed;
  public Climber_Subsystem() {
  }
public void motorSpeed(double speed) {//creating a method to control the motor 
  climberMotor.set(speed);// setting the speed of the motor
}  

  public double getClimbPosition(){
    return climberMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}