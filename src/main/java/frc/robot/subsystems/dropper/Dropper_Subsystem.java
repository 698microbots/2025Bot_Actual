// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dropper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dropper_Subsystem extends SubsystemBase {
  /** Creates a new Dropper. */
  // SparkMax max = new SparkMax(1, MotorType.kBrushless);
  // SparkFlex flex = new SparkFlex(2, MotorType.kBrushless);
  // SparkClosedLoopController maxPid = max.getClosedLoopController();
  private final SparkMax dropperMotor = new SparkMax(Constants.dropperMotorID, MotorType.kBrushless); 
  public Dropper_Subsystem() {}

  public void dropCoral(){
    dropperMotor.set(.2);
  }

  public void driveUp(){
    dropperMotor.set(-.2);
  }

  public void stopDrive(){
    dropperMotor.set(0);
  }
  public void testDropCoral(double speed){
    dropperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
