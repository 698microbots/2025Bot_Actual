// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator_subsystem extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(Constants.elevator_motor_1);
  private TalonFX motor2 = new TalonFX(Constants.elevator_motor_2);
  // private DutyCycleEncoder revEncoder = new DutyCycleEncoder(Constants.boreEncoderId);
  private Encoder revEncoder = new Encoder(0, 1);
  private DigitalInput limitSwitch = new DigitalInput(2);
  /** Creates a new slevator. */
  
  public Elevator_subsystem() {
    // motor1.setNeutralMode(NeutralModeValue.Coast);
    // motor2.setNeutralMode(NeutralModeValue.Coast);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }



  public void setspeed(double speed) {
    //Max Encoder: 8.077

    
    if (getPosition() < .3 && speed < 0){
      speed = 0;
    } else if (getPosition() > 7.7 && speed > 0){
      speed = 0;
    }

    motor1.set(-speed); //RIGHT NOW POSITIE VALUES FED INTO THE PARAMETER GOES UP
    motor2.set(-speed); // without direction changes, pushing up on the joystick goes down   
  }

  // public void setspeed(double speed) {
  //   if (toplimitSwitch.get()) {
  //     motor1.set(0);
  //     motor2.set(0);
  //   } else {
  //     motor1.set(speed);
  //     motor2.set(speed);
  //   }
  //   if (bottomlimitSwitch.get()) {
  //     motor1.set(0);
  //     motor2.set(0);
  //   } else {
  //     motor1.set(speed);
  //     motor2.set(speed);
  //   }
 
  public double getPosition() {
    return revEncoder.get() * -.001;
  }

  public boolean getPressed(){
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getPressed()){
      revEncoder.reset();
    } 
  
  
  }



}



