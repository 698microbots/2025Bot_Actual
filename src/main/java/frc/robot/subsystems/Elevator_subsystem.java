// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator_subsystem extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(Constants.elevator_motor_1);
  private TalonFX motor2 = new TalonFX(Constants.elevator_motor_2);
  private Encoder revEncoder = new Encoder(0,1);
  private DigitalInput bottomLimitSwitch = new DigitalInput(2);
  /** Creates a new slevator. */
  
  public Elevator_subsystem() {
    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
  }
  
  public void setspeed(double speed) {
    if(getPressed()){
      revEncoder.reset();
      System.out.print("position reset");
    }

    if(getPosition() < 1 && speed < 0){
     speed = 0;
    } else if (getPosition() > 7 && speed > 0){
      speed = 0;
    }

    if(getPressed()){
      speed = 0;
    }

    motor1.set(-speed);
    motor2.set(-speed); // without direction changes, pushing up on the joystick goes down
  }

  //public void setspeed(double speed) {
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

  //Encoder values
  // L2: 2.913
  // L3: 4.740
  // L4: 7.750
  // TODO - change upper limit value (8)

  public boolean getPressed(){
    return bottomLimitSwitch.get();
  }
 
  public double getPosition() {
    return revEncoder.get() * -0.001;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}



