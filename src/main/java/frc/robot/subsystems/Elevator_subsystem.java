// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator_subsystem extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(Constants.elevator_motor_1);
  private TalonFX motor2 = new TalonFX(Constants.elevator_motor_2);

  private TalonFXConfiguration motor1_config = new TalonFXConfiguration();
  private TalonFXConfiguration motor2_config = new TalonFXConfiguration();



  // private DutyCycleEncoder revEncoder = new
  // DutyCycleEncoder(Constants.boreEncoderId);
  private DigitalInput limitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);
  private Encoder revEncoder = new Encoder(0, 1);
  Slot0Configs slot0Configs1 = motor1_config.Slot0;
  Slot0Configs slot0Configs2 = motor2_config.Slot0;

  MotionMagicConfigs motionMagicConfigs1 = motor1_config.MotionMagic;
  MotionMagicConfigs motionMagicConfigs2 = motor2_config.MotionMagic;
  /** Creates a new slevator. */
  
  public Elevator_subsystem() {
    // motor1.setNeutralMode(NeutralModeValue.Coast);
    // motor2.setNeutralMode(NeutralModeValue.Coast);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
    
    slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs1.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs1.kI = 0; // no output for integrated error
    slot0Configs1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    slot0Configs2.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs2.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs2.kI = 0; // no output for integrated error
    slot0Configs2.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs1.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    motionMagicConfigs1.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    motionMagicConfigs1.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    motionMagicConfigs2.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    motionMagicConfigs2.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    motionMagicConfigs2.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    motor1.getConfigurator().apply(motor1_config);
    motor2.getConfigurator().apply(motor2_config);

    final MotionMagicExpoVoltage  m_request1 = new MotionMagicExpoVoltage(0);
    final MotionMagicExpoVoltage m_request2 = new MotionMagicExpoVoltage(0);    
    motor1.setControl(m_request1.withPosition(7.8));
    motor2.setControl(m_request2.withPosition(7.8));


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

  public void setSpeed(double speed, int  level) {
    if (level == 2 && getPosition() > 3) {
      speed = 0;
    } else if (level == 3 && getPosition() > 4.85) {
      speed = 0;
    } else if (level == 4 && getPosition() > 7.85) {
      speed = 0;
    }
  }

  public double getPosition() {
    return revEncoder.get() * -.001;
  }

  public boolean getPressed(){
    return limitSwitch.get();
  }

  // public boolean getPressed2(){
  //   return limitSwitchTop.get();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getPressed()){
      revEncoder.reset();
    } 
  }
}