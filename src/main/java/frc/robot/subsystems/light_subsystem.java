// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;


public class light_subsystem extends SubsystemBase {
  /** Creates a new light_subsystem. */
  CANdle candle = new CANdle(0); // creates a new CANdle with ID 0

  public light_subsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  public void setBrightness() {
    candle.setLEDs(0,0,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
