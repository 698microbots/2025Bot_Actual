// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */
  // private final CANdle candle = new CANdle(0);
  // private final CANdleConfiguration config = new CANdleConfiguration();

  public LightSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    // candle.configAllSettings(config);    
  }

  public void setLights(int r, int b, int g){
    // candle.setLEDs(r, g, b);
    // candle.setLEDs(r, g, b, 100, 0, 150); THERE ARE 300 LIGHTS
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
