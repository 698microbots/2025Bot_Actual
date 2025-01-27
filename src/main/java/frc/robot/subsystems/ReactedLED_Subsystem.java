// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ReactedLED_Subsystem extends SubsystemBase {

  /** Creates a new LED. */
  CANdle candle = new CANdle(0);
  CANrange CANrange = new CANrange(0);

  double distance = CANrange.getDistance().getValueAsDouble();
  
  public ReactedLED_Subsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; //set strip type RGB
    config.brightnessScalar = 0.5;
    CANrangeConfiguration configs = new CANrangeConfiguration();
    config.configAllSettings(config);
  }

  public double returnDistance(){
    return CANrange.getDistance().getValueAsDouble();
  }

  public void setBrightness() {
    candle.setLEDs(0,0,0);
  }

  public void setColor(int r, int g, int b) {
    candle.setLEDs(r,g,b);
  }

  /*public void setColorWithString(String chosenColor) {
    if (chosenColor == "Yellow") {
      candle.setLEDs(255,255,0);
    }else if (chosenColor == "Green") {
      candle.setLEDs(0,255,0);
    }else{
      candle.setLEDs(255,0,0);
    }
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
