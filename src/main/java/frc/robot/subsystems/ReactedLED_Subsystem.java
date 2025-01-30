// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ReactedLED_Subsystem extends SubsystemBase {

  /** Creates a new LED. */
  CANdle CANdle = new CANdle(0);
  CANrange CANrange = new CANrange(0);
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  

  
  public ReactedLED_Subsystem() {
    CANdleConfiguration configCANdle = new CANdleConfiguration();
    CANrangeConfiguration configCANrange = new CANrangeConfiguration();

    configCANdle.stripType = LEDStripType.RGB; //set strip type RGB
    configCANdle.brightnessScalar = 0.5;
    CANdle.configAllSettings(configCANdle);
    CANrange.getConfigurator().apply(configCANrange);
  }

  public double returnDistance(){
    return CANrange.getDistance().getValueAsDouble();
  }


  public void setColor(int r, int g, int b) {
    CANdle.setLEDs(r, g, b, 0, 0, 50);
  }

  public void setColorWithString(String chosenColor) {
    if (chosenColor == "Yellow") {
      CANdle.setLEDs(255,255,0, 0, 0, 50);
    }else if (chosenColor == "Green") {
      CANdle.setLEDs(0,255,0, 0, 0, 50);
    }else if (chosenColor == "Red"){
      CANdle.setLEDs(255,0,0, 0, 0, 50);
    }
  }

  public boolean visibleTarget(){
    if (limelight.getEntry("tv").getDouble(0) == 1){
      return true;
    } else{
      return false;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //this all might need to be put into a command
    if (returnDistance() < .25 && visibleTarget()) {
      setColorWithString("Red");
    } else if (returnDistance() < .25 ){
      setColorWithString("Yellow");
    } else {
      setColorWithString("Green");
     
    }
    
  }

}
