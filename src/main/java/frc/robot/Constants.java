// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //test
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final int elevator_motor_1 = 10;
  public static final int elevator_motor_2 = 20;
  public static final int dropper_id = 30;
  public static final int lidar_id = 40;
  public static final int revId = 50;
  public static final int rotation_sensor = 60;
  public static final int joystick_1 = 7;
  public static final int joystick_2 = 8;
  public static final double l2 = 5; // change this to say like coralL2 to not get confused
  public static final double l3 = 6;
  public static final double l4 = 7;
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  
}
//hello