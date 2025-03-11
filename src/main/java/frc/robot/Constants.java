// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsOLD;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    public static double numSeconds(double seconds){ //does the calculations for how many 20ms are in a second, compare it to a counter adds every 20ms
      return seconds * 50;
  }  
  public static final int elevator_motor_1 = 15;
  public static final int elevator_motor_2 = 14;
  public static final int lidar_id = 0;
  public static final int dropperMotorID = 17;
  public static final int boreEncoderId = 0;
  public static final int joystick_1 = 0;
  public static final int joystick_2 = 1;
  public static final int joystick_3 = 2;

  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top speed
  public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                                // second max angular
                                                                                                // velocity
  public static final int toplimitSwitchID = 0;
  public static final int bottomlimitSwitchID = 1;

  public static final Transform3d robotToCam = new Transform3d(
    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(2)),
    new Rotation3d(0, 0, 0)
  );
  // public static final RobotConfig ROBOT_CONFIG = RobotConfig.fromGUISettings();
}
