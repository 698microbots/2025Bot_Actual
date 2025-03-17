// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LimeLight_Subsystem;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
<<<<<<< HEAD
    SmartDashboard.putNumber("Elevator Position", m_robotContainer.elevator.getPosition());
    SmartDashboard.putNumber("Sensor Distance", m_robotContainer.reactedLeds.returnDistance());
    // SmartDashboard.putNumber("speed", Constants.MaxSpeed);

    SmartDashboard.putNumber("Robot Pose X", m_robotContainer.limelight.getRelative3dBotPose().getX());
    SmartDashboard.putNumber("Robot Pose Z", m_robotContainer.limelight.getRelative3dBotPose().getZ());
    SmartDashboard.putBoolean("Has Target", m_robotContainer.limelight.getHasTargets());
    SmartDashboard.putNumber("Angle", m_robotContainer.limelight.getH_angle());

    SmartDashboard.putNumber("Module 0 Angle", m_robotContainer.drivetrain.getAngle(0));
    SmartDashboard.putNumber("Module 1 Angle", m_robotContainer.drivetrain.getAngle(1));
    SmartDashboard.putNumber("Module 2 Angle", m_robotContainer.drivetrain.getAngle(2));
    SmartDashboard.putNumber("Module 3 Angle", m_robotContainer.drivetrain.getAngle(3));
    SmartDashboard.putNumber("Yaw", m_robotContainer.limelight.getYaw());


    // SmartDashboard.putString("Rotation", kDefaultPeriod);
    SmartDashboard.putBoolean("limit switch", m_robotContainer.elevator.getPressed());


    // SmartDashboard.putNumber("yaw", m_robotContainer.limelight.getYaw());    
    // SmartDashboard.putNumber("pitch", m_robotContainer.limelight.getPitch());    
    SmartDashboard.putNumber("roll", m_robotContainer.limelight.getRoll());   

    SmartDashboard.putBoolean("elevator bottom pressed", m_robotContainer.elevator.getPressed());
    SmartDashboard.putBoolean("whisker left pressed", m_robotContainer.whisker.getLeftWhiskerClicked());
    SmartDashboard.putBoolean("whisker right pressed", m_robotContainer.whisker.getRightWhiskerClicked());
    // SmartDashboard.putBoolean("whisker right pressed", m_robotContainer.whisker.getRightWhiskerClicked());
    
    SmartDashboard.putNumber("ensimated Y position", (m_robotContainer.limelight.getRelative3dBotPose().getZ()/Math.tan(m_robotContainer.limelight.getYaw())));

    // 
    // SmartDashboard.putData("Robot Pose _ ", m_robotContainer.limelight.get)    
=======
>>>>>>> 832b341bd6bad1fb07078e6caef8d7c93646e5fd
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
// Hello World