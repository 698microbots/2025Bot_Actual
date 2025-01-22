// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.l1_lift_command;
import frc.robot.subsystems.Dropper;
import frc.robot.subsystems.Elevator_subsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.light_subsystem;
import frc.robot.subsystems.sensor_subsystem;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController joystick_1 = new CommandXboxController(Constants.joystick_1);
  private final CommandXboxController joystick_2 = new CommandXboxController(Constants.joystick_2);
  public Dropper dropper = new Dropper();
  public Elevator_subsystem elevator = new Elevator_subsystem();
  public light_subsystem light = new light_subsystem();
  public sensor_subsystem sensor = new sensor_subsystem();
  public LimeLightSubsystem limelight = new LimeLightSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Dropper dropper, Elevator_subsystem elevator, light_subsystem light, sensor_subsystem sensor, LimeLightSubsystem limelight) {
    this.dropper = dropper;
    this.elevator = elevator;
    this.light = light;
    this.sensor = sensor;
    this.limelight = limelight;
    // Configure the trigger bindings
    configureBindings();
  }
  public RobotContainer() {}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    joystick_1.b().whileTrue(new l1_lift_command(elevator, dropper,l1_lift_command.l2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
