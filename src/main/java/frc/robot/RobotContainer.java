// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Drop_Cmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualLift_Cmd;
import frc.robot.commands.TagAlign_Cmd;
import frc.robot.commands.testReleaseCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsOLD;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight_Subsystem;
import frc.robot.subsystems.ReactedLED_Subsystem;

import com.ctre.phoenix.Logger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1); // Add a 10% deadband
// Use open-loop control for drive motors
  
  private final CommandXboxController joystick_1 = new CommandXboxController(Constants.joystick_1);
  private final CommandXboxController joystick_2 = new CommandXboxController(Constants.joystick_2);
  private final CommandXboxController joystick_3 = new CommandXboxController(Constants.joystick_3);

  
  public Dropper_Subsystem dropper = new Dropper_Subsystem();
  public Elevator_subsystem elevator = new Elevator_subsystem();
  public LimeLight_Subsystem limelight = new LimeLight_Subsystem();
  public Swerve_Subsystem drivetrain = TunerConstants.createDrivetrain();
  public ReactedLED_Subsystem reactedLeds = new ReactedLED_Subsystem();
    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
        // autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        // SmartDashboard.putData("Auto Mode", autoChooser);    

    configureBindings();
  }

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
    dropper.setDefaultCommand(new testReleaseCoral(dropper, () -> -joystick_2.getRightY()));

    elevator.setDefaultCommand(new ManualLift_Cmd(elevator, () -> -joystick_2.getLeftY()));

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-joystick_1.getLeftY() * Constants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystick_1.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick_1.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
  );
  
      // reset the field-centric heading on left bumper press
    joystick_1.leftBumper().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    joystick_1.b().whileTrue(new TagAlign_Cmd(limelight, drivetrain));

    joystick_2.a().whileTrue(new Drop_Cmd(dropper));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // return autoChooser.getSelected();
    return new PathPlannerAuto("New Auto");
  }
}
