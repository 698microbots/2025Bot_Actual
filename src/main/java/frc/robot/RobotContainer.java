// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoRotate_Cmd;
import frc.robot.commands.AutoSetLEDS_cmd;
import frc.robot.commands.Drop_Cmd;
import frc.robot.commands.ElevatorDown_Cmd;
import frc.robot.commands.ElevatorLiftWhisker_Cmd;
import frc.robot.commands.ExElevator;
import frc.robot.commands.Slow_Cmd;
import frc.robot.commands.ElevatorLift_Cmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GeneratePath_Cmd;
import frc.robot.commands.ManualLift_Cmd;
import frc.robot.commands.SetLeds_Cmd;
import frc.robot.commands.TagAlignFinal_Cmd;
import frc.robot.commands.Slow_Cmd;
import frc.robot.commands.TagAlignTest_Cmd;
import frc.robot.commands.TagAlign_Cmd;
import frc.robot.commands.Whisker_Cmd;
import frc.robot.commands.testReleaseCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsOLD;
import frc.robot.subsystems.Swerve_Subsystem;
import frc.robot.subsystems.Whisker_Subsystem;
import frc.robot.subsystems.Dropper_Subsystem;
import frc.robot.subsystems.Elevator_subsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Light_Subsystem;
import frc.robot.subsystems.LimeLight_Subsystem;
import frc.robot.subsystems.ReactedLED_Subsystem;

import com.ctre.phoenix.Logger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1); // Add a 10%
                                                                                                      // deadband
  // Use open-loop control for drive motors

  private final CommandXboxController joystick_1 = new CommandXboxController(Constants.joystick_1);
  private final CommandXboxController joystick_2 = new CommandXboxController(Constants.joystick_2);
  private final CommandXboxController joystick_TEST = new CommandXboxController(2);

  public Dropper_Subsystem dropper = new Dropper_Subsystem();
  public Elevator_subsystem elevator = new Elevator_subsystem();
  public LimeLight_Subsystem limelight = new LimeLight_Subsystem();
  public Swerve_Subsystem drivetrain = TunerConstants.createDrivetrain();
  public ReactedLED_Subsystem reactedLeds = new ReactedLED_Subsystem();
  public Light_Subsystem lights = new Light_Subsystem();
  public Whisker_Subsystem whisker = new Whisker_Subsystem();
  /* Path follower */
  private SendableChooser<Command> autoChooser;

  private final Field2d field;

  public RobotContainer() {

    // Register Named Commands
    // NamedCommands.registerCommand("autoBalance",
    // drivetrain.autoBalanceCommand());
    // NamedCommands.registerCommand("exampleCommand",
    // exampleSubsystem.exampleCommand());
    // NamedCommands.registerCommand("dropCommand", Commands.runOnce(()-> new Drop_Cmd(dropper)));

    // NamedCommands.registerCommand("alignToTag", new TagAlign_Cmd(limelight, drivetrain, "Right"));
    NamedCommands.registerCommand("elevatorDown", new ElevatorDown_Cmd(elevator));
    NamedCommands.registerCommand("dropCommand", new Drop_Cmd(dropper));
    NamedCommands.registerCommand("RaiseElevatorL2", new ElevatorLift_Cmd(elevator, dropper, 2, true));
    NamedCommands.registerCommand("RaiseElevatorL3", new ElevatorLift_Cmd(elevator, dropper, 3, true));
    NamedCommands.registerCommand("RaiseElevatorL4", new ElevatorLift_Cmd(elevator, dropper, 4, true));


    //
    // MOVED THIS DOWN BELOW THE NamedCommands REGISTRATIONS
    // SUPPOSED TO FIX NamedCommands ERROR WE WERE GETTING
    //
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // code for PathPlanner logging

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the\
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    reactedLeds.setDefaultCommand(new SetLeds_Cmd(reactedLeds));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.


    //P2 manual dropper
    dropper.setDefaultCommand(new testReleaseCoral(dropper, () -> -joystick_2.getRightY()));
    //P2 manual elevator
    elevator.setDefaultCommand(new ManualLift_Cmd(elevator, () -> -joystick_2.getLeftY(), dropper));

    //P1 driver
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick_1.getLeftY() * Constants.MaxSpeed * .5) // Drive
                                                                                                            // forward
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // Y
                                                                                                            // (forward)
            .withVelocityY(-joystick_1.getLeftX() * Constants.MaxSpeed * .5) // Drive left with negative X (left)
            .withRotationalRate(-joystick_1.getRightX() * Constants.MaxAngularRate * .8) // Drive counterclockwise with
                                                                                         // negative X (left)
        ));

    //P1 reset the field-centric heading on left bumper press
    joystick_1.leftBumper().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //P1 Slowmode
    joystick_1.rightTrigger().whileTrue(new Slow_Cmd(drivetrain, () -> -joystick_1.getLeftY(), () -> -joystick_1.getLeftX(), () -> -joystick_1.getRightX()));

    //P2 drop button
    joystick_2.x().whileTrue(new Drop_Cmd(dropper));
    
    //P2 Auto Rotate
    // joystick_2.rightBumper().whileTrue(new AutoRotate_Cmd(drivetrain, () -> -joystick_1.getLeftY(), () -> -joystick_1.getLeftX(), limelight));

    //P2 auto leveling and drops
    joystick_2.a().whileTrue(new ElevatorLift_Cmd(elevator, dropper, 2, false));
    joystick_2.b().whileTrue(new ElevatorLift_Cmd(elevator, dropper, 3, false));
    joystick_2.y().whileTrue(new ElevatorLift_Cmd(elevator, dropper, 4, false));

    //P2 right whisker
    // joystick_2.a().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "right", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 2, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // ));

    // joystick_2.b().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "right", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 3, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // ));
    
    // joystick_2.y().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "right", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 4, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // )); 

    // //P2 left whisker
    // joystick_2.povDown().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "left", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 2, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // ));  

    // joystick_2.povRight().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "left", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 3, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // ));
  
    // joystick_2.povUp().whileTrue(new SequentialCommandGroup(
    //   new Whisker_Cmd(whisker, "left", elevator),
    //   new ElevatorLiftWhisker_Cmd(elevator, dropper, 4, drivetrain),
    //   new Drop_Cmd(dropper),
    //   new ElevatorDown_Cmd(elevator)
    // ));    



    // joystick_TEST.a().whileTrue(new ElevatorLift_Cmd(elevator, dropper, 2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
    // return new PathPlannerAuto("New Auto");
  }
}


