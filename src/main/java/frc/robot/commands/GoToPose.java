package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inch;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve_Subsystem;

public class GoToPose {

    public static Command goToPose(
        Supplier<Pose2d> goalPoseSupplier, Translation2d tolerance, Swerve_Subsystem drive){
        return Commands.defer(() -> goToPose(goalPoseSupplier.get(), drive), Set.of(drive));
    }

    public static Command goToPose(Pose2d goalPose, Translation2d tolerance, Swerve_Subsystem drive) {
        SwerveRequest.FieldCentric fieldBase = new SwerveRequest.FieldCentric();

        // All controller constants are in terms of feet and degrees for 
        // easier intuitive understanding
        var xController = new PIDController(4, 0, 0.5); // TODO: Tune this since i have no clue how it'll translate
        xController.setSetpoint(goalPose.getMeasureX().in(Feet));
        xController.setTolerance(tolerance.getMeasureX().in(Feet), 1);

        var yController = new PIDController(4, 0, 0.5); // TODO: Also tune this cause im pretty sped
        yController.setSetpoint(goalPose.getMeasureY().in(Feet));
        yController.setTolerance(tolerance.getMeasureY().in(Feet), 1);

        PIDController thetaController = new PIDController(3.0, 0, 0); // TODO: uk the point, just tune allat for yall's robot
        thetaController.setTolerance(5.0, 5.0);
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setSetpoint(goalPose.getRotation().getDegrees());

        return Commands.startRun(
            () -> {
                xController.reset();
                yController.reset();
                thetaController.reset();
            },
            () -> {
                Pose2d currentPose = drive.getPose2d();

                LinearVelocity x = 
                    FeetPerSecond.of(xController.calculate(currentPose.getMeasureX().in(Feet)));
                LinearVelocity y =
                    FeetPerSecond.of(yController.calculate(currentPose.getMeasureY().in(Feet)));
                AngularVelocity theta =
                    DegreesPerSecond.of(
                        thetaController.calculate(currentPose.getRotation().getDegrees()));

                drive.setControl(fieldBase.withVelocityX(x).withVelocityY(y).withRotationalRate(theta));
            },
            drive)
            .until(
                () -> {
                    return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
                }
            )
            .andThen(
                Commands.runOnce(
                    () -> {
                        drive.setControl(fieldBase.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                    },
                    drive
                )
            )
            .withName(String.format("Go(%s)", goalPose.toString()));
    }

    public static Command goToPose(Pose2d goalPose, Swerve_Subsystem drive) {
        return goToPose(goalPose, new Translation2d(Inch.of(0.5), Inch.of(0.5)), drive);
    }

    
}
