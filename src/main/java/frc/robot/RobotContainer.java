// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.JoystickSlewRateLimiter;

public class RobotContainer {

    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1 1/2 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.11)
            .withRotationalDeadband(MaxAngularRate * 0.11) // Add an 11% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle pointAtAngle =
        new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(0.013893, 0, 0);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private JoystickSlewRateLimiter translationX = new JoystickSlewRateLimiter(1, 0.5);
    private JoystickSlewRateLimiter translationY = new JoystickSlewRateLimiter(1, 0.5);
    private final JoystickSlewRateLimiter rotation = new JoystickSlewRateLimiter(0.8, 0.35);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-translationX.calculate(driverController.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                        .withVelocityY(-translationY.calculate(driverController.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                        .withRotationalRate(-rotation.calculate(driverController.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(
            drivetrain.applyRequest(() -> brake)
        );
        driverController.b().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(
                    new Rotation2d(
                        -driverController.getLeftY(),
                        -driverController.getLeftX()
                    )
                )
            )
        );
        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
                    pointAtAngle
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withTargetDirection(Rotation2d.kZero) // Make the robot face away from you
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain
                .applyRequest(() ->
                    drive
                        .withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ).withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
