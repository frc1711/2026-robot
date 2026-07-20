// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.*;

public class RobotContainer {

    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double speedMultiplier = 1;
    private double maxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.11)
            .withRotationalDeadband(maxAngularRate * 0.11) // Add an 11% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle pointAtAngle =
        new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(2.402, 0, 0)
        .withDeadband(maxSpeed * 0.11)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private JoystickSlewRateLimiter translationX = new JoystickSlewRateLimiter(0.35, 0.15);
    private JoystickSlewRateLimiter translationY = new JoystickSlewRateLimiter(0.35, 0.15);
    private final JoystickSlewRateLimiter rotation = new JoystickSlewRateLimiter(0.2, 0.05);

    private DPad direction = DPad.UP;

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
                        .withVelocityX(-translationX.calculate(driverController.getLeftY() * maxSpeed * speedMultiplier)) // Drive forward with negative Y (forward)
                        .withVelocityY(-translationY.calculate(driverController.getLeftX() * maxSpeed * speedMultiplier)) // Drive left with negative X (left)
                        .withRotationalRate(-rotation.calculate(driverController.getRightX() * maxAngularRate)) // Drive counterclockwise with negative X (left)
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

        driverController.povDown().onTrue(Commands.runOnce(() -> direction = DPad.DOWN));
        driverController.povUp().onTrue(Commands.runOnce(() -> direction = DPad.UP));
        driverController.povLeft().onTrue(Commands.runOnce(() -> direction = DPad.LEFT));
        driverController.povRight().onTrue(Commands.runOnce(() -> direction = DPad.RIGHT));

        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
                    pointAtAngle
                        .withVelocityX(-translationX.calculate(driverController.getLeftY() * maxSpeed)) // Drive forward with negative Y (forward)
                        .withVelocityY(-translationY.calculate(driverController.getLeftX() * maxSpeed)) // Drive left with negative X (left)
                        .withTargetDirection(Rotation2d.fromDegrees(direction.getDegrees())) // Make the robot face where the right joystick is pointed
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

    public void updateDashboard() {

        speedMultiplier = SmartDashboard.getNumber("Drive/Speed Multiplier", speedMultiplier);
        SmartDashboard.putNumber("Drive/Speed Multiplier", speedMultiplier);

    }
}
