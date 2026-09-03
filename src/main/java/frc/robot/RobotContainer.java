// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.WheelSpeeds;
import frc.robot.utils.*;

public class RobotContainer {

    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double speedMultiplier = 1;
    private double previousMultiplier = speedMultiplier;
    private double maxAngularRate = RotationsPerSecond.of(0.625).in(RadiansPerSecond); // 3/4 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.12)
            .withRotationalDeadband(maxAngularRate * 0.13) // Use an 11% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle pointAtAngle =
        new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(maxSpeed * 0.11) // Use an 11% deadband
        .withHeadingPID(5, 0, 0)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();
    public final Agitator agitator = new Agitator();
    public final Indexer indexer = new Indexer();
    public final Turret turret = new Turret();

    private final ComplexCommands complexCommands = new ComplexCommands(this);

    private SlewRateLimiter translationX = new SlewRateLimiter(12, -17, 0);
    private SlewRateLimiter translationY = new SlewRateLimiter(12, -17, 0);
    private final SlewRateLimiter rotation = new SlewRateLimiter(8, -12, 0);

    private CardinalDirections direction = CardinalDirections.UP;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        setupNamedCommands();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void setupNamedCommands() {

        NamedCommands.registerCommand("shoot", complexCommands.shoot(WheelSpeeds.FAR_SHOT, false));

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
                        .withRotationalRate(-rotation.calculate(driverController.getRightX() * maxAngularRate * speedMultiplier)) // Drive counterclockwise with negative X (left)
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

        driverController.y().whileTrue(complexCommands.shoot(WheelSpeeds.fromStaticAngularWheelVelocities(
            RotationsPerSecond.of(55),
            RotationsPerSecond.of(55)), false));
        //driverController.y().onTrue(complexCommands.lockTurretHeadingToHub());

        driverController.povDown().onTrue(Commands.runOnce(() -> direction = CardinalDirections.DOWN));
        driverController.povUp().onTrue(Commands.runOnce(() -> direction = CardinalDirections.UP));
        driverController.povLeft().onTrue(Commands.runOnce(() -> direction = CardinalDirections.LEFT));
        driverController.povRight().onTrue(Commands.runOnce(() -> direction = CardinalDirections.RIGHT));

        driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
                    pointAtAngle
                        .withVelocityX(-translationX.calculate(MathUtil.copyDirectionPow(driverController.getLeftY(), 3) * maxSpeed * speedMultiplier)) // Drive forward with negative Y (forward)
                        .withVelocityY(-translationY.calculate(MathUtil.copyDirectionPow(driverController.getLeftX(), 3) * maxSpeed * speedMultiplier)) // Drive left with negative X (left)
                        .withTargetDirection(Rotation2d.fromDegrees(direction.getDegrees().in(Degrees))) // Make the robot face where the right joystick is pointed
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.rightBumper().onTrue(drivetrain.runOnce(drivetrain::resetPose));
        driverController.rightTrigger().whileTrue(Commands.runOnce(() -> {
            previousMultiplier = speedMultiplier;
            speedMultiplier = 0.3;
        }));
        driverController.rightTrigger().whileFalse(Commands.runOnce(() -> speedMultiplier = previousMultiplier));

        complexCommands.lockTurretHeadingToHub();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void updateDashboard() {

        speedMultiplier = SmartDashboard.getNumber("Drive/Speed Multiplier", speedMultiplier);
        SmartDashboard.putNumber("Drive/Speed Multiplier", speedMultiplier);

    }
}
