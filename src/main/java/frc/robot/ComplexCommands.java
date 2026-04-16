package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.Direction;
import frc.robot.configuration.RobotDimensions;
import frc.robot.state.IntakePosition;
import frc.robot.subsystems.*;
import frc.robot.util.ChassisSpeedsSupplierBuilder;
import frc.robot.util.PoseBuilder;
import frc.robot.util.VirtualField;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ComplexCommands {
    
    protected final RobotContainer robot;
    
    protected final Swerve.Commands swerve;
    
    protected final Intake.Commands intake;
    
    protected final Agitator.Commands agitator;
    
    protected final Indexer.Commands indexer;
    
    protected final Turret.Commands turret;
    
    protected final Vision.Commands vision;

    public ComplexCommands(RobotContainer robotContainer) {
        
        this.robot = robotContainer;
        this.swerve = robotContainer.swerve.commands;
        this.intake = robotContainer.intake.commands;
        this.agitator = robotContainer.agitator.commands;
        this.indexer = robotContainer.indexer.commands;
        this.turret = robotContainer.turret.commands;
        this.vision = robotContainer.vision.commands;
        
    }
    
    public Command drive(CommandXboxController controller) {
        
        return this.swerve.drive(
            ChassisSpeedsSupplierBuilder.fromControllerJoysticks(controller)
                .withAdditional(ChassisSpeedsSupplierBuilder.fromControllerDPad(controller))
                .withFieldRelative(this.robot.swerve)
                .withSpeedMultiplierCheck(this.robot.swerve)
                .withHeadingLock(this.robot.swerve)
                .withRadiusLock(this.robot.swerve)
                .withMaxVelocityCheck()
                .withMaxAccelerationCheck()
        ).finallyDo(this.robot.swerve::stop);
        
    }
    
    public Command resetFieldHeading() {
        
        Command haltSwerve = this.swerve.useDriveSpeedMultiplier(0);
        Command beginCalibration = this.swerve.calibrateFieldRelativeHeading()
            .alongWith(this.vision.beginStableSeeding());
        Command endCalibration = this.vision.beginUsingInternalLL4IMUAssist();
        
        return haltSwerve
            .alongWith(beginCalibration)
            .withTimeout(Seconds.of(2))
            .andThen(endCalibration)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
        
    }
    
    public Command intake() {
        
        Command prepareAndRunIntake =
            this.intake.goToPosition(IntakePosition.INTAKING)
                .andThen(this.intake.intake(() -> 0.65));
        Runnable resetIntakePosition = () ->
            this.robot.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command outtake() {
        
        Command prepareAndRunIntake =
            this.intake.goToPosition(IntakePosition.INTAKING)
                .andThen(this.intake.intake(() -> -0.65));
        Runnable resetIntakePosition = () ->
            this.robot.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command lockTurretHeadingToHub() {
        
        Supplier<Angle> headingToHub = () -> {
            
            Translation2d turretOffset = new Translation2d(
                RobotDimensions.TURRET_X_OFFSET_FROM_ROBOT_CENTER,
                RobotDimensions.TURRET_Y_OFFSET_FROM_ROBOT_CENTER
            );
            Translation2d turretCenterPoint =
                PoseBuilder.fromPose(this.robot.swerve.getOdometry().getPose())
                    .withTranslation(PoseBuilder.CoordinateSystem.ROBOT_RELATIVE, turretOffset)
                    .get()
                    .getTranslation();
            Translation2d hubCenterPoint = VirtualField.getHubCenterPoint();
            Translation2d delta = hubCenterPoint.minus(turretCenterPoint);
            
            return delta.getAngle()
                .getMeasure()
                .minus(this.robot.swerve.getFieldRelativeHeading());
            
        };
        
        return new InstantCommand(
            () -> this.robot.turret.setHeadingSupplier(headingToHub)
        );
        
    }
    
    public Command shoot(
        Turret.WheelSpeeds turretState,
        Time spinupWaitTime,
        boolean withLock,
        boolean withPulse
    ) {
        
        Command enableLock = !withLock
            ? new InstantCommand()
            : this.swerve.enablePOIHeadingAndRadiusLock(
                VirtualField.getHubCenterPoint(),
                Feet.of(9),
                Direction.RIGHT
            );
        Command spinUpShooter = this.turret.shoot(turretState);
//        Command pulse = withPulse
//            ? this.intake.pulseV3()
//            : new InstantCommand();
        Command waitForLocks = !withLock
            ? new InstantCommand()
            : Commands.waitUntil(() -> this.robot.swerve.headingLock.hasLock(Degrees.of(3)))
                .alongWith(Commands.waitUntil(() -> this.robot.swerve.radiusLock.hasLock(Feet.of(1))));
        Command waitUntilReady = Commands.waitTime(spinupWaitTime)
            .alongWith(waitForLocks);
        Command feedShooter = this.indexer.forward()
            .alongWith(Commands.waitTime(Seconds.of(0.5)).andThen(this.agitator.spin()));
        
        return Commands.parallel(
            enableLock,
            spinUpShooter,
//            pulse,
            waitUntilReady.andThen(feedShooter)
        );
        
    }
    
    public Command shoot(
        Turret.WheelSpeeds turretState,
        boolean withLocks
    ) {
        
        return this.shoot(turretState, Seconds.of(0.5), withLocks, true);
        
    }
    
    public Command shoot() {
        
        return this.shoot(Turret.WheelSpeeds.MID_SHOT, true);
        
    }
    
}
