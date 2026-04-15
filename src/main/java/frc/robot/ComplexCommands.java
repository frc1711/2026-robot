package frc.robot;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.Direction;
import frc.robot.state.IntakePosition;
import frc.robot.subsystems.*;
import frc.robot.util.ChassisSpeedsSupplierBuilder;
import frc.robot.util.VirtualField;

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
    
    public Command shoot(
        Turret.WheelSpeeds turretState,
        Time spinupWaitTime,
        boolean withLock
    ) {
        
        Command enableLock = !withLock
            ? new InstantCommand()
            : this.swerve.enablePOIHeadingAndRadiusLock(
                VirtualField.getHubCenterPoint(),
                Feet.of(9),
                Direction.RIGHT
            );
        Command spinUpShooter = this.robot.turret.commands.shoot(turretState);
        Command pulse = this.robot.intake.commands.pulseV3();
        Command waitForLocks = !withLock
            ? new InstantCommand()
            : Commands.waitUntil(() -> this.robot.swerve.headingLock.hasLock(Degrees.of(3)))
                .alongWith(Commands.waitUntil(() -> this.robot.swerve.radiusLock.hasLock(Feet.of(1))));
        Command waitUntilReady = Commands.waitTime(spinupWaitTime)
            .alongWith(waitForLocks);
        Command feedShooter = this.indexer.forward();
        
        return Commands.parallel(
            enableLock,
            spinUpShooter,
            pulse,
            waitUntilReady.andThen(feedShooter)
        );
        
    }
    
    public Command shoot(
        Turret.WheelSpeeds turretState,
        boolean withLocks
    ) {
        
        return this.shoot(turretState, Seconds.of(0.5), withLocks);
        
    }
    
    public Command shoot() {
        
        return this.shoot(Turret.WheelSpeeds.MID_SHOT, true);
        
    }
    
}
