package frc.robot;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.math.DoubleSupplierBuilder;
import frc.robot.state.IntakePosition;
import frc.robot.state.TurretWheelSpeeds;
import frc.robot.util.ChassisSpeedsSupplierBuilder;
import frc.robot.util.VirtualField;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class ComplexCommands {
    
    protected final RobotContainer robot;

    public ComplexCommands(RobotContainer robotContainer) {
        
        this.robot = robotContainer;
        
    }
    
    public Command drive(CommandXboxController controller) {
        
        DoubleSupplier rotationInput = DoubleSupplierBuilder.getRotationDoubleSupplier(controller);
        Trigger driverIsTryingToManuallyTurn = new Trigger(() -> rotationInput.getAsDouble() != 0);
        driverIsTryingToManuallyTurn.onTrue(this.robot.swerve.commands.disableHeadingLock());
        
        return this.robot.swerve.commands.drive(
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
        
        Command haltSwerve = this.robot.swerve.commands.useDriveSpeedMultiplier(0);
        Command beginCalibration = this.robot.swerve.commands.calibrateFieldRelativeHeading()
            .alongWith(this.robot.vision.commands.beginStableSeeding());
        Command endCalibration = this.robot.vision.commands.beginUsingInternalLL4IMUAssist();
        
        return haltSwerve
            .alongWith(beginCalibration)
            .withTimeout(Seconds.of(2))
            .andThen(endCalibration)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
        
    }
    
    public Command intake() {
        
        Command prepareAndRunIntake =
            this.robot.intake.commands.goToPosition(IntakePosition.INTAKING)
                .andThen(this.robot.intake.commands.intake(() -> 0.65));
        Runnable resetIntakePosition = () ->
            this.robot.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command outtake() {
        
        Command prepareAndRunIntake =
            this.robot.intake.commands.goToPosition(IntakePosition.INTAKING)
                .andThen(this.robot.intake.commands.intake(() -> -0.65));
        Runnable resetIntakePosition = () ->
            this.robot.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command shoot(
        TurretWheelSpeeds turretState,
        Time spinupWaitTime,
        boolean withLock
    ) {
        
        
        Command headingLock = this.robot.swerve.commands.enablePOIHeadingLock(
            VirtualField.getHubCenterPoint(),
            Direction.RIGHT
        );
        Command radiusLock = this.robot.swerve.commands.enablePOIRadiusLock(
            VirtualField.getHubCenterPoint(),
            Feet.of(9)
        );
        Command enableLocks = withLock
            ? headingLock.alongWith(radiusLock)
            : new InstantCommand();
        Command spinUpShooter = this.robot.turret.commands.shoot(turretState);
//        Command agitate = this.robot.agitator.commands.agitate()
        Command pulse = this.robot.intake.commands.pulseV3();
        Command waitForSpinup = Commands.waitTime(spinupWaitTime);
//        Command waitForHeadingLock = Commands.waitUntil(this.robot.swerve.headingLock::hasLock);
//        Command waitForRadiusLock = Commands.waitUntil(this.robot.swerve.radiusLock::hasLock);
        Command waitUntilReady = waitForSpinup;
        Command feedShooter = this.robot.indexer.commands.forward();
        
        return enableLocks.andThen(
            spinUpShooter
                .alongWith(pulse)
                .alongWith(waitUntilReady.andThen(feedShooter))
        );
        
    }
    
    public Command shoot(
        TurretWheelSpeeds turretState,
        boolean withLocks
    ) {
        
        return this.shoot(turretState, Seconds.of(0.5), withLocks);
        
    }
    
    public Command shoot() {
        
        return this.shoot(TurretWheelSpeeds.MID_SHOT, true);
        
    }
    
}
