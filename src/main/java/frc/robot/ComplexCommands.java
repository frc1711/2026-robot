package frc.robot;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.IntakePosition;
import frc.robot.state.TurretWheelSpeeds;
import frc.robot.util.ChassisSpeedsSupplierBuilder;

import static edu.wpi.first.units.Units.Seconds;

public class ComplexCommands {
    
    protected final RobotContainer robot;

    public ComplexCommands(RobotContainer robotContainer) {
        
        this.robot = robotContainer;
        
    }
    
    public Command drive(CommandXboxController controller) {
        
        return this.robot.swerve.commands.drive(
            ChassisSpeedsSupplierBuilder.fromControllerJoysticks(controller)
                .withAdditional(ChassisSpeedsSupplierBuilder.fromControllerDPad(controller))
                .withFieldRelative(this.robot.swerve)
//                .withSlowModeCheck(this.robot.swerve)
//                .withHeadingLock(this.robot.swerve)
                .withMaxVelocityCheck()
                .withMaxAccelerationCheck()
        ).finallyDo(this.robot.swerve::stop);
        
    }
    
    public Command intake() {
        
        Command prepareAndRunIntake =
            this.robot.intake.commands.goToPosition(IntakePosition.INTAKING)
                .andThen(this.robot.intake.commands.intake(() -> 0.65));
        Command driveSlowly = this.robot.swerve.commands.useDriveSpeedMultiplier(0.3);
        Runnable resetIntakePosition = () ->
            this.robot.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake.alongWith(driveSlowly)
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command shoot(
        TurretWheelSpeeds turretState,
        Time spinupWaitTime
    ) {
        
        Command spinUpShooter = this.robot.turret.commands.shoot(turretState);
        Command agitate = this.robot.agitator.commands.agitate()
            .alongWith(this.robot.intake.commands.pulseV1());
        Command waitForSpinup = Commands.waitTime(spinupWaitTime);
        Command feedShooter = this.robot.indexer.commands.forward();
        
        return spinUpShooter
            .alongWith(agitate)
            .alongWith(waitForSpinup.andThen(feedShooter));
        
    }
    
    public Command shoot(
        TurretWheelSpeeds turretState
    ) {
        
        return this.shoot(turretState, Seconds.of(0.5));
        
    }
    
    public Command shoot() {
        
        return this.shoot(TurretWheelSpeeds.MID_SHOT);
        
    }
    
}
