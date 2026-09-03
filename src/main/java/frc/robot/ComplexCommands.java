package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.configuration.RobotDimensions;
import frc.robot.state.IntakePosition;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.utils.PoseBuilder;
import frc.robot.utils.VirtualField;

public class ComplexCommands {

    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Agitator agitator;
    private final Indexer indexer;
    //private final Intake intake;

    public ComplexCommands(RobotContainer robot) {

        this.turret = robot.turret;
        this.drivetrain = robot.drivetrain;
        this.agitator = robot.agitator;
        this.indexer = robot.indexer;
        //this.intake = robot.intake;

    }

    /*public Command intake() {
        
        Command prepareAndRunIntake =
            new InstantCommand(() -> this.intake.goToPosition(IntakePosition.INTAKING));
                //.andThen(this.intake.intake(() -> 0.65));
        Command waitSeconds = Commands.waitSeconds(0.25);
        Command spinIntake = this.intake.commands.intake(() -> 0.6);
        Runnable resetIntakePosition = () ->
            this.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .alongWith(waitSeconds.andThen(spinIntake))
            .finallyDo(resetIntakePosition);
        
    }
    
    public Command outtake() {
        
        Command prepareAndRunIntake =
            new InstantCommand(() -> this.intake.goToPosition(IntakePosition.INTAKING));
                //.andThen(this.intake.intake(() -> 0.65));
        Command waitSeconds = Commands.waitSeconds(0.25);
        Command spinIntake = this.intake.commands.intake(() -> -0.65);
        Runnable resetIntakePosition = () ->
            this.intake.goToPosition(IntakePosition.PARTIALLY_STOWED);
        
        return prepareAndRunIntake
            .alongWith(waitSeconds.andThen(spinIntake))
            .finallyDo(resetIntakePosition);
        
        
    }*/

    public Command lockTurretHeadingToHub() {

            Supplier<Angle> headingToHub = () -> {

                Translation2d turretOffset = new Translation2d(
                    RobotDimensions.TURRET_X_OFFSET_FROM_ROBOT_CENTER,
                    RobotDimensions.TURRET_Y_OFFSET_FROM_ROBOT_CENTER
                );
                Translation2d turretCenterPoint =
                    PoseBuilder.fromPose(this.drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).orElse(Pose2d.kZero))
                        .withTranslation(PoseBuilder.CoordinateSystem.ROBOT_RELATIVE, turretOffset)
                        .get()
                        .getTranslation();
                Translation2d hubCenterPoint = VirtualField.getHubCenterPoint();
                Translation2d delta = hubCenterPoint.minus(turretCenterPoint);

                return delta.getAngle()
                    .getMeasure()
                    .minus(this.drivetrain.getRotation3d().getMeasureZ());

            };

            return new InstantCommand(
                () -> this.turret.setHeadingSupplier(headingToHub)
            );

        }

    public Command shoot(
        Turret.WheelSpeeds turretState,
        Time spinupWaitTime,
        boolean withLock,
        boolean withPulse
    ) {

        Command spinUpShooter = this.turret.commands.shoot(turretState);
//        Command pulse = withPulse
//            ? this.intake.pulseV3()
//            : new InstantCommand();
        Command waitUntilReady = Commands.waitTime(spinupWaitTime);
        Command feedShooter = this.indexer.commands.forward()
            .alongWith(Commands.waitTime(Seconds.of(0.5)).andThen(this.agitator.commands.agitate()));

        return Commands.parallel(
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
}
