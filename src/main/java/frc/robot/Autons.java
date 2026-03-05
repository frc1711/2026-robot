package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Highway;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class Autons {
    private final AutoFactory autoFactory;
    private final SwerveSubsystem swerve;
    private final Shooter shooter;
    private final Highway highway;

    public Autons(SwerveSubsystem swerve, Shooter shooter, Highway highway) {
        this.autoFactory = new AutoFactory(
            swerve::getPose, 
            swerve::resetOdometry, 
            swerve::followTrajectory, 
            true, 
            swerve
        );

        this.swerve = swerve;
        this.shooter = shooter;
        this.highway = highway;
    }

    public Command leftHandedAuton() {
        return Commands.sequence(
            shooter.commands.shootVelocity(),
            autoFactory.resetOdometry("LeftShoot"),
            autoFactory.trajectoryCmd("LeftShoot")
        ).withName("Left Handed Auton");
    }

    public Command rightHandedAuton() {
        return Commands.sequence(
            shooter.commands.shootVelocity(),
            autoFactory.resetOdometry("RightShoot"),
            autoFactory.trajectoryCmd("RightShoot")
        ).withName("Right Handed Auton");
    }

    public Command ambidextriousAuton() {
        return Commands.sequence(
            shooter.commands.shootVelocity(),
            autoFactory.resetOdometry("MiddleShoot"),
            autoFactory.trajectoryCmd("MiddleShoot")
        ).withName("Ambidextrious Auton");
    }

    // This would only shoot our 8 preload, the name is to be funny
    public Command superChargedAuton() {
        return Commands.sequence(
            shooter.commands.shootVelocity(),
            new InstantCommand(() -> swerve.setChassisSpeeds(new ChassisSpeeds(
                -1,
                0,
                0
            ), false), swerve)
        );
    }

    public AutoRoutine fireAndAscentAuton() {
        AutoRoutine autoRoutine = autoFactory.newRoutine("fireAndAscent");

        AutoTrajectory shootAndClimb = autoRoutine.trajectory("ShootAndClimb");

        autoRoutine.active().onTrue(
            Commands.sequence(
                shootAndClimb.resetOdometry(),
                shooter.commands.shootVelocity(),
                shootAndClimb.cmd()
                //climber.commands.climb()
            )
        );

        return autoRoutine;
    }
}
