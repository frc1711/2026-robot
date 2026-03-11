package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.math.DoubleSupplierBuilder;
import frc.robot.math.Point;
import frc.robot.math.PointSupplierBuilder;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ChassisSpeedsSupplierBuilder implements Supplier<ChassisSpeeds> {

    protected final Supplier<ChassisSpeeds> supplier;

    public ChassisSpeedsSupplierBuilder(Supplier<ChassisSpeeds> supplier) {

        this.supplier = supplier;

    }

    public static ChassisSpeedsSupplierBuilder stopped() {

        return new ChassisSpeedsSupplierBuilder(ChassisSpeeds::new);

    }
    
    public static ChassisSpeedsSupplierBuilder forwards(LinearVelocity velocity) {
        
        return new ChassisSpeedsSupplierBuilder(() -> new ChassisSpeeds(
            velocity,
            InchesPerSecond.of(0),
            DegreesPerSecond.of(0)
        ));
        
    }
    
    public static ChassisSpeedsSupplierBuilder backwards(LinearVelocity velocity) {
        
        return new ChassisSpeedsSupplierBuilder(() -> new ChassisSpeeds(
            velocity.times(-1),
            InchesPerSecond.of(0),
            DegreesPerSecond.of(0)
        ));
        
    }

    public static ChassisSpeedsSupplierBuilder left(LinearVelocity velocity) {

        return new ChassisSpeedsSupplierBuilder(() -> new ChassisSpeeds(
                InchesPerSecond.of(0),
                velocity,
                DegreesPerSecond.of(0)
        ));

    }

    public static ChassisSpeedsSupplierBuilder right(LinearVelocity velocity) {

        return new ChassisSpeedsSupplierBuilder(() -> new ChassisSpeeds(
            InchesPerSecond.of(0),
            velocity.times(-1),
            DegreesPerSecond.of(0)
        ));

    }
    
    public static ChassisSpeedsSupplierBuilder fromControllerJoysticks(CommandXboxController controller) {

        Supplier<Point> translationInput =
            PointSupplierBuilder.getTranslationPointSupplier(controller);
        DoubleSupplier rotationInput =
            DoubleSupplierBuilder.getRotationDoubleSupplier(controller);

        return new ChassisSpeedsSupplierBuilder(() -> {
            double maxVelocityMetersPerSecond = Swerve.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
            Translation2d translation = translationInput.get().times(maxVelocityMetersPerSecond);
            return new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                Swerve.MAX_ANGULAR_VELOCITY.times(rotationInput.getAsDouble()).in(RadiansPerSecond)
            );
        });

    }

    public static ChassisSpeedsSupplierBuilder fromControllerDPad(CommandXboxController controller) {

        return new ChassisSpeedsSupplierBuilder(() -> {

            boolean isUp = false;
            boolean isRight = false;
            boolean isDown = false;
            boolean isLeft = false;
            int povAngle = controller.getHID().getPOV();

            if (povAngle < 0) {}
            else if (povAngle > 45 && povAngle <= 135) isRight = true;
            else if (povAngle > 135 && povAngle <= 225) isDown = true;
            else if (povAngle > 225 && povAngle <= 315) isLeft = true;
            else isUp = true;

            LinearVelocity speed = FeetPerSecond.of(1.5);
            LinearVelocity vx = isUp ? speed : isDown ? speed.times(-1) : speed.times(0);
            LinearVelocity vy = isLeft ? speed : isRight ? speed.times(-1) : speed.times(0);

            return new ChassisSpeeds(
                vx,
                vy,
                DegreesPerSecond.of(0)
            );

        });

    }

    public ChassisSpeedsSupplierBuilder withOverride(Supplier<ChassisSpeeds> overrideSupplier) {

        return new ChassisSpeedsSupplierBuilder(() -> {
            ChassisSpeeds base = this.get();
            ChassisSpeeds override = overrideSupplier.get();
            boolean overrideHasTranslation = override.vxMetersPerSecond != 0 || override.vyMetersPerSecond != 0;
            return new ChassisSpeeds(
                overrideHasTranslation ? override.vxMetersPerSecond : base.vxMetersPerSecond,
                overrideHasTranslation ? override.vyMetersPerSecond : base.vyMetersPerSecond,
                override.omegaRadiansPerSecond != 0 ? override.omegaRadiansPerSecond : base.omegaRadiansPerSecond
            );
        });

    }

    public ChassisSpeedsSupplierBuilder withAdditional(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {

        return new ChassisSpeedsSupplierBuilder(() -> {
            ChassisSpeeds base = this.get();
            ChassisSpeeds additional = chassisSpeedsSupplier.get();
            return new ChassisSpeeds(
                base.vxMetersPerSecond + additional.vxMetersPerSecond,
                base.vyMetersPerSecond + additional.vyMetersPerSecond,
                base.omegaRadiansPerSecond + additional.omegaRadiansPerSecond
            );
        });

    }

    public ChassisSpeedsSupplierBuilder withFieldRelative(Swerve swerve) {

        return new ChassisSpeedsSupplierBuilder(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
            this.get(),
            Rotation2d.fromDegrees(swerve.getFieldRelativeHeading().in(Degrees))
        ));

    }

    public ChassisSpeedsSupplierBuilder withSlowModeCheck(Swerve swerve) {

        return new ChassisSpeedsSupplierBuilder(() -> {

            ChassisSpeeds chassisSpeeds = this.get();

            if (!swerve.isSlowModeEnabled) return chassisSpeeds;

            Translation2d originalLinearSpeeds = new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond
            );

            Translation2d newLinearSpeeds = new Translation2d(
                originalLinearSpeeds.getNorm() * (
                    Swerve.SLOW_MODE_MAX_LINEAR_VELOCITY.in(InchesPerSecond) /
                    Swerve.MAX_LINEAR_VELOCITY.in(InchesPerSecond)
                ),
                originalLinearSpeeds.getAngle()
            );

            return new ChassisSpeeds(
                newLinearSpeeds.getX(),
                newLinearSpeeds.getY(),
                chassisSpeeds.omegaRadiansPerSecond * (
                    Swerve.SLOW_MODE_MAX_ANGULAR_VELOCITY.in(DegreesPerSecond) /
                    Swerve.MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
                )
            );

        });

    }

    public ChassisSpeedsSupplierBuilder withMaxVelocityCheck() {
        
        return new ChassisSpeedsSupplierBuilder(() -> {
            ChassisSpeeds chassisSpeeds = this.get();
            return new Point(chassisSpeeds)
                .applyMaxNorm(Swerve.MAX_LINEAR_VELOCITY.times(Seconds.of(1)))
                .toChassisSpeeds(Math.min(
                    chassisSpeeds.omegaRadiansPerSecond,
                    Swerve.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
                ));
        });

    }

    public ChassisSpeedsSupplierBuilder withMaxAccelerationCheck() {

        ChassisSpeedsSlewRateLimiter limiter = new ChassisSpeedsSlewRateLimiter(
            Swerve.MAX_LINEAR_ACCELERATION,
            Swerve.MAX_LINEAR_DECELERATION,
            Swerve.MAX_ANGULAR_ACCELERATION,
            Swerve.MAX_ANGULAR_DECELERATION
        );

        return new ChassisSpeedsSupplierBuilder(() -> limiter.calculate(this.get()));

    }
    
    public ChassisSpeedsSupplierBuilder withHeadingLock(Swerve swerve) {

        PIDController headingLockController = new PIDController(0.1, 0, 0);
        
        return new ChassisSpeedsSupplierBuilder(() -> {
            
            ChassisSpeeds chassisSpeeds = this.get();
            Angle headingLock = swerve.headingLockSupplier != null
                ? swerve.headingLockSupplier.get()
                : Degrees.zero();
            double headingCorrection = headingLockController.calculate(
                swerve.getFieldRelativeHeading().in(Degrees),
                headingLock.in(Degrees)
            );
            double omegaRadiansPerSecond = swerve.headingLockSupplier != null
                ? headingCorrection
                : chassisSpeeds.omegaRadiansPerSecond;
            
            return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                omegaRadiansPerSecond
            );
            
        });
        
    }

    @Override
    public ChassisSpeeds get() {
        return this.supplier.get();
    }
}
