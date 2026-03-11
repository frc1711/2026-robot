package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

public class ChassisSpeedsSlewRateLimiter {
    
    protected final SlewRateLimiterTranslation2D translationSlewRatLimiter;
    
    protected final ZeroCenteredSlewRateLimiter angularSlewRateLimiter;
    
    public ChassisSpeedsSlewRateLimiter(
        LinearAcceleration maxLinearAcceleration,
        AngularAcceleration maxAngularAcceleration
    ) {

        this(
            maxLinearAcceleration,
            maxLinearAcceleration,
            maxAngularAcceleration,
            maxAngularAcceleration
        );

    }

    public ChassisSpeedsSlewRateLimiter(
        LinearAcceleration maxLinearAcceleration,
        LinearAcceleration maxLinearDeceleration,
        AngularAcceleration maxAngularAcceleration,
        AngularAcceleration maxAngularDeceleration
    ) {

        this.translationSlewRatLimiter = new SlewRateLimiterTranslation2D(
            maxLinearAcceleration.in(MetersPerSecondPerSecond),
            maxLinearDeceleration.in(MetersPerSecondPerSecond)
        );
        this.angularSlewRateLimiter = new ZeroCenteredSlewRateLimiter(
            maxAngularAcceleration.in(RadiansPerSecondPerSecond),
            maxAngularDeceleration.in(RadiansPerSecondPerSecond),
            0
        );

    }

    public ChassisSpeeds calculate(ChassisSpeeds value) {

        Translation2d translation = this.translationSlewRatLimiter.calculate(
            new Translation2d(
                value.vxMetersPerSecond,
                value.vyMetersPerSecond
            )
        );

        double omegaRadiansPerSecond = this.angularSlewRateLimiter
            .calculate(value.omegaRadiansPerSecond);

        return new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            omegaRadiansPerSecond
        );

    }

}
