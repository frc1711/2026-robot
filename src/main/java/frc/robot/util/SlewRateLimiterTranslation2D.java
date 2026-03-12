package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class SlewRateLimiterTranslation2D extends SlewRateLimiter2D<Translation2d> {

    public SlewRateLimiterTranslation2D(double rateLimit) {
        super(rateLimit);
    }

    public SlewRateLimiterTranslation2D(
        double positiveRateLimit,
        double negativeRateLimit
    ) {
        super(positiveRateLimit, negativeRateLimit);
    }

    @Override
    protected Translation2d toTranslation2d(Translation2d value) {
        return value;
    }

    @Override
    protected Translation2d fromTranslation2d(Translation2d value) {
        return value;
    }
}
