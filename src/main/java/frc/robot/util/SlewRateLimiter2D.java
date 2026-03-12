package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public abstract class SlewRateLimiter2D<T> {

    /**
     * The maximum rate of positive change allowed in the overall magnitude of
     * the Translation2d per second.
     */
    protected final double positiveRateLimit;

    /**
     * The maximum rate of negative change in the overall magnitude of the
     * Translation2d per second.
     */
    protected final double negativeRateLimit;

    /**
     * The last value that was returned by the calculate method.
     */
    protected Translation2d lastValue;

    /**
     * The last time the calculate method was called.
     */
    protected double lastTime;

    /**
     * Initializes a new SlewRateLimiter2D with the specified rate limit (in units per second).
     *
     * @param rateLimit The maximum rate of change allowed in the overall
     * magnitude of the Translation2d per second.
     */
    public SlewRateLimiter2D(double rateLimit) {

        this(rateLimit, rateLimit);

    }

    /**
     * Initializes a new SlewRateLimiter2D with the specified positive and
     * negative rate limits (in units per second).
     *
     * @param positiveRateLimit The maximum rate of positive change allowed in
     * the overall magnitude of the Translation2d per second.
     * @param negativeRateLimit The maximum rate of negative change allowed in
     * the overall magnitude of the Translation2d per second.
     */
    public SlewRateLimiter2D(
        double positiveRateLimit,
        double negativeRateLimit
    ) {

        this.positiveRateLimit = Math.abs(positiveRateLimit);
        this.negativeRateLimit = Math.abs(negativeRateLimit);
        this.lastValue = new Translation2d(0, 0);
        this.lastTime = Timer.getFPGATimestamp();

    }

    protected abstract Translation2d toTranslation2d(T value);

    protected abstract T fromTranslation2d(Translation2d value);

    public T calculate(T value) {

        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - this.lastTime;

        this.lastTime = currentTime;

        if (deltaTime <= 0) return value;

        Translation2d currentValue = this.toTranslation2d(value);
        Translation2d delta = currentValue.minus(lastValue);

        boolean isApproachingZero = currentValue.getNorm() < this.lastValue.getNorm();
        double rateLimit = isApproachingZero ? this.negativeRateLimit : this.positiveRateLimit;
        double maximumDelta = rateLimit * deltaTime;
        double requestedDelta = delta.getNorm();
        double scalingFactor = Math.min(maximumDelta/requestedDelta, 1);

        Translation2d resultantValue = lastValue.plus(delta.times(scalingFactor));

        lastValue = resultantValue;
        lastTime = currentTime;

        return this.fromTranslation2d(lastValue);

    }

}
