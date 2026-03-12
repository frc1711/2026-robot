package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A slew rate limiter class that provides asymmetric rate limiting control
 * depending on whether the value is approaching zero or not.
 *
 * In other words, the 'negative' rate limit refers to the rate limit when the
 * controlled value is approaching zero, while the 'positive' rate limit refers
 * to the rate limit when the controlled value is deviating from zero.
 */
public class ZeroCenteredSlewRateLimiter {
    
    /**
     * The rate limit when the controlled value is deviating from zero.
     */
    protected final double positiveRateLimit;

    /**
     * The rate limit when the controlled value is approaching zero.
     */
    protected final double negativeRateLimit;

    /**
     * The most recent input value received by the `calculate` function. 
     */
    protected double lastValue;

    /**
     * A timestamp for the last moment in time at which the `calculate` function
     * was called.
     */
    protected double lastTime;
    
    /**
     * Initializes a new ZeroCenteredSlewRateLimiter instance with the given
     * rate limit.
     * 
     * @param rateLimit The maximum rate at which (in input units per second)
     * that the controlled value should be allowed to change.
     */
    public ZeroCenteredSlewRateLimiter(double rateLimit) {

        this(rateLimit, rateLimit, 0);

    }

    /**
     * Initializes a new ZeroCenteredSlewRateLimiter instance with the given
     * positive and negative rate limits.
     * 
     * @param positiveRateLimit The rate limit when the controlled value is
     * deviating from zero.
     * @param negativeRateLimit The rate limit when the controlled value is
     * approaching zero.
     * @param initialValue The initial value for the rate limiter to operate off
     * of for the first calls against `calculate`.
     */
    public ZeroCenteredSlewRateLimiter(
        double positiveRateLimit,
        double negativeRateLimit,
        double initialValue
    ) {

        this.positiveRateLimit = Math.abs(positiveRateLimit);
        this.negativeRateLimit = Math.abs(negativeRateLimit);
        this.lastValue = initialValue;
        this.lastTime = Timer.getFPGATimestamp();

    }

    /**
     * Returns the rate limited output value based on the given input value,
     * rate limit, and previous value.
     * 
     * @param value The raw requested output value to be rate limited.
     * @return The rate limited output value based on the given input value,
     * rate limit, and previous value.
     */
    public double calculate(double value) {
        
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - this.lastTime;
        
        this.lastTime = currentTime;
        
        if (deltaTime <= 0) return value;
        
        double requestedDelta = value - this.lastValue;
        boolean isApproachingZero = requestedDelta < 0
            ? value >= 0
            : value <= 0;
        double rateLimit = isApproachingZero ? this.negativeRateLimit : this.positiveRateLimit;
        double maxDelta = rateLimit * deltaTime;
        
        this.lastValue += Math.copySign(
            Math.min(Math.abs(requestedDelta), maxDelta),
            requestedDelta
        );
        
        return this.lastValue;
        
    }

}
