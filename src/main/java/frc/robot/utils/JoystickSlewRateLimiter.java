package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.Timer;

public class JoystickSlewRateLimiter {
    private final double m_accelRate; // units per second (always positive)
    private final double m_decelRate; // units per second (always positive)

    private double m_lastValue;
    private double m_lastTimestamp;

    /**
     * @param timeToFullSpeed Seconds to go from 0 to 1.0 (accelerating).
     *                        Smaller = snappier, larger = smoother.
     * @param timeToStop      Seconds to go from 1.0 to 0 (decelerating).
     *                        Should usually be smaller than
     *                        {@code timeToFullSpeed} so the robot
     *                        brakes faster than it accelerates.
     */
    public JoystickSlewRateLimiter(double timeToFullSpeed, double timeToStop) {
        m_accelRate = 1.0 / Math.max(timeToFullSpeed, 0.001);
        m_decelRate = 1.0 / Math.max(timeToStop, 0.001);
        m_lastValue = 0.0;
        m_lastTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Feed in the raw input (–1 to 1 typically) and get back the
     * rate-limited output.
     */
    public double calculate(double input) {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_lastTimestamp;
        m_lastTimestamp = now;

        double currentMag = Math.abs(m_lastValue);
        double targetMag = Math.abs(input);
        int signCurrent = (int) Math.signum(m_lastValue);
        int signTarget = (int) Math.signum(input);

        double newMag;
        if (signCurrent == signTarget || currentMag < 1e-12) {
            // Same direction (or starting from rest) — simple accel/decel
            if (targetMag > currentMag) {
                newMag = Math.min(currentMag + m_accelRate * dt, targetMag);
            } else {
                newMag = Math.max(currentMag - m_decelRate * dt, targetMag);
            }
            m_lastValue = Math.copySign(newMag, input);
        } else {
            // Sign is flipping — decelerate toward 0 first
            // (next iteration, when we're near 0, we'll accelerate
            // in the new direction)
            newMag = Math.max(currentMag - m_decelRate * dt, 0.0);
            m_lastValue = Math.copySign(newMag, m_lastValue);
        }

        return m_lastValue;
    }

    /** Reset the limiter's internal value (e.g. on disable). */
    public void reset(double value) {
        m_lastValue = value;
        m_lastTimestamp = Timer.getFPGATimestamp();
    }

    /** The current output value without running another iteration. */
    public double lastValue() {
        return m_lastValue;
    }

    /** The configured acceleration time in seconds. */
    public double getTimeToFullSpeed() {
        return 1.0 / m_accelRate;
    }

    /** The configured deceleration time in seconds. */
    public double getTimeToStop() {
        return 1.0 / m_decelRate;
    }
}
