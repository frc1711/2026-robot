package frc.robot.math;

/**
 * A collection of utility methods for manipulating doubles.
 */
public class DoubleUtilities {

    // Prevent instantiation of this utility class.
    private DoubleUtilities() {}

    /**
     * Normalizes the input value to a specified range, wrapping it around
     * the minimum and maximum values if it exceeds the range.
     *
     * @param input The input value to normalize.
     * @param minimum The minimum value of the range.
     * @param maximum The maximum value of the range.
     * @return The normalized value within the specified range.
     */
    public static double normalizeToRange(
        double input,
        double minimum,
        double maximum
    ) {

        double delta = maximum - minimum;
        return minimum + ((input - minimum + delta) % delta);

    }

    /**
     * Applies an exponential curve to the input value, raising it to the
     * specified exponent and preserving its sign.
     *
     * @param input The input value.
     * @param exponent The exponent to which to raise the input value.
     * @return The input value raised to the specified exponent, preserving its
     * sign.
     */
    public static double applyExponentialCurve(double input, double exponent) {

        return Math.copySign(Math.pow(input, exponent), input);

    }

    /**
     * Returns the input value so long as its absolute value is greater than the
     * specified deadband. If the absolute value of the input value is less than
     * the specified deadband, 0 will be returned.
     *
     * @param input The input value.
     * @param deadband The deadband.
     * @return The input value, or 0 if the absolute value of the input value is
     * less than the specified deadband.
     */
    public static double applyDeadband(double input, double deadband) {

        return Math.abs(input) < deadband ? 0 : input;

    }

    /**
     * Applies a deadband to the input value whose output is scaled to the
     * initial input range. This avoids the issue of control cut-outs at the low
     * end of the output range.
     *
     * @param input The input value.
     * @param deadband The deadband.
     * @return The input value scaled to the output range, minus the deadband,
     * or zero if the input is lower than the deadband.
     */
    public static double applyScaledDeadband(double input, double deadband) {

        return Math.abs(input) < deadband ? 0 : Math.copySign(
            (Math.abs(input) - deadband) / (1 - deadband),
            input
        );

    }

    /**
     * Returns the new value so long as its delta from the old value does not
     * exceed the specified maximum permissible amount. If this delta is greater
     * than what is permissible, the returned value will instead be the old
     * value plus the maximum permissible delta in the direction of the new
     * value.
     *
     * @param oldValue The old value of the variable.
     * @param newValue The new value of the variable.
     * @param maxDelta The maximum allowable change.
     * @return The new value, or the old value plus the maximum permissible
     * delta in the direction of the new value, if the delta between the old and
     * new values exceeds what is permissible.
     */
    public static double enforceMaximumDelta(
        double oldValue,
        double newValue,
        double maxDelta
    ) {

        double change = newValue - oldValue;

        if (Math.abs(change) <= maxDelta) return newValue;
        else return oldValue + Math.copySign(maxDelta, change);

    }

    /**
     * Returns the new value so long as it is either moving towards zero, or its
     * delta from the old value does not exceed the specified maximum
     * permissible amount. If this delta is greater than what is permissible,
     * the returned value will instead be the old value plus the maximum
     * permissible delta in the direction of the new value.
     *
     * @param oldValue The old value of the variable.
     * @param newValue The new value of the variable.
     * @param maxIncrease The maximum allowable increase.
     * @return The new value, or the old value plus the maximum permissible
     * delta in the direction of the new value, if the delta between the old and
     * new values exceeds what is permissible and the new value is not moving
     * towards zero.
     */
    public static double enforceMaximumPositiveDelta(
        double oldValue,
        double newValue,
        double maxIncrease
    ) {

        boolean isValueIncreasing = newValue > oldValue;
        boolean isMovementTowardsZero =
                (isValueIncreasing && oldValue < 0) ||
                        (!isValueIncreasing && oldValue > 0);

        if (isMovementTowardsZero) return newValue;

        return isValueIncreasing ?
                Math.min(newValue, oldValue + maxIncrease) :
                Math.max(newValue, oldValue - maxIncrease);

    }

}
