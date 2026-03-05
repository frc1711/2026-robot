package frc.robot.util;

public class FunctionUtilities {
     public static double applyClamp(double input, double minimum, double maximum) {
        if (input < minimum) return minimum;
        else if (input > maximum) return maximum;
        else return input;
    }

    public static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        }

        return Math.signum(input) * (Math.abs(input) - deadband) / (1 - deadband);
    }
}
