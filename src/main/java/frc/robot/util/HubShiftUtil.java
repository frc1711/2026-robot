// Used 6328 Mechanical Advantage's as a base and adjusted from what was needed

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class HubShiftUtil {
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED
    }

    public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    private static Timer shiftTimer = new Timer();
    private static final ShiftEnum[] shifts = ShiftEnum.values();

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
    private static final double timeResetThreshold = 3.0;
    private static double shiftTimerOffset = 0.0;

    /** Get which alliance lost auto */
    public static Alliance getFirstActiveAlliance() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return FMS value
        String gameMessage = DriverStation.getGameSpecificMessage();
        if (!gameMessage.isEmpty()) {
            char character = gameMessage.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Starts the timer at the beginning of teleop. */
    public static void initialize() {
        shiftTimerOffset = 0;
        shiftTimer.restart();
    }

    private static boolean[] getSchedule() {
        boolean[] currentSchedule;
        Alliance startAlliance = getFirstActiveAlliance();
        currentSchedule =
                startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
                        ? activeSchedule
                        : inactiveSchedule;
        return currentSchedule;
    }

    private static ShiftInfo getShiftInfo(
            boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
        double timerValue = shiftTimer.get();
        double currentTime = timerValue - shiftTimerOffset;
        double stateTimeElapsed = currentTime;
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;
        double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            // Adjust the current offset if the time difference above the theshold
            if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
                    && fieldTeleopTime <= 135
                    && DriverStation.isFMSAttached()) {
                shiftTimerOffset += currentTime - fieldTeleopTime;
                currentTime = timerValue + shiftTimerOffset;
            }
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                currentShiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shifts[currentShiftIndex];
        }
        ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
        return shiftInfo;
    }

    public static ShiftInfo getOfficialShiftInfo() {
        return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
    }
}
