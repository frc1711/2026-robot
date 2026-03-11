package frc.robot.configuration;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

public class RobotDimensions {

    public static final Distance SWERVE_LR_WHEELBASE_DISTANCE = Inches.of(27.125);
    
    public static final Distance SWERVE_FR_WHEELBASE_DISTANCE = Inches.of(16);
    
    public static final int SWERVE_DRIVE_S1_R1_DRIVING_TOOTH_COUNT = 12;
    
    public static final int SWERVE_DRIVE_S1_R2_DRIVING_TOOTH_COUNT = 14;
    
    public static final int SWERVE_DRIVE_S1_R3_DRIVING_TOOTH_COUNT = 16;
    
    public static final int SWERVE_DRIVE_S1_DRIVEN_TOOTH_COUNT = 54;
    
    public static final int SWERVE_DRIVE_S2_DRIVING_TOOTH_COUNT = 32;
    
    public static final int SWERVE_DRIVE_S2_DRIVEN_TOOTH_COUNT = 25;
    
    public static final int SWERVE_DRIVE_S3_DRIVING_TOOTH_COUNT = 15;
    
    public static final int SWERVE_DRIVE_S3_DRIVEN_TOOTH_COUNT = 30;
    
    public static final Distance SWERVE_WHEEL_DIAMETER = Inches.of(4);
    
    public static final Distance SWERVE_WHEEL_CIRCUMFERENCE =
        RobotDimensions.SWERVE_WHEEL_DIAMETER.times(Math.PI);
    
    public static final Distance HTD5_BELT_PITCH = Millimeters.of(5);
    
    public static final int INTAKE_EXTENSION_PULLEY_TOOTH_COUNT = 11;
    
    public static final int TURRET_ROTATION_DRIVING_PULLEY_TOOTH_COUNT = 11;
    
    public static final int TURRET_ROTATION_DRIVEN_PULLEY_TOOTH_COUNT = 136;
    
    public static final int TURRET_LOWER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT = 24;
    
    public static final int TURRET_LOWER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT = 36;
    
    public static final Distance TURRET_LOWER_WHEEL_DIAMETER = Inches.of(4);
    
    public static final Distance TURRET_LOWER_WHEEL_CIRCUMFERENCE =
        RobotDimensions.TURRET_LOWER_WHEEL_DIAMETER.times(Math.PI);
    
    public static final int TURRET_UPPER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT = 24;
    
    public static final int TURRET_UPPER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT = 30;
    
    public static final Distance TURRET_UPPER_WHEEL_DIAMETER = Inches.of(4);
    
    public static final Distance TURRET_UPPER_WHEEL_CIRCUMFERENCE =
        RobotDimensions.TURRET_UPPER_WHEEL_DIAMETER.times(Math.PI);
    
}
