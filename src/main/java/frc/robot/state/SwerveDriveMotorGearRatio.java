package frc.robot.state;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.*;

public class SwerveDriveMotorGearRatio {
	
	public static final SwerveDriveMotorGearRatio RATIO_ONE =
		new SwerveDriveMotorGearRatio(RobotDimensions.SWERVE_DRIVE_S1_R1_DRIVING_TOOTH_COUNT);
	
	public static final SwerveDriveMotorGearRatio RATIO_TWO =
		new SwerveDriveMotorGearRatio(RobotDimensions.SWERVE_DRIVE_S1_R2_DRIVING_TOOTH_COUNT);
	
	public static final SwerveDriveMotorGearRatio RATIO_THREE =
		new SwerveDriveMotorGearRatio(RobotDimensions.SWERVE_DRIVE_S1_R3_DRIVING_TOOTH_COUNT);
	
	protected final int stageOneDrivingGearToothCount;
	
	protected SwerveDriveMotorGearRatio(int stageOneDriveGearToothCount) {
		
		this.stageOneDrivingGearToothCount = stageOneDriveGearToothCount;
		
	}
	
	public SwerveDriveMotorGearRatioWithSpeeds withMotorShaftAngularVelocity(
		AngularVelocity motorShaftAngularVelocity
	) {
		
		return new SwerveDriveMotorGearRatioWithSpeeds(
			this,
			motorShaftAngularVelocity
		);
		
	}
	
	public SwerveDriveMotorGearRatioWithSpeeds withWheelAngularVelocity(
		AngularVelocity wheelAngularVelocity
	) {
		
		return new SwerveDriveMotorGearRatioWithSpeeds(
			this,
			this.convertWheelThetaToMotorShaftTheta(wheelAngularVelocity)
		);
		
	}
	
	public SwerveDriveMotorGearRatioWithSpeeds withWheelSurfaceSpeed(
		LinearVelocity wheelSurfaceSpeed
	) {
		
		return new SwerveDriveMotorGearRatioWithSpeeds(
			this,
			this.convertWheelThetaToMotorShaftTheta(RotationsPerSecond.of(
				wheelSurfaceSpeed.in(InchesPerSecond) /
				RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.in(Inches)
			))
		);
		
	}
	
	public <T extends Measure<?>> T convertMotorShaftThetaToWheelTheta(T motorShaftTheta) {
		
		return (T) motorShaftTheta.mutableCopy()
			.mut_times(this.stageOneDrivingGearToothCount)
			.mut_divide(RobotDimensions.SWERVE_DRIVE_S1_DRIVEN_TOOTH_COUNT)
			.mut_times(RobotDimensions.SWERVE_DRIVE_S2_DRIVING_TOOTH_COUNT)
			.mut_divide(RobotDimensions.SWERVE_DRIVE_S2_DRIVEN_TOOTH_COUNT)
			.mut_times(RobotDimensions.SWERVE_DRIVE_S3_DRIVING_TOOTH_COUNT)
			.mut_divide(RobotDimensions.SWERVE_DRIVE_S3_DRIVEN_TOOTH_COUNT);
		
	}
	
	public <T extends Measure<?>> T convertWheelThetaToMotorShaftTheta(T wheelTheta) {
		
		return (T) wheelTheta.mutableCopy()
			.mut_times(RobotDimensions.SWERVE_DRIVE_S3_DRIVEN_TOOTH_COUNT)
			.mut_divide(RobotDimensions.SWERVE_DRIVE_S3_DRIVING_TOOTH_COUNT)
			.mut_times(RobotDimensions.SWERVE_DRIVE_S2_DRIVEN_TOOTH_COUNT)
			.mut_divide(RobotDimensions.SWERVE_DRIVE_S2_DRIVING_TOOTH_COUNT)
			.mut_times(RobotDimensions.SWERVE_DRIVE_S1_DRIVEN_TOOTH_COUNT)
			.mut_divide(this.stageOneDrivingGearToothCount);
		
	}
	
	public double getEffectiveReduction() {
		
		return (
			((double) this.stageOneDrivingGearToothCount / RobotDimensions.SWERVE_DRIVE_S1_DRIVEN_TOOTH_COUNT) *
			((double) RobotDimensions.SWERVE_DRIVE_S2_DRIVING_TOOTH_COUNT / RobotDimensions.SWERVE_DRIVE_S2_DRIVEN_TOOTH_COUNT) *
			((double) RobotDimensions.SWERVE_DRIVE_S3_DRIVING_TOOTH_COUNT / RobotDimensions.SWERVE_DRIVE_S3_DRIVEN_TOOTH_COUNT)
		);
		
	}
	
}
