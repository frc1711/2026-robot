package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.state.SwerveDriveMotorGearRatio;

import java.util.stream.Stream;

public enum SwerveModuleConfiguration {
	
	FRONT_LEFT(
		0,
		CANDevice.SWERVE_FRONT_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_LEFT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_LEFT_STEER_ENCODER,
		DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveDriveMotorGearRatio.RATIO_ONE,
		0.19938,
		0.11574,
		0.0034994,
		0.0019956,
		0,
		0,
		true,
		true
	),
	
	FRONT_RIGHT(
		1,
		CANDevice.SWERVE_FRONT_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_RIGHT_STEER_ENCODER,
		DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveDriveMotorGearRatio.RATIO_ONE,
		0.1944,
		0.11547,
		0.0034045,
		0.0092993,
		0,
		0,
		true,
		false
	),
	
	REAR_LEFT(
		2,
		CANDevice.SWERVE_REAR_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_LEFT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_LEFT_STEER_ENCODER,
		DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveDriveMotorGearRatio.RATIO_ONE,
		0.16595,
		0.11566,
		0.0083564,
//		0.086531,
		0.002,
		0,
		0,
		false,
		true
	),
	
	REAR_RIGHT(
		3,
		CANDevice.SWERVE_REAR_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_RIGHT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_RIGHT_STEER_ENCODER,
		DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveDriveMotorGearRatio.RATIO_ONE,
		0.12722,
		0.11571,
		0.012295,
//		0.087194,
		0.002,
		0,
		0,
		false,
		false
	);
	
	public final int moduleID;
	
	public final CANDevice steerMotorControllerCANDevice;
	
	public final CANDevice driveMotorControllerCANDevice;
	
	public final CANDevice steerEncoderCANDevice;
	
	/**
	 * The offset (in rotations) of the steer encoder such that the module is
	 * facing straight forward when the encoder reads 0.
	 */
	public final DoublePreference steerEncoderOffset;
	
	public final SwerveDriveMotorGearRatio gearRatio;
	
	public final double kS;
	
	public final double kV;
	
	public final double kA;
	
	public final double kP;
	
	public final double kI;
	
	public final double kD;
	
	public final boolean isFront;
	
	public final boolean isLeft;
	
	/**
	 * The position of this module within the robot.
	 */
	public final Translation2d positionInRobot;
	
	SwerveModuleConfiguration(
		int moduleID,
		CANDevice steerMotorControllerCANDevice,
		CANDevice driveMotorControllerCANDevice,
		CANDevice steerEncoderCANDevice,
		DoublePreference steerEncoderOffset,
		SwerveDriveMotorGearRatio gearRatio,
		double kS,
		double kV,
		double kA,
		double kP,
		double kI,
		double kD,
		boolean isFront,
		boolean isLeft
	) {
		
		this.moduleID = moduleID;
		this.steerMotorControllerCANDevice = steerMotorControllerCANDevice;
		this.driveMotorControllerCANDevice = driveMotorControllerCANDevice;
		this.steerEncoderCANDevice = steerEncoderCANDevice;
		this.steerEncoderOffset = steerEncoderOffset;
		this.gearRatio = gearRatio;
		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.isFront = isFront;
		this.isLeft = isLeft;
		this.positionInRobot = new Translation2d(
			RobotDimensions.SWERVE_FR_WHEELBASE_DISTANCE.div(2).times(isFront ? 1 : -1),
			RobotDimensions.SWERVE_LR_WHEELBASE_DISTANCE.div(2).times(isLeft ? 1 : -1)
		);
		
	}
	
	public static Stream<SwerveModuleConfiguration> getModuleConfigurations() {
		
		return Stream.of(
			FRONT_LEFT,
			FRONT_RIGHT,
			REAR_LEFT,
			REAR_RIGHT
		);
		
	}
	
}
