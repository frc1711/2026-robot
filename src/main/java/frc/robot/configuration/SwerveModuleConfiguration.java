package frc.robot.configuration;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.state.SwerveGearRatio;

import java.util.stream.Stream;

public enum SwerveModuleConfiguration {
	
	FRONT_LEFT(
		0,
		CANDevice.SWERVE_FRONT_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_LEFT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_LEFT_STEER_ENCODER,
		DoublePreference.FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveGearRatio.RATIO_TWO,
		true,
		true
	),
	
	FRONT_RIGHT(
		1,
		CANDevice.SWERVE_FRONT_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_FRONT_RIGHT_STEER_ENCODER,
		DoublePreference.FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveGearRatio.RATIO_TWO,
		true,
		false
	),
	
	REAR_LEFT(
		2,
		CANDevice.SWERVE_REAR_LEFT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_LEFT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_LEFT_STEER_ENCODER,
		DoublePreference.REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveGearRatio.RATIO_TWO,
		false,
		true
	),
	
	REAR_RIGHT(
		3,
		CANDevice.SWERVE_REAR_RIGHT_STEER_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_RIGHT_DRIVE_MOTOR_CONTROLLER,
		CANDevice.SWERVE_REAR_RIGHT_STEER_ENCODER,
		DoublePreference.REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES,
		SwerveGearRatio.RATIO_TWO,
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
	
	public final SwerveGearRatio gearRatio;
	
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
		SwerveGearRatio gearRatio,
		boolean isFront,
		boolean isLeft
	) {
		
		this.moduleID = moduleID;
		this.steerMotorControllerCANDevice = steerMotorControllerCANDevice;
		this.driveMotorControllerCANDevice = driveMotorControllerCANDevice;
		this.steerEncoderCANDevice = steerEncoderCANDevice;
		this.steerEncoderOffset = steerEncoderOffset;
		this.gearRatio = gearRatio;
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
