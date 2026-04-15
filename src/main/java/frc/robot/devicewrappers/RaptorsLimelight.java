package frc.robot.devicewrappers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.configuration.LimelightCameraOrientation;
import frc.robot.configuration.LimelightIMUMode;

import static edu.wpi.first.units.Units.*;

public class RaptorsLimelight {
	
	public static final RaptorsLimelight FRONT_LIMELIGHT = new RaptorsLimelight(
		"limelight-front",
		"10.17.11.11",
		new LimelightCameraOrientation()
			.withRightOffset(Meters.of(-0.032512))
			.withUpwardOffset(Meters.of(0.46512))
			.withForwardOffset(Meters.of(-0.1914398))
			.withPitchOffset(Degrees.of(20))
//			.withYawOffset(Direction.LEFT)
	);
	
	public static final RaptorsLimelight REAR_LIMELIGHT = new RaptorsLimelight(
		"limelight-rear",
		"10.17.11.12",
		new LimelightCameraOrientation()
			.withRightOffset(Meters.of(0.247777))
			.withUpwardOffset(Meters.of(0.4651248))
			.withForwardOffset(Meters.of(-0.2512568))
			.withPitchOffset(Degrees.of(20))
			.withYawOffset(Degrees.of(170))
	);
	
	public static final RaptorsLimelight LEFT_LIMELIGHT = new RaptorsLimelight(
		"limelight-left",
		"10.17.11.13",
		new LimelightCameraOrientation()
	);
	
	public static final RaptorsLimelight RIGHT_LIMELIGHT = new RaptorsLimelight(
		"limelight-right",
		"10.17.11.14",
		new LimelightCameraOrientation()
	);
	
	protected final String hostname;
	
	protected final String ipAddress;
	
	protected final LimelightCameraOrientation orientation;
	
	protected RaptorsLimelight(
		String hostname,
		String ipAddress,
		LimelightCameraOrientation orientation
	) {
		
		this.hostname = hostname;
		this.ipAddress = ipAddress;
		this.orientation = orientation;
		
	}
	
	public void setCameraOrientation(
		LimelightCameraOrientation orientation
	) {
		
		LimelightHelpers.setCameraPose_RobotSpace(
			this.hostname,
			orientation.forwardOffset.in(Meters),
			orientation.rightOffset.in(Meters),
			orientation.upwardOffset.in(Meters),
			orientation.rollOffset.in(Degrees),
			orientation.pitchOffset.in(Degrees),
			orientation.yawOffset.in(Degrees)
		);
		
	}
	
	public void setRobotOrientation(
		Angle yaw,
		AngularVelocity yawVelocity
	) {
		
		LimelightHelpers.SetRobotOrientation(
			this.hostname,
			yaw.in(Degrees),
			yawVelocity.in(DegreesPerSecond),
			0,
			0,
			0,
			0
		);
		
	}
	
	public void setRobotOrientation(Angle yaw) {
		
		this.setRobotOrientation(yaw, RotationsPerSecond.of(0));
		
	}
	
	public void setIMUMode(LimelightIMUMode mode) {
		
		LimelightHelpers.SetIMUMode(this.hostname, mode.modeID);
		
	}
	
	public void setIMUAssistAlpha(double alpha) {
		
		LimelightHelpers.SetIMUAssistAlpha(this.hostname, alpha);
		
	}
	
	public void initialize() {
		
		this.setCameraOrientation(this.orientation);
		this.setIMUMode(LimelightIMUMode.EXTERNAL_SEED);
		
	}
	
	public LimelightHelpers.PoseEstimate getBotPoseEstimate() {
		
		return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
			this.hostname
		);
		
	}
	
}
