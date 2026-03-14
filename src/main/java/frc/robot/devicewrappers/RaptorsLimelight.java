package frc.robot.devicewrappers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.Direction;
import frc.robot.configuration.LimelightCameraOrientation;
import frc.robot.configuration.LimelightIMUMode;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.*;

public class RaptorsLimelight {
	
	public static final RaptorsLimelight LEFT_LIMELIGHT = new RaptorsLimelight(
		"limelight-left",
		"10.17.11.11",
		new LimelightCameraOrientation()
			.withRightOffset(RobotDimensions.FRAME_WIDTH.div(2).minus(Inches.of(1.75)).times(-1))
			.withUpwardOffset(Inches.of(7.5))
			.withPitchOffset(Degrees.of(20))
			.withYawOffset(Direction.LEFT)
	);
	
	public static final RaptorsLimelight RIGHT_LIMELIGHT = new RaptorsLimelight(
		"limelight-right",
		"10.17.11.12",
		new LimelightCameraOrientation()
			.withRightOffset(RobotDimensions.FRAME_WIDTH.div(2).minus(Inches.of(1.75)))
			.withUpwardOffset(Inches.of(7.5))
			.withPitchOffset(Degrees.of(20))
			.withYawOffset(Direction.RIGHT)
	);
	
	public static final RaptorsLimelight REAR_LIMELIGHT = new RaptorsLimelight(
		"limelight-rear",
		"10.17.11.13",
		new LimelightCameraOrientation()
	);
	
	public static final RaptorsLimelight FRONT_LIMELIGHT = new RaptorsLimelight(
		"limelight-front",
		"10.17.11.14",
		new LimelightCameraOrientation()
	);
	
	public static final RaptorsLimelight TURRET_LIMELIGHT = new RaptorsLimelight(
		"limelight-turret",
		"10.17.11.15",
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
