package frc.robot.configuration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class LimelightCameraOrientation {
	
	public final Distance forwardOffset;
	
	public final Distance rightOffset;
	
	public final Distance upwardOffset;
	
	public final Angle rollOffset;
	
	public final Angle pitchOffset;
	
	public final Angle yawOffset;
	
	public LimelightCameraOrientation(
		Distance forwardOffset,
		Distance rightOffset,
		Distance upwardOffset,
		Angle rollOffset,
		Angle pitchOffset,
		Angle yawOffset
	) {
		
		this.forwardOffset = forwardOffset;
		this.rightOffset = rightOffset;
		this.upwardOffset = upwardOffset;
		this.rollOffset = rollOffset;
		this.pitchOffset = pitchOffset;
		this.yawOffset = yawOffset;
		
	}
	
	public LimelightCameraOrientation() {
		
		this(
			Meters.of(0),
			Meters.of(0),
			Meters.of(0),
			Degrees.of(0),
			Degrees.of(0),
			Degrees.of(0)
		);
		
	}
	
	public LimelightCameraOrientation withForwardOffset(Distance forwardOffset) {
		
		return new LimelightCameraOrientation(
			forwardOffset,
			this.rightOffset,
			this.upwardOffset,
			this.rollOffset,
			this.pitchOffset,
			this.yawOffset
		);
		
	}
	
	public LimelightCameraOrientation withRightOffset(Distance rightOffset) {
		
		return new LimelightCameraOrientation(
			this.forwardOffset,
			rightOffset,
			this.upwardOffset,
			this.rollOffset,
			this.pitchOffset,
			this.yawOffset
		);
		
	}
	
	public LimelightCameraOrientation withUpwardOffset(Distance upwardOffset) {
		
		return new LimelightCameraOrientation(
			this.forwardOffset,
			this.rightOffset,
			upwardOffset,
			this.rollOffset,
			this.pitchOffset,
			this.yawOffset
		);
		
	}
	
	public LimelightCameraOrientation withRollOffset(Angle rollOffset) {
		
		return new LimelightCameraOrientation(
			this.forwardOffset,
			this.rightOffset,
			this.upwardOffset,
			rollOffset,
			this.pitchOffset,
			this.yawOffset
		);
		
	}
	
	public LimelightCameraOrientation withPitchOffset(Angle pitchOffset) {
		
		return new LimelightCameraOrientation(
			this.forwardOffset,
			this.rightOffset,
			this.upwardOffset,
			this.rollOffset,
			pitchOffset,
			this.yawOffset
		);
		
	}
	
	public LimelightCameraOrientation withYawOffset(Angle yawOffset) {
		
		return new LimelightCameraOrientation(
			this.forwardOffset,
			this.rightOffset,
			this.upwardOffset,
			this.rollOffset,
			this.pitchOffset,
			yawOffset
		);
		
	}
	
}
