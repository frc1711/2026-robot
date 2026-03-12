package frc.robot.state;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class SwerveDriveMotorGearRatioWithSpeeds {
	
	protected final SwerveDriveMotorGearRatio gearRatio;
	
	protected final AngularVelocity motorShaftAngularVelocity;
	
	public SwerveDriveMotorGearRatioWithSpeeds(
		SwerveDriveMotorGearRatio gearRatio,
		AngularVelocity motorShaftAngularVelocity
	) {
		
		this.gearRatio = gearRatio;
		this.motorShaftAngularVelocity = motorShaftAngularVelocity;
		
	}
	
	public AngularVelocity getMotorShaftAngularVelocity() {
		
		return this.motorShaftAngularVelocity;
		
	}
	
	public AngularVelocity getWheelAngularVelocity() {
		
		return this.gearRatio
			.convertMotorShaftThetaToWheelTheta(this.motorShaftAngularVelocity);
		
	}
	
	public LinearVelocity getWheelSurfaceSpeed() {
		
		return RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.times(Hertz.of(
			this.getWheelAngularVelocity().in(RotationsPerSecond)
		));
		
	}
	
}
