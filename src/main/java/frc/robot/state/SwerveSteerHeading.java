package frc.robot.state;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import frc.robot.configuration.RobotDimensions;

public class SwerveSteerHeading {
	
	protected final Angle heading;
	
	public SwerveSteerHeading(Angle heading) {
		
		this.heading = heading;
		
	}
	
	public static <T extends Measure<?>> T convertMotorShaftThetaToHeadingTheta(T motorShaftTheta) {
		
		return (T) motorShaftTheta
			.div(RobotDimensions.SWERVE_STEERING_GEAR_RATIO);
		
	}
	
	public static <T extends Measure<?>> T convertHeadingThetaToMotorShaftTheta(T headingTheta) {
		
		return (T) headingTheta
			.times(RobotDimensions.SWERVE_STEERING_GEAR_RATIO);
		
	}
	
	public Angle getHeading() {
		
		return this.heading;
		
	}
	
	public Angle getMotorShaftAngle() {
		
		return SwerveSteerHeading
			.convertHeadingThetaToMotorShaftTheta(this.heading);
		
	}
	
}
