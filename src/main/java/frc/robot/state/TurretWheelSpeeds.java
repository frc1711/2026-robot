package frc.robot.state;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.configuration.RobotDimensions;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class TurretWheelSpeeds {
	
	public static final TurretWheelSpeeds CLOSE_SHOT =
		TurretWheelSpeeds.fromRelativeStaticAngularWheelVelocities(
			RotationsPerSecond.of(50),
			0.75
		);
	
	public static final TurretWheelSpeeds MID_SHOT =
		TurretWheelSpeeds.fromRelativeStaticAngularWheelVelocities(
			RotationsPerSecond.of(55),
			0.75
		);
	
	public static final TurretWheelSpeeds FAR_SHOT =
		TurretWheelSpeeds.fromRelativeStaticAngularWheelVelocities(
			RotationsPerSecond.of(60),
			0.75
		);
	
	Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier;
	
	Supplier<AngularVelocity> upperWheelAngularVelocitySupplier;
	
	public TurretWheelSpeeds(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
	) {
		
		this.lowerWheelAngularVelocitySupplier = lowerWheelAngularVelocitySupplier;
		this.upperWheelAngularVelocitySupplier = upperWheelAngularVelocitySupplier;
		
	}
	
	public static TurretWheelSpeeds fromDynamicAngularWheelVelocities(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
	) {
		
		return new TurretWheelSpeeds(
			lowerWheelAngularVelocitySupplier,
			upperWheelAngularVelocitySupplier
		);
		
	}
	
	public static TurretWheelSpeeds fromStaticAngularWheelVelocities(
		AngularVelocity lowerWheelAngularVelocity,
		AngularVelocity upperWheelAngularVelocity
	) {
		
		return new TurretWheelSpeeds(
			() -> lowerWheelAngularVelocity,
			() -> upperWheelAngularVelocity
		);
		
	}
	
	public static TurretWheelSpeeds fromRelativeDynamicAngularWheelVelocities(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		DoubleSupplier upperWheelRelativeSpeedMultiplierSupplier
	) {
		
		return new TurretWheelSpeeds(
			lowerWheelAngularVelocitySupplier,
			() -> lowerWheelAngularVelocitySupplier.get()
				.times(upperWheelRelativeSpeedMultiplierSupplier.getAsDouble())
		);
		
	}
	
	public static TurretWheelSpeeds fromRelativeStaticAngularWheelVelocities(
		AngularVelocity lowerWheelAngularVelocity,
		double upperWheelRelativeSpeedMultiplier
	) {
		
		return new TurretWheelSpeeds(
			() -> lowerWheelAngularVelocity,
			() -> lowerWheelAngularVelocity
				.times(upperWheelRelativeSpeedMultiplier)
		);
		
	}
	
	public static TurretWheelSpeeds fromDynamicWheelSurfaceVelocities(
		Supplier<LinearVelocity> lowerWheelSurfaceVelocitySupplier,
		Supplier<LinearVelocity> upperWheelSurfaceVelocitySupplier
	) {
		
		return new TurretWheelSpeeds(
			() -> RotationsPerSecond.of(
				lowerWheelSurfaceVelocitySupplier.get().in(InchesPerSecond) /
				RobotDimensions.TURRET_LOWER_WHEEL_CIRCUMFERENCE.in(Inches)
			),
			() -> RotationsPerSecond.of(
				upperWheelSurfaceVelocitySupplier.get().in(InchesPerSecond) /
				RobotDimensions.TURRET_UPPER_WHEEL_CIRCUMFERENCE.in(Inches)
			)
		);
		
	}
	
	public static TurretWheelSpeeds fromStaticWheelSurfaceVelocities(
		LinearVelocity lowerWheelSurfaceVelocity,
		LinearVelocity upperWheelSurfaceVelocity
	) {
		
		return TurretWheelSpeeds.fromDynamicWheelSurfaceVelocities(
			() -> lowerWheelSurfaceVelocity,
			() -> upperWheelSurfaceVelocity
		);
		
	}
	
	public TurretWheelSpeeds withScaling(double scalingFactor) {
		
		return new TurretWheelSpeeds(
			() -> this.getLowerWheelAngularVelocity().times(scalingFactor),
			() -> this.getUpperWheelAngularVelocity().times(scalingFactor)
		);
		
	}
	
	public AngularVelocity getLowerWheelMotorShaftAngularVelocity() {
		
		return this.lowerWheelAngularVelocitySupplier.get()
			.div(RobotDimensions.TURRET_LOWER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT)
			.times(RobotDimensions.TURRET_LOWER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT);
		
	}
	
	public AngularVelocity getLowerWheelAngularVelocity() {
		
		return this.lowerWheelAngularVelocitySupplier.get();
		
	}
	
	public LinearVelocity getLowerWheelSurfaceSpeed() {
		
		return RobotDimensions.TURRET_LOWER_WHEEL_CIRCUMFERENCE
			.times(Hertz.of(this.getLowerWheelAngularVelocity().in(RotationsPerSecond)));
		
	}
	
	public AngularVelocity getUpperWheelMotorShaftAngularVelocity() {
		
		return this.upperWheelAngularVelocitySupplier.get()
			.div(RobotDimensions.TURRET_UPPER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT)
			.times(RobotDimensions.TURRET_UPPER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT);
		
	}
	
	public AngularVelocity getUpperWheelAngularVelocity() {
		
		return this.upperWheelAngularVelocitySupplier.get();
		
	}
	
	public LinearVelocity getUpperWheelSurfaceSpeed() {
		
		return RobotDimensions.TURRET_UPPER_WHEEL_CIRCUMFERENCE
			.times(Hertz.of(this.getUpperWheelAngularVelocity().in(RotationsPerSecond)));
		
	}
	
}
