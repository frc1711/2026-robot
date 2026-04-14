package frc.robot.state;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.configuration.RobotDimensions;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class FlyWheelSpeeds {
	
	public static final FlyWheelSpeeds STOPPED =
		FlyWheelSpeeds.fromStaticAngularWheelVelocities(
			RotationsPerSecond.zero(),
			RotationsPerSecond.zero()
		);
	
	public static final FlyWheelSpeeds CLOSE_SHOT =
		FlyWheelSpeeds.fromStaticWheelSurfaceVelocities(
			FeetPerSecond.of(14.5),
			FeetPerSecond.of(14.5).plus(FeetPerSecond.of(20))
		);
	
	public static final FlyWheelSpeeds MID_SHOT =
		FlyWheelSpeeds.fromStaticWheelSurfaceVelocities(
			FeetPerSecond.of(17.5),
			FeetPerSecond.of(17.5).plus(FeetPerSecond.of(20))
		);
	
	public static final FlyWheelSpeeds FAR_SHOT =
		FlyWheelSpeeds.fromStaticWheelSurfaceVelocities(
			FeetPerSecond.of(30),
			FeetPerSecond.of(30).plus(FeetPerSecond.of(15))
		);
	
	Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier;
	
	Supplier<AngularVelocity> upperWheelAngularVelocitySupplier;
	
	public FlyWheelSpeeds(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
	) {
		
		this.lowerWheelAngularVelocitySupplier = lowerWheelAngularVelocitySupplier;
		this.upperWheelAngularVelocitySupplier = upperWheelAngularVelocitySupplier;
		
	}
	
	public static FlyWheelSpeeds fromDynamicAngularMotorShaftVelocities(
		Supplier<AngularVelocity> lowerMotorShaftAngularVelocitySupplier,
		Supplier<AngularVelocity> upperMotorShaftAngularVelocitySupplier
	) {
		
		return new FlyWheelSpeeds(
			() -> lowerMotorShaftAngularVelocitySupplier.get(),
			() -> upperMotorShaftAngularVelocitySupplier.get()
		);
		
	}
	
	public static FlyWheelSpeeds fromDynamicAngularWheelVelocities(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
	) {
		
		return new FlyWheelSpeeds(
			lowerWheelAngularVelocitySupplier,
			upperWheelAngularVelocitySupplier
		);
		
	}
	
	public static FlyWheelSpeeds fromStaticAngularWheelVelocities(
		AngularVelocity lowerWheelAngularVelocity,
		AngularVelocity upperWheelAngularVelocity
	) {
		
		return new FlyWheelSpeeds(
			() -> lowerWheelAngularVelocity,
			() -> upperWheelAngularVelocity
		);
		
	}
	
	public static FlyWheelSpeeds fromRelativeDynamicAngularWheelVelocities(
		Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
		DoubleSupplier upperWheelRelativeSpeedMultiplierSupplier
	) {
		
		return new FlyWheelSpeeds(
			lowerWheelAngularVelocitySupplier,
			() -> lowerWheelAngularVelocitySupplier.get()
				.times(upperWheelRelativeSpeedMultiplierSupplier.getAsDouble())
		);
		
	}
	
	public static FlyWheelSpeeds fromRelativeStaticAngularWheelVelocities(
		AngularVelocity lowerWheelAngularVelocity,
		double upperWheelRelativeSpeedMultiplier
	) {
		
		return new FlyWheelSpeeds(
			() -> lowerWheelAngularVelocity,
			() -> lowerWheelAngularVelocity
				.times(upperWheelRelativeSpeedMultiplier)
		);
		
	}
	
	public static FlyWheelSpeeds fromDynamicWheelSurfaceVelocities(
		Supplier<LinearVelocity> lowerWheelSurfaceVelocitySupplier,
		Supplier<LinearVelocity> upperWheelSurfaceVelocitySupplier
	) {
		
		return new FlyWheelSpeeds(
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
	
	public static FlyWheelSpeeds fromStaticWheelSurfaceVelocities(
		LinearVelocity lowerWheelSurfaceVelocity,
		LinearVelocity upperWheelSurfaceVelocity
	) {
		
		return FlyWheelSpeeds.fromDynamicWheelSurfaceVelocities(
			() -> lowerWheelSurfaceVelocity,
			() -> upperWheelSurfaceVelocity
		);
		
	}
	
	public FlyWheelSpeeds withScaling(double scalingFactor) {
		
		return new FlyWheelSpeeds(
			() -> this.getLowerWheelAngularVelocity().times(scalingFactor),
			() -> this.getUpperWheelAngularVelocity().times(scalingFactor)
		);
		
	}
	
	public AngularVelocity getLowerWheelMotorShaftAngularVelocity() {
		
		return this.lowerWheelAngularVelocitySupplier.get()
			.times(RobotDimensions.TURRET_LOWER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT)
			.div(RobotDimensions.TURRET_LOWER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT);
		
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
			.times(RobotDimensions.TURRET_UPPER_WHEEL_DRIVEN_PULLEY_TOOTH_COUNT)
			.div(RobotDimensions.TURRET_UPPER_WHEEL_DRIVING_PULLEY_TOOTH_COUNT);
		
	}
	
	public AngularVelocity getUpperWheelAngularVelocity() {
		
		return this.upperWheelAngularVelocitySupplier.get();
		
	}
	
	public LinearVelocity getUpperWheelSurfaceSpeed() {
		
		return RobotDimensions.TURRET_UPPER_WHEEL_CIRCUMFERENCE
			.times(Hertz.of(this.getUpperWheelAngularVelocity().in(RotationsPerSecond)));
		
	}
	
}
