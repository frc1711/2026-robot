package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import frc.robot.math.Vector;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class RadiusLock {
	
	protected static final DistanceUnit INTRINSIC_DISTANCE_UNITS = Feet;
	
	protected static final LinearVelocityUnit INTRINSIC_LINEAR_VELOCITY_UNIT =
		RadiusLock.INTRINSIC_DISTANCE_UNITS.per(Second);
	
	protected static final LinearAccelerationUnit INTRINSIC_LINEAR_ACCELERATION_UNIT =
		RadiusLock.INTRINSIC_LINEAR_VELOCITY_UNIT.per(Second);
	
	protected static final double CONTROLLER_KP = 1;
	
	protected static final double CONTROLLER_KI = 0;
	
	protected static final double CONTROLLER_KD = 0;
	
	protected static final LinearVelocity MAX_LINEAR_VELOCITY =
		FeetPerSecond.of(2);
	
	protected static final LinearAcceleration MAX_LINEAR_ACCELERATION =
		FeetPerSecondPerSecond.of(4);
	
	protected final Swerve swerve;
	
	protected final ProfiledPIDController controller;
	
	protected Supplier<Translation2d> centerPointSupplier;
	
	protected Supplier<Distance> radiusSupplier;
	
	protected double lastPIDValue;
	
	public RadiusLock(Swerve swerve) {
		
		this.swerve = swerve;
		this.controller = new ProfiledPIDController(
			RadiusLock.CONTROLLER_KP,
			RadiusLock.CONTROLLER_KI,
			RadiusLock.CONTROLLER_KD,
			new TrapezoidProfile.Constraints(
				RadiusLock.MAX_LINEAR_VELOCITY
					.in(RadiusLock.INTRINSIC_LINEAR_VELOCITY_UNIT),
				RadiusLock.MAX_LINEAR_ACCELERATION
					.in(RadiusLock.INTRINSIC_LINEAR_ACCELERATION_UNIT)
			)
		);
		this.centerPointSupplier = null;
		this.radiusSupplier = null;
		
	}
	
	public void enable(
		Supplier<Translation2d> centerPointSupplier,
		Supplier<Distance> radiusSupplier
	) {
		
		this.centerPointSupplier = centerPointSupplier;
		this.radiusSupplier = radiusSupplier;
		
		this.controller.reset(
			this.getActualRadius().in(RadiusLock.INTRINSIC_DISTANCE_UNITS)
		);
		
	}
	
	public void disable() {
		
		this.centerPointSupplier = null;
		this.radiusSupplier = null;
		
	}
	
	public boolean isEnabled() {
		
		return (
			this.centerPointSupplier != null &&
			this.radiusSupplier != null
		);
		
	}
	
	public Distance getActualRadius() {
		
		if (this.centerPointSupplier == null) return Meters.zero();
		
		return Meters.of(
			this.centerPointSupplier.get().getDistance(this.swerve.getOdometry().getTranslation())
		);
		
	}
	
	public Distance getError() {
		
		return !this.isEnabled()
			? RadiusLock.INTRINSIC_DISTANCE_UNITS.zero()
			: this.getActualRadius().minus(this.radiusSupplier.get());
		
	}
	
	public boolean hasLock(Distance tolerance) {
		
		if (this.radiusSupplier == null) return false;
		
		return this.getActualRadius().isNear(
			this.radiusSupplier.get(),
			tolerance
		);
		
	}
	
	public boolean hasLock() {
		
		return this.hasLock(Inches.of(3));
		
	}
	
	public void periodic() {
		
		if (
			this.centerPointSupplier == null ||
			this.radiusSupplier == null
		) return;
		
		this.lastPIDValue = this.controller.calculate(
			this.getActualRadius().in(RadiusLock.INTRINSIC_DISTANCE_UNITS),
			this.radiusSupplier.get().in(RadiusLock.INTRINSIC_DISTANCE_UNITS)
		);
		
	}
	
	public ChassisSpeeds apply(ChassisSpeeds chassisSpeeds) {
		
		if (!this.isEnabled()) return chassisSpeeds;
		
		Rotation2d robotHeading =
			new Rotation2d(this.swerve.getFieldRelativeHeading());
		Vector centerPoint = new Vector(this.centerPointSupplier.get());
		
		ChassisSpeeds originalFieldRelativeSpeeds =
			ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotHeading);
		Vector originalFieldRelativeSpeedsVector =
			new Vector(originalFieldRelativeSpeeds);
		
		ChassisSpeeds tangentialSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			originalFieldRelativeSpeedsVector
				.isolateMagnitudeTangentTo(centerPoint)
				.asChassisSpeeds(originalFieldRelativeSpeeds.omegaRadiansPerSecond),
			robotHeading
		);
		
		LinearVelocity controllerCorrectionVelocity =
			RadiusLock.INTRINSIC_LINEAR_VELOCITY_UNIT.of(this.lastPIDValue);
		LinearVelocity controllerSetpointVelocity =
			RadiusLock.INTRINSIC_LINEAR_VELOCITY_UNIT
				.of(this.controller.getSetpoint().velocity);
		LinearVelocity controllerTotalVelocity = controllerCorrectionVelocity
			.plus(controllerSetpointVelocity);

		ChassisSpeeds correctiveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			new Vector(this.swerve.getOdometry().getTranslation())
				.vectorTo(centerPoint)
				.withMagnitude(controllerTotalVelocity.times(Seconds.of(-1)))
				.asChassisSpeeds(),
			new Rotation2d(swerve.getFieldRelativeHeading())
		);
		
		correctiveSpeeds.omegaRadiansPerSecond =
			originalFieldRelativeSpeeds.omegaRadiansPerSecond;
		
		return correctiveSpeeds;
		
	}
	
}
