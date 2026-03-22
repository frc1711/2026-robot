package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class HeadingLock {
	
	protected static final AngleUnit INTRINSIC_ANGLE_UNITS = Degrees;
	
	protected static final AngularVelocityUnit INTRINSIC_ANGULAR_VELOCITY_UNIT =
		HeadingLock.INTRINSIC_ANGLE_UNITS.per(Second);
	
	protected static final AngularAccelerationUnit INTRINSIC_ANGULAR_ACCELERATION_UNIT =
		HeadingLock.INTRINSIC_ANGULAR_VELOCITY_UNIT.per(Second);
	
	protected static final double CONTROLLER_KP = 1;
	
	protected static final double CONTROLLER_KI = 0;
	
	protected static final double CONTROLLER_KD = 0;
	
	protected static final AngularVelocity MAX_ANGULAR_VELOCITY =
		RotationsPerSecond.of(0.75);
	
	protected static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
		RotationsPerSecondPerSecond.of(1.5);
	
	protected final Swerve swerve;
	
	protected final ProfiledPIDController controller;
	
	protected Supplier<Angle> headingSupplier;
	
	protected double lastPIDValue;
	
	public HeadingLock(Swerve swerve) {
		
		this.swerve = swerve;
		this.controller = new ProfiledPIDController(
			HeadingLock.CONTROLLER_KP,
			HeadingLock.CONTROLLER_KI,
			HeadingLock.CONTROLLER_KD,
			new TrapezoidProfile.Constraints(
				HeadingLock.MAX_ANGULAR_VELOCITY
					.in(HeadingLock.INTRINSIC_ANGULAR_VELOCITY_UNIT),
				HeadingLock.MAX_ANGULAR_ACCELERATION
					.in(HeadingLock.INTRINSIC_ANGULAR_ACCELERATION_UNIT)
			)
		);
		this.headingSupplier = null;
		
		this.controller.enableContinuousInput(
			Rotations.of(-0.5).in(HeadingLock.INTRINSIC_ANGLE_UNITS),
			Rotations.of(0.5).in(HeadingLock.INTRINSIC_ANGLE_UNITS)
		);
		
	}
	
	public void enable(Supplier<Angle> headingSupplier) {
		
		this.headingSupplier = headingSupplier;
		
		this.controller.reset(
			this.swerve.getFieldRelativeHeading().in(HeadingLock.INTRINSIC_ANGLE_UNITS),
			this.swerve.getAngularVelocity().in(HeadingLock.INTRINSIC_ANGULAR_VELOCITY_UNIT)
		);
		
	}
	
	public void disable() {
		
		this.headingSupplier = null;
		
	}
	
	public boolean isEnabled() {
		
		return this.headingSupplier != null;
		
	}
	
	public Angle getHeading() {
		
		return this.headingSupplier != null
			? this.headingSupplier.get()
			: null;
		
	}
	
	public boolean hasLock(Angle tolerance) {
		
		if (this.headingSupplier == null) return false;
		
		return this.swerve.getFieldRelativeHeading().isNear(
			this.headingSupplier.get(),
			tolerance
		);
		
	}
	
	public boolean hasLock() {
		
		return this.hasLock(Degrees.of(10));
		
	}
	
	public void periodic() {
		
		if (this.headingSupplier == null) return;
		
		this.lastPIDValue = this.controller.calculate(
			this.swerve.getFieldRelativeHeading().in(HeadingLock.INTRINSIC_ANGLE_UNITS),
			this.headingSupplier.get().in(HeadingLock.INTRINSIC_ANGLE_UNITS)
		);
		
	}
	
	public ChassisSpeeds apply(ChassisSpeeds chassisSpeeds) {
		
		if (!this.isEnabled()) return chassisSpeeds;
		
		AngularVelocity setpointVelocity =
			HeadingLock.INTRINSIC_ANGULAR_VELOCITY_UNIT
				.of(this.lastPIDValue + this.controller.getSetpoint().velocity);
		
		return new ChassisSpeeds(
			chassisSpeeds.vxMetersPerSecond,
			chassisSpeeds.vyMetersPerSecond,
			setpointVelocity.in(RadiansPerSecond)
		);
		
	}
	
}
