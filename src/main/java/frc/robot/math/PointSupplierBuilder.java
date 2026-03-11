package frc.robot.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.DoublePreference;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Inches;

public class PointSupplierBuilder implements Supplier<Point> {
	
	protected final Supplier<Point> supplier;
	
	public PointSupplierBuilder(Supplier<Point> initial) {
		
		this.supplier = initial;
		
	}

	public PointSupplierBuilder(
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier
	) {
		
		this.supplier = () -> new Point(
			xSupplier.getAsDouble(),
			ySupplier.getAsDouble()
		);
		
	}

	public PointSupplierBuilder(
		Supplier<Distance> xSupplier,
		Supplier<Distance> ySupplier
	) {

		this.supplier = () -> new Point(xSupplier.get(), ySupplier.get());

	}
	
	/**
	 * Creates a new PointSupplierBuilder from the left joystick of the given
	 * XboxController.
	 *
	 * This method normalizes the joystick input to the standard FRC
	 * 'North West Up' ('NMU') coordinate system.
	 *
	 * @param controller The XboxController from which to read the left joystick
	 * input.
	 */
	public static PointSupplierBuilder fromLeftJoystick(
		XboxController controller
	) {
		
		return new PointSupplierBuilder(
			controller::getLeftX,
			controller::getLeftY
		);
		
	}
	
	public static PointSupplierBuilder fromLeftJoystick(
		CommandXboxController controller
	) {
		
		return new PointSupplierBuilder(
			controller::getLeftX,
			controller::getLeftY
		);
		
	}

	/**
	 * Returns a Translation2dSupplierBuilder for controlling the translation of
	 * the drivetrain.
	 *
	 * @param controller The XboxController from which to read the left joystick
	 * input (to use as the translation input).
	 * @return A Translation2dSupplierBuilder for controlling the translation of
	 * the drivetrain.
	 */
	public static Supplier<Point> getTranslationPointSupplier(
		CommandXboxController controller
	) {

		return PointSupplierBuilder.fromLeftJoystick(controller)
			.normalizeXboxJoystickToNWU()
			.withClamp(-1, 1)
			.withScaledDeadband(DoublePreference.JOYSTICK_DEADBAND.get())
			.withExponentialCurve(DoublePreference.LINEAR_INPUT_POWER_SMOOTHING.get());

	}
	
	public PointSupplierBuilder swapXY() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Point(original.getY(), original.getX());
			
		});
		
	}
	
	public PointSupplierBuilder invertX() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Point(-original.getX(), original.getY());
			
		});
		
	}
	
	public PointSupplierBuilder invertY() {
		
		return new PointSupplierBuilder(() -> {
			
			Translation2d original = this.supplier.get();
			
			return new Point(original.getX(), -original.getY());
			
		});
		
	}
	
	public PointSupplierBuilder normalizeXboxJoystickToNWU() {
		
		return this
			.swapXY()
			.invertX()
			.invertY();
		
	}
	
	public PointSupplierBuilder withDeadband(double deadband) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return original.getNorm() <= 0
				? new Point(Translation2d.kZero)
				: new Point(
					DoubleUtilities.applyDeadband(original.getNorm(), deadband),
					original.getAngle()
				); 
			
		});
		
	}
	
	public PointSupplierBuilder withScaledDeadband(double deadband) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return original.getNorm() <= 0
				? new Point(Translation2d.kZero)
				: new Point(
					DoubleUtilities.applyScaledDeadband(original.getNorm(), deadband),
					original.getAngle()
				);
			
		});
		
	}
	
	public PointSupplierBuilder withExponentialCurve(double exponent) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return original.getNorm() <= 0
				? new Point(Translation2d.kZero)
				: new Point(
					DoubleUtilities.applyExponentialCurve(original.getNorm(), exponent),
					original.getAngle()
				);
			
		});
		
	}
	
	public PointSupplierBuilder withClamp(double minimum, double maximum) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return original.getNorm() <= 0
				? new Point(Translation2d.kZero)
				: new Point(
					MathUtil.clamp(original.getNorm(), minimum, maximum),
					original.getAngle()
				);
			
		});
		
	}
	
	public PointSupplierBuilder withScaling(double scaling) {
		
		return new PointSupplierBuilder(() -> {
			
			Point original = this.supplier.get();
			
			return original.getNorm() <= 0
				? new Point(Translation2d.kZero)
				: new Point(
					original.getNorm() * scaling,
					original.getAngle()
				);
			
		});
		
	}
	
	public PointSupplierBuilder withMaximumSlewRate(double limit) {

		SlewRateLimiter xLimiter = new SlewRateLimiter(limit);
		SlewRateLimiter yLimiter = new SlewRateLimiter(limit);
		
		return new PointSupplierBuilder(() -> {

			Point original = PointSupplierBuilder.this.get();
			
			return new Point(
				Inches.of(xLimiter.calculate(original.getMeasureX().in(Inches))),
				Inches.of(yLimiter.calculate(original.getMeasureY().in(Inches)))
			);
			
		});
		
	}
	
	@Override
	public Point get() {
		
		return this.supplier.get();
		
	}
	
}
