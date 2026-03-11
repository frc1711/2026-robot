package frc.robot.devicewrappers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;

import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Degrees;

/**
 * A wrapper over the base {@link Pigeon2} class that provides additional
 * features.
 */
public class RaptorsPigeon2 {

	/**
	 * The units intrinsically used by the Studica/AHRS library to return
	 * angular measurements in.
	 */
	public static final AngleUnit INTRINSIC_ANGLE_UNIT = Degrees;

	/**
	 * The underlying Pigeon2 object.
	 */
	protected final Pigeon2 pigeon;

	/**
	 * The yaw axis of this gyro, representing rotation around the vertical
	 * axis. Counter-clockwise yaw is positive.
	 */
	public final Axis yaw;

	/**
	 * The pitch axis of this gyro, representing forward/backwards rotation
	 * around the left/right axis. Forward pitch is positive.
	 */
	public final Axis pitch;

	/**
	 * The roll axis of this gyro, representing left/right rotation around the
	 * forwards/backwards axis. Right roll is positive.
	 */
	public final Axis roll;

	/**
	 * Initializes a new RaptorsNavX instance. 
	 */
	public RaptorsPigeon2(CANDevice canDevice) {

		this.pigeon = new Pigeon2(canDevice.id);
		this.yaw = new Axis(this.pigeon.getYaw().asSupplier());
		this.pitch = new Axis(this.pigeon.getRoll().asSupplier());
		this.roll = new Axis(this.pigeon.getPitch().asSupplier());
		
	}

	/**
	 * Returns a Stream of the Axis of this gyro.
	 * 
	 * @return A Stream of the Axis of this gyro.
	 */
	protected Stream<Axis> getAxisStream() {
		
		return Stream.of(this.yaw, this.pitch, this.roll);
		
	}

	/**
	 * Returns the angular velocity of the yaw of this gyro.
	 * 
	 * @return The angular velocity of the yaw of this gyro.
	 */
	public AngularVelocity getYawAngularVelocity() {
		
		return this.pigeon.getAngularVelocityZDevice().getValue();
		
	}

	/**
	 * Calibrates this gyro by resetting the underlying NavX, as well as setting
	 * all axis to 'zero'.
	 * 
	 * @see RaptorsPigeon2#calibrate(boolean)
	 */
	public void calibrate() {
		
		this.calibrate(false);
		
	}

	/**
	 * Calibrates this gyro by resetting the underlying NavX.
	 * 
	 * @param preserveAngles true if the current angles of the axis of this gyro
	 * should be preserved when resetting the gyro. If set to false, all axis
	 * will/should read 'zero' after this operation.
	 * @see Axis#calibrate(Angle)
	 */
	public void calibrate(boolean preserveAngles) {
		
		Stream<Pair<Axis, Angle>> axisZeroOffsets = this.getAxisStream().map(
			(Axis axis) ->
				new Pair<Axis, Angle>(axis, axis.getAngle())
		);
		
		this.pigeon.reset();
		
		axisZeroOffsets.forEach((Pair<Axis, Angle> pair) ->
			pair.getFirst().zeroOffset = preserveAngles
				? pair.getSecond()
				: Degrees.zero()
		);
		
	}

	/**
	 * A single axis of a gyro.
	 */
	public class Axis {

		/**
		 * The {@link Supplier<Angle>} that supplies the raw angle of this axis.
		 */
		protected final Supplier<Angle> rawAngleSupplier;

		/**
		 * The angular offset between the intrinsic (raw) 'zero' and the
		 * 'calibrated zero' of this axis.
		 */
		public Angle zeroOffset;

		/**
		 * Initializes a new Axis instance, defined by the provided angle
		 * supplier.
		 * 
		 * @param rawAngleSupplier The {@link Supplier<Angle>} that supplies the
		 * raw angle of this axis.
		 */
		protected Axis(Supplier<Angle> rawAngleSupplier) {
			
			this.rawAngleSupplier = rawAngleSupplier;
			this.zeroOffset = Degrees.zero();
			
		}
		
		/**
		 * Returns the raw (uncalibrated) angle of this axis, as an
		 * {@link Angle}.
		 *
		 * @return The raw (uncalibrated) yaw angle of this axis, as an Angle.
		 */
		public Angle getRawAngle() {
			
			return this.rawAngleSupplier.get();
			
		}
		
		/**
		 * Returns the raw (uncalibrated) angle of this axis, as a
		 * {@link Rotation2d}.
		 *
		 * @return The raw (uncalibrated) angle of this axis, as a Rotation2d.
		 */
		public Rotation2d getRawRotation() {
			
			return new Rotation2d(this.getAngle());
			
		}
		
		/**
		 * Returns the calibrated angle of this axis, as an {@link Angle}.
		 *
		 * @return The calibrated angle of this axis, as an Angle.
		 */
		public Angle getAngle() {
			
			return this.getRawAngle().minus(this.zeroOffset);
			
		}
		
		/**
		 * Returns the calibrated angle of this axis, as a {@link Rotation2d}.
		 *
		 * @return The calibrated angle of this axis, as a Rotation2d.
		 */
		public Rotation2d getRotation() {
			
			return new Rotation2d(this.getAngle());
			
		}
		
		/**
		 * Calibrates the underlying gyro of this Axis and 'zeroes' this axis.
		 * 
		 * @see Axis#calibrate(Angle)
		 */
		public void calibrate() {
			
			this.calibrate(Degrees.zero());
			
		}
		
		/**
		 * Calibrates the underlying gyro of this Axis and adjusts the zero
		 * offset such that this Axis will/should read at the specified angle
		 * after this operation completes.
		 * 
		 * @param currentAngle The 'desired current angle', or in other words,
		 * the angle that this Axis should report after this operation
		 * completes.
		 */
		public void calibrate(Angle currentAngle) {
			
			RaptorsPigeon2.this.calibrate();
			this.zeroOffset = currentAngle;
			
		}
		
		public Trigger getGreaterThanTrigger(Angle minimum) {
			
			return new Trigger(() -> this.getAngle().gt(minimum));
			
		}
		
		public Trigger getGreaterThanOrEqualTrigger(Angle minimum) {
			
			return new Trigger(() -> this.getAngle().gte(minimum));
			
		}
		
		public Trigger getLessThanTrigger(Angle maximum) {
			
			return new Trigger(() -> this.getAngle().lt(maximum));
			
		}

		public Trigger getLessThanOrEqualTrigger(Angle maximum) {

			return new Trigger(() -> this.getAngle().lte(maximum));

		}
		
		public Trigger getNearTrigger(Angle angle, Angle tolerance) {
			
			return new Trigger(() -> this.getAngle().isNear(angle, tolerance));
			
		}

		public Trigger getBetweenTrigger(Angle minimum, Angle maximum) {
			
			return new Trigger(() -> {
				Angle angle = this.getAngle();
				return angle.gte(minimum) && angle.lte(maximum);
			});
			
		}
		
	}
	
}
