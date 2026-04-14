package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class Vector {
	
	public static final DistanceUnit INTRINSIC_DISTANCE_UNIT = Meters;
	
	public static final AngleUnit INTRINSIC_ANGLE_UNIT = Radians;
	
	public static final Vector ZERO = new Vector(
		Vector.INTRINSIC_DISTANCE_UNIT.zero(),
		Vector.INTRINSIC_ANGLE_UNIT.zero()
	);
	
	protected final double magnitudeMeters;
	
	protected final double headingRadians;
	
	public Vector(Distance magnitude, Angle heading) {
		
		this.magnitudeMeters = magnitude.in(Vector.INTRINSIC_DISTANCE_UNIT);
		this.headingRadians = heading.in(Vector.INTRINSIC_ANGLE_UNIT);
		
	}
	
	public Vector(Distance x, Distance y) {
		
		this.magnitudeMeters = Math.sqrt(
			Math.pow(x.in(Vector.INTRINSIC_DISTANCE_UNIT), 2) +
			Math.pow(y.in(Vector.INTRINSIC_DISTANCE_UNIT), 2)
		);
		
		this.headingRadians = Math.atan2(
			y.in(Vector.INTRINSIC_DISTANCE_UNIT),
			x.in(Vector.INTRINSIC_DISTANCE_UNIT)
		);
		
	}
	
	public Vector(Translation2d point) {
		
		this(
			point.getMeasureX(),
			point.getMeasureY()
		);
		
	}
	
	public Vector(Pose2d pose) {
		
		this(pose.getTranslation());
		
	}
	
	public Vector(ChassisSpeeds chassisSpeeds) {
		
		this(
			Meters.of(chassisSpeeds.vxMetersPerSecond),
			Meters.of(chassisSpeeds.vyMetersPerSecond)
		);
		
	}
	
	public static Vector average(Vector... vectors) {
		
		Vector sum = Stream.of(vectors)
			.reduce(Vector.ZERO, Vector::plus);
		
		return sum.dividedBy(vectors.length);
		
	}
	
	public Distance getMagnitude() {
		
		return Vector.INTRINSIC_DISTANCE_UNIT.of(this.magnitudeMeters);
		
	}
	
	public Angle getHeading() {
		
		return Vector.INTRINSIC_ANGLE_UNIT.of(this.headingRadians);
		
	}
	
	public Distance getXComponent() {
		
		return Vector.INTRINSIC_DISTANCE_UNIT.of(
			this.magnitudeMeters * Math.cos(this.headingRadians)
		);
		
	}
	
	public Distance getYComponent() {
		
		return Vector.INTRINSIC_DISTANCE_UNIT.of(
			this.magnitudeMeters * Math.sin(this.headingRadians)
		);
		
	}
	
	/**
	 * Returns a new Vector with the same heading as this vector but with the
	 * specified magnitude.
	 * 
	 * @param magnitude The magnitude for the resulting vector.
	 * @return A new Vector with the same heading as this vector but with the
	 * specified magnitude.
	 */
	public Vector withMagnitude(Distance magnitude) {
		
		return new Vector(magnitude, this.getHeading());
		
	}
	
	/**
	 * Returns a new Vector with the same heading as this vector but with a
	 * magnitude that is at least the specified minimum magnitude.
	 * 
	 * If this vector's magnitude is already greater than or equal to the
	 * specified minimum magnitude, then this vector is returned unchanged.
	 * 
	 * @param minimumMagnitude The minimum magnitude for the resulting vector.
	 * @return A new Vector with the same heading as this vector but with a
	 * magnitude that is at least the specified minimum magnitude.
	 */
	public Vector withMinimumMagnitude(Distance minimumMagnitude) {
		
		return this.getMagnitude().lt(minimumMagnitude)
				? this.withMagnitude(minimumMagnitude)
				: this;
		
	}
	
	/**
	 * Returns a new Vector with the same heading as this vector but with a
	 * magnitude that is at most the specified maximum magnitude.
	 * 
	 * If this vector's magnitude is already less than or equal to the specified
	 * maximum magnitude, then this vector is returned unchanged.
	 * 
	 * @param maximumMagnitude The maximum magnitude for the resulting vector.
	 * @return A new Vector with the same heading as this vector but with a
	 * magnitude that is at most the specified maximum magnitude.
	 */
	public Vector withMaximumMagnitude(Distance maximumMagnitude) {
		
		return this.getMagnitude().gt(maximumMagnitude)
				? this.withMagnitude(maximumMagnitude)
				: this;
		
	}
	
	/**
	 * Returns a new Vector with the same heading as this vector but with a
	 * magnitude that is at least the specified minimum magnitude and at most
	 * the specified maximum magnitude.
	 * 
	 * If this vector's magnitude is already between the specified minimum and
	 * maximum magnitudes (inclusive), then this vector is returned unchanged.
	 * 
	 * @param minimumMagnitude The minimum magnitude for the resulting vector.
	 * @param maximumMagnitude The maximum magnitude for the resulting vector.
	 * @return A new Vector with the same heading as this vector but with a
	 * magnitude that is at least the specified minimum magnitude and at most
	 * the specified maximum magnitude.
	 * @see Vector#withMinimumMagnitude(Distance)
	 * @see Vector#withMaximumMagnitude(Distance)
	 */
	public Vector withClampedMagnitude(
		Distance minimumMagnitude,
		Distance maximumMagnitude
	) {
		
		return this
			.withMinimumMagnitude(minimumMagnitude)
			.withMaximumMagnitude(maximumMagnitude);
		
	}
	
	public Vector withXComponent(Distance xComponent) {
		
		return new Vector(
			xComponent,
			this.getYComponent()
		);
		
	}
	
	public Vector withYComponent(Distance yComponent) {
		
		return new Vector(
			this.getXComponent(),
			yComponent
		);
		
	}
	
	public Vector withHeading(Angle heading) {
		
		return new Vector(this.getMagnitude(), heading);
		
	}
	
	public Vector plus(Vector other) {
		
		return new Vector(
			this.getXComponent().plus(other.getXComponent()),
			this.getYComponent().plus(other.getYComponent())
		);
		
	}
	
	public Vector minus(Vector other) {
		
		return new Vector(
			this.getXComponent().minus(other.getXComponent()),
			this.getYComponent().minus(other.getYComponent())
		);
		
	}
	
	public Vector times(double scalar) {
		
		return new Vector(
			this.getMagnitude().times(scalar),
			this.getHeading()
		);
		
	}
	
	public Vector dividedBy(double scalar) {
		
		return new Vector(
			this.getMagnitude().div(scalar),
			this.getHeading()
		);
		
	}
	
	public Vector rotatedBy(Angle angle) {
		
		return new Vector(
			this.getMagnitude(),
			this.getHeading().plus(angle)
		);
		
	}
	
	/**
	 * Returns the vector that extends from this vector to the specified other
	 * vector, i.e. the vector that, when added to this vector, results in
	 * the other vector.
	 * 
	 * @param other The other vector to which to calculate the extending vector.
	 * @return The vector that extends from this vector to the specified other
	 * vector.
	 * @see Vector#minus(Vector)
	 */
	public Vector vectorTo(Vector other) {
		
		return other.minus(this);
		
	}
	
	/**
	 * Returns the distance between this vector and the specified other vector,
	 * i.e. the magnitude of the vector that extends from this vector to the
	 * other vector.
	 * 
	 * @param other The other vector to which to calculate the distance.
	 * @return The distance between this vector and the specified other vector.
	 * @see Vector#vectorTo(Vector)
	 */
	public Distance distanceTo(Vector other) {
		
		return this.vectorTo(other).getMagnitude();
		
	}
	
	/**
	 * Returns the angle between this vector and the specified other vector,
	 * i.e. the heading of the vector that extends from this vector to the other
	 * vector.
	 * 
	 * @param other The other vector to which to calculate the heading.
	 * @return The angle between this vector and the specified other vector.
	 * @see Vector#vectorTo(Vector)
	 */
	public Angle headingTo(Vector other) {
		
		return this.vectorTo(other).getHeading();
		
	}
	
	public Vector projectionOnto(Vector other) {
		
		double angleToOther = this.headingTo(other).in(Radians);
		
		return new Vector(
			this.getMagnitude().times(Math.cos(angleToOther)),
			this.getHeading()
		);
		
	}
	
	public Vector rejectionFrom(Vector other) {
		
		double angleToOther = this.headingTo(other).in(Radians);
		
		return new Vector(
			this.getMagnitude().times(Math.sin(angleToOther)),
			this.getHeading().plus(Degrees.of(90))
		);
		
	}
	
	public Vector isolateMagnitudeTangentTo(Vector other) {
		
//		Angle rotation = other.getHeading();
//		
//		return this.rotatedBy(rotation.times(-1))
//			.withXComponent(Inches.of(0))
//			.rotatedBy(rotation);
		
		double magSq = other.getXComponent().in(Meters) * other.getXComponent().in(Meters)
			+ other.getYComponent().in(Meters) * other.getYComponent().in(Meters);
		
		if (magSq == 0) return Vector.ZERO;
		
		double scale = (this.getXComponent().in(Meters) * other.getXComponent().in(Meters) +
			this.getYComponent().in(Meters) * other.getYComponent().in(Meters)) / magSq;
		
		return new Vector(
			Meters.of(this.getXComponent().in(Meters) - other.getXComponent().in(Meters) * scale),
			Meters.of(this.getYComponent().in(Meters) - other.getYComponent().in(Meters) * scale)
		);
		
	}
	
	/**
	 * Returns this vector represented as a Point, where the x and y components
	 * of the Point correspond to the x and y components of this vector,
	 * respectively.
	 * 
	 * @return This vector represented as a Point.
	 */
	public Point asPoint() {
		
		return new Point(this.getXComponent(), this.getYComponent());
		
	}
	
	/**
	 * Returns this vector represented as a ChassisSpeeds, where the x and y
	 * components of the ChassisSpeeds correspond to the x and y components of
	 * this vector, respectively, and the angular velocity component of the
	 * ChassisSpeeds is set to the specified angular velocity in radians per
	 * second.
	 * 
	 * @param angularVelocityRadiansPerSecond The angular velocity to set in the
	 * resulting ChassisSpeeds, in radians per second.
	 * @return This vector represented as a ChassisSpeeds, with the specified
	 * angular velocity in radians per second.
	 */
	public ChassisSpeeds asChassisSpeeds(double angularVelocityRadiansPerSecond) {
		
		return new ChassisSpeeds(
			this.getXComponent().in(Meters),
			this.getYComponent().in(Meters),
			angularVelocityRadiansPerSecond
		);
		
	}
	
	/**
	 * Returns this vector represented as a ChassisSpeeds, where the x and y
	 * components of the ChassisSpeeds correspond to the x and y components of
	 * this vector, respectively, and the angular velocity component of the
	 * ChassisSpeeds is set to the specified angular velocity.
	 * 
	 * @param angularVelocity The angular velocity to set in the resulting
	 * ChassisSpeeds, in the same coordinate system as this vector's heading.
	 * @return This vector represented as a ChassisSpeeds, with the specified
	 * angular velocity.
	 */
	public ChassisSpeeds asChassisSpeeds(AngularVelocity angularVelocity) {
		
		return this.asChassisSpeeds(angularVelocity.in(RadiansPerSecond));
		
	}
	
	/**
	 * Returns this vector represented as a ChassisSpeeds, where the x and y
	 * components of the ChassisSpeeds correspond to the x and y components of
	 * this vector, respectively, and with the angular velocity component of the
	 * ChassisSpeeds set to zero.
	 * 
	 * @return This vector represented as a ChassisSpeeds, with zero angular
	 * velocity.
	 */
	public ChassisSpeeds asChassisSpeeds() {
		
		return this.asChassisSpeeds(RadiansPerSecond.zero());
		
	}
	
	@Override
	public String toString() {
		
		return String.format(
			"Vector(%.1f in., %.1f deg.)",
			this.getMagnitude().in(Inches),
			this.getHeading().in(Degrees)
		);
		
	}
	
}
