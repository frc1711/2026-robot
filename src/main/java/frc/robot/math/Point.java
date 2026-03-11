package frc.robot.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * A wrapper over the WPI build-in Translation2d class that provides additional
 * functionality.
 */
public class Point extends Translation2d {

    /**
     * The units intrinsically used by the underlying Translation2d object.
     */
    private static final DistanceUnit INTRINSIC_MEASUREMENT_UNIT = Meters;

    /**
     * Initializes a new Point instance at 0,0.
     */
    public Point() {
        
        super();
        
    }

    /**
     * Initializes a new Point instance by cloning the provided point.
     * 
     * @param point The point object to clone.
     */
    public Point(Translation2d point) {
        
        super(point.getX(), point.getY());
        
    }

    /**
     * Initializes a new Point instance with the specified coordinates.
     * 
     * @param xMetersPerSecond The x coordinate of the resultant Point.
     * @param yMetersPerSecond The y coordinate of the resultant Point.
     */
    public Point(double xMetersPerSecond, double yMetersPerSecond) {
        
        super(xMetersPerSecond, yMetersPerSecond);
        
    }

    /**
     * Initializes a new Point instance with the specified coordinates.
     * 
     * @param x The x coordinate of the resultant Point.
     * @param y The y coordinate of the resultant Point.
     */
    public Point(Distance x, Distance y) {
        
        super(x, y);
        
    }

    /**
     * Initializes a new Point instance with the provided norm and rotation.
     * 
     * @param normMeters The norm of the resultant Point.
     * @param angle The rotation of the resultant Point relative to the
     * positive x-axis (i.e. 0 degrees represents an angle that points 'to the
     * right' of a standard coordinate graph).
     */
    public Point(double normMeters, Rotation2d angle) {
        
        super(normMeters, angle);
        
    }

    /**
     * Initializes a new Point instance with the provided norm and rotation.
     * 
     * @param norm The norm of the resultant Point.
     * @param rotation The rotation of the resultant Point relative to the
     * positive x-axis (i.e. 0 degrees represents an angle that points 'to the
     * right' of a standard coordinate graph).
     */
    public Point(Distance norm, Rotation2d rotation) {
        
        super(norm.in(Point.INTRINSIC_MEASUREMENT_UNIT), rotation);
        
    }

    /**
     * Initializes a new Point instance with the provided norm and angle.
     * 
     * @param norm The norm of the resultant Point.
     * @param angle The angle of the resultant Point relative to the
     * positive x-axis (i.e. 0 degrees represents an angle that points 'to the
     * right' of a standard coordinate graph).
     */
    public Point(Distance norm, Angle angle) {
        
        this(norm, new Rotation2d(angle));
        
    }

    /**
     * Initializes a new Point instance using the vx and vy components of the
     * provided ChassisSpeeds object.
     * 
     * The angular velocity of the provided ChassisSpeeds object is discarded.
     * 
     * @param chassisSpeeds The ChassisSpeeds object from which to extract and
     * use the vx and vy components.
     */
    public Point(ChassisSpeeds chassisSpeeds) {
        
        super(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        
    }

    /**
     * Returns the average point of a list of points.
     * 
     * @param points The points to average.
     * @return The average point of the given list of points.
     */
    public static Point average(Translation2d ...points) {
        
        return Stream.of(points).reduce(
            (Translation2d point, Translation2d acc) -> new Translation2d(
                point.getX() + acc.getX(),
                point.getY() + acc.getY()
            )
        ).map((Translation2d value) -> new Point(value.div(points.length)))
            .orElseGet(Point::new);
        
    }
    
    /**
     * Returns a new Point that has been scaled such that the resulting norm is
     * equal to the specified norm.
     * 
     * @param norm The desired norm of the resultant Point.
     * @return A new Point that has been scaled such that the resulting norm is
     * equal to the specified norm.
     */
    public Point setNorm(Distance norm) {
        
        double original = this.getNorm();
        
        return Double.isNaN(original) || original == 0
            ? this
            : new Point(this.times(
                norm.in(Point.INTRINSIC_MEASUREMENT_UNIT) / original
            ));
        
    }
    
    /**
     * Returns a new {@link Translation2d} that is normalized to the specified
     * maximum norm.
     *
     * In other words, if the norm of this translation is greater than maxNorm,
     * the returned translation will have a norm of maxNorm, otherwise it will
     * be unchanged.
     *
     * @param maxNorm The maximum norm to which to normalize the input.
     * @return A new {@link Point} that is normalized to the specified maximum
     * norm.
     */
    public Point applyMaxNorm(Distance maxNorm) {

        return new Point(this.times(Math.min(
            maxNorm.in(Point.INTRINSIC_MEASUREMENT_UNIT)/this.getNorm(),
            1
        )));

    }

    /**
     * Converts and returns this point to a ChassisSpeeds object, using the
     * given angular velocity.
     *
     * @param angularVelocity The value to use for the rotational speed of
     * the resultant ChassisSpeeds object.
     * @return A ChassisSpeeds object built from this point using the provided
     * angular velocity.
     */
    public ChassisSpeeds toChassisSpeeds(AngularVelocity angularVelocity) {

        return this.toChassisSpeeds(angularVelocity.in(RadiansPerSecond));

    }

    /**
     * Converts and returns this point to a ChassisSpeeds object, using the
     * given omegaRadiansPerSecond value.
     * 
     * @param omegaRadiansPerSecond The value to use for the rotational speed of
     * the resultant ChassisSpeeds object.
     * @return A ChassisSpeeds object built from this point using the provided
     * omegaRadiansPerSecond value.
     */
    public ChassisSpeeds toChassisSpeeds(double omegaRadiansPerSecond) {
        
        return new ChassisSpeeds(
            this.getX(),
            this.getY(),
            omegaRadiansPerSecond
        );
        
    }
    
}
