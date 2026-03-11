package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A collection of utility methods for manipulating {@link Translation2d}
 * objects.
 */
public class Translation2dUtilities {

	// Prevent instantiation of this utility class.
	private Translation2dUtilities() {}

	/**
	 * Returns a new {@link Translation2d} that is normalized to the specified
	 * maximum norm.
	 *
	 * In other words, if the norm of the input translation is greater than
	 * maxNorm, the returned translation will have a norm of maxNorm,
	 * otherwise it will be unchanged.
	 *
	 * @param translation The input {@link Translation2d} to normalize.
	 * @param maxNorm The maximum norm to which to normalize the input.
	 * @return A new {@link Translation2d} that is normalized to the specified
	 * maximum norm.
	 */
	public static Translation2d applyMaxNorm(
		Translation2d translation,
		double maxNorm
	) {

		return translation.times(Math.min(maxNorm/translation.getNorm(), 1));

	}
    
}
