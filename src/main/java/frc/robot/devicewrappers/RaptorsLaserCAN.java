package frc.robot.devicewrappers;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.Millimeters;

/**
 * A wrapper over the base {@link LaserCan} class that provides additional
 * features.
 */
public class RaptorsLaserCAN {

	/**
	 * The units intrinsically used by the LaserCAN library to return
	 * measurements in.
	 */
	public static final DistanceUnit INTRINSIC_DISTANCE_UNIT = Millimeters;

	/**
	 * The underlying {@link LaserCan} object.
	 */
	protected final LaserCan laserCAN;

	/**
	 * Initializes a new RaptorsLaserCAN instance with the given CANDevice.
	 * 
	 * @param canDevice The CANDevice instance representing the LaserCAN device
	 * to represent with this instance.   
	 */
	public RaptorsLaserCAN(CANDevice canDevice) {
		
		this.laserCAN = new LaserCan(canDevice.id);
		
	}

	/**
	 * Sets the ranging mode of this LaserCAN to the given mode.
	 * 
	 * @param rangingMode The ranging mode to configure this LaserCAN with.
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setRangingMode(
		LaserCanInterface.RangingMode rangingMode
	) {
		
		try {
			
			this.laserCAN.setRangingMode(rangingMode);
			
		} catch (ConfigurationFailedException e) {
			
			throw new RuntimeException(e);
			
		}
		
		return this;
		
	}

	/**
	 * Sets the ranging mode of this LaserCAN to the 'short' mode, wherein
	 * objects can be detected within a range of ~1.3 meters (~51 inches) of the
	 * device.
	 * 
	 * This ranging mode should be preferred when possible, as it is more
	 * accurate and less susceptible to variation/noise caused by ambient light.
	 * 
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setRangingModeToShort() {
		
		return this.setRangingMode(LaserCanInterface.RangingMode.SHORT);

    }
	
	/**
	 * Sets the ranging mode of this LaserCAN to the 'long' mode, wherein
	 * objects can be detected within a range of ~4 meters (~157 inches) of the
	 * device.
	 * 
	 * This ranging mode should only be used when the 'short' ranging mode is
	 * insufficient, as it is less accurate and more susceptible to measurement
	 * variation/noise caused by ambient light.
	 * 
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setRangingModeToLong() {
		
		return this.setRangingMode(LaserCanInterface.RangingMode.LONG);

	}

	/**
	 * Sets the timing budget of this LaserCAN to the given value in
	 * milliseconds.
	 * 
	 * Higher timing budgets provide more accurate and repeatable results,
	 * however at a lower rate than smaller timing budgets.
	 * 
	 * @param budgetMs The timing budget, in milliseconds, to configure this
	 * LaserCAN device with.   
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setTimingBudget(int budgetMs) {

		LaserCanInterface.TimingBudget budget;
		
		if (budgetMs < 30) budget = LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS;
		else if (budgetMs < 45) budget = LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS;
		else if (budgetMs < 80) budget = LaserCanInterface.TimingBudget.TIMING_BUDGET_50MS;
		else budget = LaserCanInterface.TimingBudget.TIMING_BUDGET_100MS;
		
		try {
			
			this.laserCAN.setTimingBudget(budget);
			
		} catch (ConfigurationFailedException e) {
			
			throw new RuntimeException(e);
			
		}
		
		return this;
		
	}

	/**
	 * Sets the region of interest of this LaserCAN, which controls the portion
	 * of the total LaserCAN sensor inside which obstruction detection can
	 * occur.
	 * 
	 * @param x The x (left/right) axis coordinate for where the center of the
	 * given region-of-interest should be centered. Input values will be clamped
	 * to the range [0, 16].   
	 * @param y The y (up/down) axis coordinate for where the center of the
	 * given region-of-interest should be centered. Input values will be clamped
	 * to the range [0, 16].
	 * @param w The width of the rectangular region-of-interest to set. Input
	 * values will be clamped to the range [0, 16].
	 * @param h The height of the rectangular region-of-interest to set. Input
	 * values will be clamped to the range [0, 16].
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setRegionOfInterest(
		int x, int y, int w, int h
	) {
		
		try {

			this.laserCAN.setRegionOfInterest(
				new LaserCanInterface.RegionOfInterest(
					MathUtil.clamp(x, 0, 16),
					MathUtil.clamp(y, 0, 16),
					MathUtil.clamp(w, 0, 16),
					MathUtil.clamp(h, 0, 16)
				)
			);

		} catch (ConfigurationFailedException e) {

			throw new RuntimeException(e);

		}
		
		return this;
		
	}

	/**
	 * Sets the region of interest of this LaserCAN to a square of size 2n
	 * (where n is the input value) centered on the sensor.
	 * 
	 * @param size The relative size of the square region of interest to set.
	 * Valid inputs are integers 0 through 8. Inputs outside of that range will
	 * be 'clamped' down to range.
	 * @return This RaptorsLaserCAN instance for chained calls.
	 */
	public RaptorsLaserCAN setRegionOfInterestAsCenteredSquare(int size) {
		
		size = MathUtil.clamp(size, 0, 8);
		
		return this.setRegionOfInterest(8, 8, 2 * size, 2 * size);

    }
	
	/**
	 * Returns a measurement to the closest obstruction in the line-of-sight of
	 * this LaserCAN.
	 * 
	 * @return A measurement to the closest obstruction in the line-of-sight of
	 * this LaserCAN. 
	 */
	public Distance getDistance() {
		
		LaserCanInterface.Measurement measurement =
			this.laserCAN.getMeasurement();
		boolean isMeasurementValid = measurement != null &&
			measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
		
		return isMeasurementValid
			? INTRINSIC_DISTANCE_UNIT.of(measurement.distance_mm)
			: null;
	
	}

	/**
	 * Returns a {@link Trigger} whose logic HIGH state is having an obstruction
	 * in front of this LaserCAN that is within the specified distance.
	 * 
	 * @param triggerDistance The distance within which the returned Trigger
	 * should be in a logic HIGH state.
	 * @return A Trigger whose logic HIGH state is having an obstruction in
	 * front of this LaserCAN that is within the specified distance.
	 */
	public Trigger getDistanceTrigger(Distance triggerDistance) {
		
		return new Trigger(() -> {
			
			Distance measuredDistance = this.getDistance();
			
			if (measuredDistance == null) return false;
			else return measuredDistance.lte(triggerDistance);
			
		});
		
	}
	
}
