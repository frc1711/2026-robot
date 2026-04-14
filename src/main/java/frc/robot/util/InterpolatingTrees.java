package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatingTrees {
    
    protected final DoubleSupplier distanceSupplier;

    protected final InterpolatingDoubleTreeMap flywheelSpeedTreeMap = new InterpolatingDoubleTreeMap();

    public InterpolatingTrees(DoubleSupplier supplier) {

        this.distanceSupplier = supplier;

        // Add the data points for the tree map

    }

    private void addTreeValue(double key, double value) {
        this.flywheelSpeedTreeMap.put(key, value);
    }

    public double getFlywheelSpeedFromDistance() {
        return this.flywheelSpeedTreeMap.get(this.distanceSupplier.getAsDouble());
    }

}
