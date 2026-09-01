package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum CardinalDirections {

    LEFT(90),
    RIGHT(-90),
    UP(0),
    DOWN(180);

    private int degrees;

    private CardinalDirections(int degrees) {

        this.degrees = degrees;

    }

    public Angle getDegrees() {

        return Degrees.of(this.degrees);

    }

}
