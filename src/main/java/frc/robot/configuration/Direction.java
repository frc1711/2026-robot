package frc.robot.configuration;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class Direction implements Angle {
    
    public static final Direction FORWARDS = new Direction(Degrees.of(0));
    
    public static final Direction BACKWARDS = new Direction(Degrees.of(180));
    
    public static final Direction LEFT = new Direction(Degrees.of(90));
    
    public static final Direction RIGHT = new Direction(Degrees.of(270));
    
    protected double degrees;
    
    public Direction(Angle angle) {
        
        this.degrees = angle.in(Degrees);
        
    }
    
    @Override
    public Angle copy() {
        return Degrees.of(this.degrees);
    }

    @Override
    public AngleUnit unit() {
        return Degrees;
    }

    @Override
    public double magnitude() {
        return this.degrees;
    }

    @Override
    public double baseUnitMagnitude() {
        return this.copy().baseUnitMagnitude();
    }
}
