package frc.robot.math;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.*;

public class LinearMotionProfiler {
    
    private static final DistanceUnit INTRINSIC_DISTANCE_UNIT = Meters;
    
    private static final TimeUnit INTRINSIC_TIME_UNIT = Seconds;
    
    private static final LinearVelocityUnit INTRINSIC_VELOCITY_UNIT =
        INTRINSIC_DISTANCE_UNIT.per(INTRINSIC_TIME_UNIT);
    
    private static final LinearAccelerationUnit INTRINSIC_ACCELERATION_UNIT =
        INTRINSIC_VELOCITY_UNIT.per(INTRINSIC_TIME_UNIT);
    
    protected final double maxVelocity;
    
    protected final double acceleration;
    
    protected final double deceleration;
    
    protected double lastTime;
    
    public LinearMotionProfiler(
        LinearVelocity maxVelocity,
        LinearAcceleration acceleration
    ) {
        
        this(maxVelocity, acceleration, acceleration);
        
    }
    
    public LinearMotionProfiler(
        LinearVelocity maxVelocity,
        LinearAcceleration acceleration,
        LinearAcceleration deceleration
    ) {
        
        this.maxVelocity = Math.abs(maxVelocity.in(INTRINSIC_VELOCITY_UNIT));
        this.acceleration = Math.abs(acceleration.in(INTRINSIC_ACCELERATION_UNIT));
        this.deceleration = Math.abs(deceleration.in(INTRINSIC_ACCELERATION_UNIT));
        this.lastTime = Timer.getFPGATimestamp();
        
    }
    
    public LinearVelocity calculate(
        Distance remainingDistance,
        LinearVelocity currentVelocity
    ) {
        
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - this.lastTime;
        double _remainingDistance = remainingDistance.in(INTRINSIC_DISTANCE_UNIT);
        double _currentVelocity = currentVelocity.in(INTRINSIC_VELOCITY_UNIT);
        
        double maxVelocityAfterAcceleration = Math.min(
            _currentVelocity + (this.acceleration * deltaTime),
            this.maxVelocity
        );
        double maxVelocityWithoutOvershoot =
            Math.sqrt(2 * this.deceleration * _remainingDistance);
        
        this.lastTime = currentTime;
        
        return MetersPerSecond.of(Math.min(
            maxVelocityAfterAcceleration,
            maxVelocityWithoutOvershoot
        ));
        
    }
    
}
