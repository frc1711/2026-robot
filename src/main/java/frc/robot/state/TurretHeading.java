package frc.robot.state;

import edu.wpi.first.units.measure.Angle;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.Degrees;

public class TurretHeading {
    
    public static final TurretHeading ZERO_POSITION = new TurretHeading(Degrees.zero());
    
    protected final Angle heading;
    
    protected TurretHeading(Angle turretAngle) {
        
        this.heading = turretAngle;
        
    }
    
    public static TurretHeading fromHeading(Angle heading) {
        
        return new TurretHeading(heading);
        
    }
    
    public static TurretHeading fromMotorShaftAngle(Angle motorShaftAngle) {
        
        return new TurretHeading(
            motorShaftAngle
                .times(RobotDimensions.TURRET_ROTATION_DRIVING_PULLEY_TOOTH_COUNT)
                .div(RobotDimensions.TURRET_ROTATION_DRIVEN_PULLEY_TOOTH_COUNT)
        );
        
    }
    
    public Angle getHeading() {
        
        return this.heading;
        
    }
    
    public Angle getMotorShaftAngle() {
        
        return this.heading
            .times(RobotDimensions.TURRET_ROTATION_DRIVEN_PULLEY_TOOTH_COUNT)
            .div(RobotDimensions.TURRET_ROTATION_DRIVING_PULLEY_TOOTH_COUNT);
        
    }
    
}
