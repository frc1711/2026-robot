package frc.robot.state;

import edu.wpi.first.units.measure.Angle;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.Degrees;

public class TurretPitch {
    
    public static final TurretPitch ZERO_POSITION = new TurretPitch(Degrees.zero());
    
    protected final Angle pitch;
    
    protected TurretPitch(Angle turretAngle) {
        
        this.pitch = turretAngle;
        
    }
    
    public static TurretPitch fromPitch(Angle pitch) {
        
        return new TurretPitch(pitch);
        
    }
    
//    public static TurretPitch fromMotorShaftAngle(Angle motorShaftAngle) {
//        
//        return new TurretPitch(
//
//        );
//        
//    }
    
    public Angle getPitch() {
        
        return this.pitch;
        
    }
    
//    public Angle getMotorShaftAngle() {
//        
//        return this.pitch
//            .times(RobotDimensions.TURRET_ROTATION_DRIVEN_PULLEY_TOOTH_COUNT)
//            .div(RobotDimensions.TURRET_ROTATION_DRIVE_PULLEY_TOOTH_COUNT);
//        
//    }
    
}
