package frc.robot.state;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.configuration.RobotDimensions;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

public class IntakePosition {
    
    public static final IntakePosition FULLY_STOWED = new IntakePosition(Inches.of(0));
    
    public static final IntakePosition PARTIALLY_STOWED = new IntakePosition(Inches.of(7));
    
    public static final IntakePosition INTAKING = new IntakePosition(Inches.of(10.75));
    
    public static final IntakePosition FULLY_EXTENDED = new IntakePosition(Inches.of(11));

    protected final Distance offsetFromFullyStowed;

    public IntakePosition(Distance offsetFromFullyStowed) {
        
        this.offsetFromFullyStowed = offsetFromFullyStowed;
        
    }
    
    public static IntakePosition fromMotorShaftAngle(Angle motorShaftAngle) {
        
        return new IntakePosition(
            motorShaftAngle.div(Rotations.of(5)).times(
                RobotDimensions.INTAKE_EXTENSION_GEAR_RACK_PITCH.times(
                    RobotDimensions.INTAKE_EXTENSION_GEAR_TOOTH_COUNT
                )
            )
        );
        
    }
    
    public IntakePosition plus(Distance extension) {
        
        return new IntakePosition(this.offsetFromFullyStowed.plus(extension));
        
    }
    
    public Distance getOffsetFromFullyStowed() {
        
        return this.offsetFromFullyStowed;
        
    }
    
    public Angle getMotorShaftAngle() {
        
        return this.offsetFromFullyStowed.div(
            RobotDimensions.INTAKE_EXTENSION_GEAR_RACK_PITCH.times(
                RobotDimensions.INTAKE_EXTENSION_GEAR_TOOTH_COUNT
            )
        ).times(Rotations.of(5));
        
    }
    
}
