package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.state.TurretHeading;
import frc.robot.state.TurretPitch;
import frc.robot.state.TurretWheelSpeeds;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    
    protected static final AngleUnit DEFAULT_HEADING_UNITS = Degrees;
    
    protected static final AngleUnit DEFAULT_PITCH_UNITS = Degrees;
    
    protected static final Angle DEFAULT_HEADING_TOLERANCE = Degrees.of(1);
    
    protected static final Angle DEFAULT_PITCH_TOLERANCE = Degrees.of(1);
    
    protected final TalonFX lowerWheelMotor;
    
    protected final TalonFX upperWheelMotor;
    
    protected final TalonFX headingMotor;
    
    protected TurretWheelSpeeds wheelSpeeds;

    public final Commands commands;
    
    public final Triggers triggers;

    public Turret() {
        
        this.lowerWheelMotor = new TalonFX(CANDevice.TURRET_LOWER_WHEEL_MOTOR_CONTROLLER.id);
        this.upperWheelMotor = new TalonFX(CANDevice.TURRET_UPPER_WHEEL_MOTOR_CONTROLLER.id);
        this.headingMotor = new TalonFX(CANDevice.TURRET_HEADING_MOTOR_CONTROLLER.id);
        this.wheelSpeeds = TurretWheelSpeeds.STOPPED;
        this.commands = new Commands();
        this.triggers = new Triggers();
        
        this.lowerWheelMotor.getConfigurator().apply(Turret.getLowerWheelMotorConfig());
        this.upperWheelMotor.getConfigurator().apply(Turret.getUpperWheelMotorConfig());
        this.headingMotor.getConfigurator().apply(Turret.getHeadingMotorConfiguration());
        
    }
    
    protected static TalonFXConfiguration getLowerWheelMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kS = 0.2;
        // 1rps @ 0.25v
        // 2rps @ 0.36v
        // 3rps @ 0.46v
        config.Slot0.kV = 0.1;
//        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        
        config.MotionMagic.MotionMagicAcceleration = 400;
//        config.MotionMagic.MotionMagicJerk = 4000;

//    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        return config;
    }
    
    protected static TalonFXConfiguration getUpperWheelMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kS = 0.2;
        config.Slot0.kV = 0.1;
//        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        
        config.MotionMagic.MotionMagicAcceleration = 400;
//        config.MotionMagic.MotionMagicJerk = 4000;

//    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        return config;
        
    }
    
    protected static TalonFXConfiguration getHeadingMotorConfiguration() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.Slot0.kS = 0.1;
        config.Slot0.kV = 1;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        
        config.MotionMagic.MotionMagicCruiseVelocity = 25;
        config.MotionMagic.MotionMagicAcceleration = 25;
        config.MotionMagic.MotionMagicJerk = 1000;
        
        return config;
        
    }
    
    public TurretWheelSpeeds getActualWheelSpeeds() {
        
        return TurretWheelSpeeds.fromDynamicAngularMotorShaftVelocities(
            this.lowerWheelMotor.getVelocity().asSupplier(),
            this.upperWheelMotor.getVelocity().asSupplier()
        );
        
    }
    
    public TurretWheelSpeeds getWheelSpeedsSetpoints() {
        
        return this.wheelSpeeds;
        
    }
    
    public void setWheelSpeeds(TurretWheelSpeeds wheelSpeeds) {
        
        this.wheelSpeeds = wheelSpeeds;
        
        this.lowerWheelMotor.setControl(new MotionMagicVelocityVoltage(
            this.wheelSpeeds.getLowerWheelMotorShaftAngularVelocity()
        ));
        
        this.upperWheelMotor.setControl(new MotionMagicVelocityVoltage(
            this.wheelSpeeds.getUpperWheelMotorShaftAngularVelocity()
        ));
        
    }
    
    public void stopWheels() {
        
        this.lowerWheelMotor.stopMotor();
        this.upperWheelMotor.stopMotor();
        
    }
    
    public TurretHeading getHeading() {
        
        return TurretHeading.fromMotorShaftAngle(
            this.headingMotor.getPosition().getValue()
        );
        
    }
    
    public void goToHeading(TurretHeading heading) {
        
        this.headingMotor.setControl(new MotionMagicVoltage(
            heading.getMotorShaftAngle()
        ));
        
    }
    
    public TurretPitch getPitch() {
        
        return TurretPitch.ZERO_POSITION;
        
    }
    
    public void goToPitch(TurretPitch pitch) {
        
        // not yet implemented -- no pitch adjustment available yet
        
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new TurretWheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new TurretWheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new TurretWheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new TurretWheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(TurretWheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(feetPerSecond),
                this.getWheelSpeedsSetpoints().getUpperWheelSurfaceSpeed()
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(TurretWheelSpeeds.fromStaticWheelSurfaceVelocities(
                this.getWheelSpeedsSetpoints().getLowerWheelSurfaceSpeed(),
                FeetPerSecond.of(feetPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Heading (Degrees)",
            () -> this.getHeading().getHeading().in(Turret.DEFAULT_HEADING_UNITS),
            (double angle) -> this.goToHeading(
                TurretHeading.fromHeading(Turret.DEFAULT_HEADING_UNITS.of(angle))
            )
        );
        
        builder.addDoubleProperty(
            "Turret Pitch (Degrees)",
            () -> this.getPitch().getPitch().in(Turret.DEFAULT_PITCH_UNITS),
            (double angle) -> this.goToPitch(
                TurretPitch.fromPitch(Turret.DEFAULT_PITCH_UNITS.of(angle))
            )
        );
        
//        builder.addDoubleArrayProperty(
//            "Turret PID", 
//            () -> new double[]{this.anglePidController.getP(), this.anglePidController.getD()}, 
//            (double[] pd) -> {
//                this.anglePidController.setP(pd[0]);
//                this.anglePidController.setD(pd[1]);
//            }
//        );
        
    }

    public class Commands {
        
        public Command shoot(TurretWheelSpeeds wheelSpeeds) {
            
            return Turret.this.startEnd(
                () -> Turret.this.setWheelSpeeds(wheelSpeeds),
				Turret.this::stopWheels
            );
            
        }
        
        public Command adjustHeading(double speed) {
            
            return Turret.this.startEnd(
                () -> Turret.this.headingMotor.set(speed),
                Turret.this.headingMotor::stopMotor
            );
            
        }
        
        public Command goToHeading(TurretHeading heading, Angle tolerance) {
            
            return Turret.this.runOnce(() -> Turret.this.goToHeading(heading))
                .andThen(this.waitUntilAtHeading(heading, tolerance));
            
        }
        
        public Command goToHeading(TurretHeading heading) {
            
            return this.goToHeading(heading, Turret.DEFAULT_HEADING_TOLERANCE);
            
        }
        
        public Command waitUntilAtHeading(
            TurretHeading heading,
            Angle tolerance
        ) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtHeading(heading, tolerance)
            );
            
        }
        
        public Command waitUntilAtHeading(TurretHeading heading) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtHeading(heading)
            ); 
            
        }
        
//        public Command adjustPitch(double speed) {
//            
//            return Turret.this.startEnd(
//                () -> Turret.this.
//            )
//
//        }
        
        public Command goToPitch(TurretPitch pitch, Angle tolerance) {
            
            return Turret.this.runOnce(() -> Turret.this.goToPitch(pitch))
                .andThen(this.waitUntilAtPitch(pitch, tolerance));
            
        }
        
        public Command goToPitch(TurretPitch pitch) {
            
            return this.goToPitch(pitch, Turret.DEFAULT_PITCH_TOLERANCE);
            
        }
        
        public Command waitUntilAtPitch(TurretPitch pitch, Angle tolerance) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtPitch(pitch, tolerance)
            );
            
        }
        
        public Command waitUntilAtPitch(TurretPitch pitch) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtPitch(pitch)
            );
            
        }
        
    }
    
    public class Triggers {
        
        public Trigger isAtHeading(TurretHeading heading, Angle tolerance) {
            
            return new Trigger(() ->
                Turret.this.getHeading().getHeading().isNear(
                    heading.getHeading(),
                    tolerance
                )
            );
            
        }
        
        public Trigger isAtHeading(TurretHeading heading) {
            
            return this.isAtHeading(heading, Turret.DEFAULT_HEADING_TOLERANCE);
            
        }
        
        public Trigger isAtPitch(TurretPitch pitch, Angle tolerance) {
            
            return new Trigger(() ->
                Turret.this.getPitch().getPitch().isNear(
                    pitch.getPitch(),
                    tolerance
                )
            );
            
        }
        
        public Trigger isAtPitch(TurretPitch pitch) {
            
            return this.isAtPitch(pitch, Turret.DEFAULT_PITCH_TOLERANCE);
            
        }
        
    }
}
