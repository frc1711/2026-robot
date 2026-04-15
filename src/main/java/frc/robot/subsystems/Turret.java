package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.RobotDimensions;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    
    protected static final AngleUnit DEFAULT_HEADING_UNITS = Degrees;
    
    protected static final AngleUnit DEFAULT_PITCH_UNITS = Degrees;
    
    protected static final Angle DEFAULT_HEADING_TOLERANCE = Degrees.of(1);
    
    protected static final Angle DEFAULT_PITCH_TOLERANCE = Degrees.of(1);
    
    protected final TalonFX lowerWheelMotor;
    
    protected final TalonFX upperWheelMotor;
    
    protected final TalonFX headingMotor;
    
    protected WheelSpeeds wheelSpeeds;

    public final Commands commands;
    
    public final Triggers triggers;

    public Turret() {
        
        this.lowerWheelMotor = new TalonFX(CANDevice.TURRET_LOWER_WHEEL_MOTOR_CONTROLLER.id);
        this.upperWheelMotor = new TalonFX(CANDevice.TURRET_UPPER_WHEEL_MOTOR_CONTROLLER.id);
        this.headingMotor = new TalonFX(CANDevice.TURRET_HEADING_MOTOR_CONTROLLER.id);
        this.wheelSpeeds = WheelSpeeds.STOPPED;
        this.commands = new Commands();
        this.triggers = new Triggers();
        
        this.lowerWheelMotor.getConfigurator().apply(Turret.getLowerWheelMotorConfig());
        this.upperWheelMotor.getConfigurator().apply(Turret.getUpperWheelMotorConfig());
        this.headingMotor.getConfigurator().apply(Turret.getHeadingMotorConfiguration());
        
        Shuffleboard.getTab("Subsystems").add("Turret", this);
        
    }
    
    protected static TalonFXConfiguration getLowerWheelMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
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
        
        TalonFXConfiguration config = Turret.getLowerWheelMotorConfig();
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
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
    
    public WheelSpeeds getActualWheelSpeeds() {
        
        return WheelSpeeds.fromDynamicAngularMotorShaftVelocities(
            this.lowerWheelMotor.getVelocity().asSupplier(),
            this.upperWheelMotor.getVelocity().asSupplier()
        );
        
    }
    
    public WheelSpeeds getWheelSpeedsSetpoints() {
        
        return this.wheelSpeeds;
        
    }
    
    public void setWheelSpeeds(WheelSpeeds wheelSpeeds) {
        
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
    
    public boolean isAtWheelSpeeds(WheelSpeeds wheelSpeeds, double varianceThreshold) {
        
        WheelSpeeds actualWheelSpeeds = this.getActualWheelSpeeds();
        boolean isLowerWheelAtSpeed = actualWheelSpeeds
            .getLowerWheelMotorShaftAngularVelocity()
            .isNear(wheelSpeeds.getLowerWheelMotorShaftAngularVelocity(), varianceThreshold);
        
        boolean isUpperWheelAtSpeed = actualWheelSpeeds
            .getUpperWheelMotorShaftAngularVelocity()
            .isNear(wheelSpeeds.getUpperWheelMotorShaftAngularVelocity(), varianceThreshold);
        
        return isLowerWheelAtSpeed && isUpperWheelAtSpeed;
        
    }
    
    public Heading getHeading() {
        
        return Heading.fromMotorShaftAngle(
            this.headingMotor.getPosition().getValue()
        );
        
    }
    
    public void goToHeading(Heading heading) {
        
        this.headingMotor.setControl(new MotionMagicVoltage(
            heading.getMotorShaftAngle()
        ));
        
    }
    
    public Pitch getPitch() {
        
        return Pitch.ZERO_POSITION;
        
    }
    
    public void goToPitch(Pitch pitch) {
        
        // not yet implemented -- no pitch adjustment available yet
        
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new WheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new WheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new WheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new WheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Lower Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(WheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(feetPerSecond),
                this.getWheelSpeedsSetpoints().getUpperWheelSurfaceSpeed()
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Upper Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(WheelSpeeds.fromStaticWheelSurfaceVelocities(
                this.getWheelSpeedsSetpoints().getLowerWheelSurfaceSpeed(),
                FeetPerSecond.of(feetPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Turret Heading (Degrees)",
            () -> this.getHeading().getHeading().in(Turret.DEFAULT_HEADING_UNITS),
            (double angle) -> this.goToHeading(
                Heading.fromHeading(Turret.DEFAULT_HEADING_UNITS.of(angle))
            )
        );
        
        builder.addDoubleProperty(
            "Turret Pitch (Degrees)",
            () -> this.getPitch().getPitch().in(Turret.DEFAULT_PITCH_UNITS),
            (double angle) -> this.goToPitch(
                Pitch.fromPitch(Turret.DEFAULT_PITCH_UNITS.of(angle))
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
        
        public Command shoot(WheelSpeeds wheelSpeeds) {
            
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
        
        public Command goToHeading(Heading heading, Angle tolerance) {
            
            return Turret.this.runOnce(() -> Turret.this.goToHeading(heading))
                .andThen(this.waitUntilAtHeading(heading, tolerance));
            
        }
        
        public Command goToHeading(Heading heading) {
            
            return this.goToHeading(heading, Turret.DEFAULT_HEADING_TOLERANCE);
            
        }
        
        public Command waitUntilAtHeading(
            Heading heading,
            Angle tolerance
        ) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtHeading(heading, tolerance)
            );
            
        }
        
        public Command waitUntilAtHeading(Heading heading) {
            
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
        
        public Command goToPitch(Pitch pitch, Angle tolerance) {
            
            return Turret.this.runOnce(() -> Turret.this.goToPitch(pitch))
                .andThen(this.waitUntilAtPitch(pitch, tolerance));
            
        }
        
        public Command goToPitch(Pitch pitch) {
            
            return this.goToPitch(pitch, Turret.DEFAULT_PITCH_TOLERANCE);
            
        }
        
        public Command waitUntilAtPitch(Pitch pitch, Angle tolerance) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtPitch(pitch, tolerance)
            );
            
        }
        
        public Command waitUntilAtPitch(Pitch pitch) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Turret.this.triggers.isAtPitch(pitch)
            );
            
        }
        
    }
    
    public class Triggers {
        
        public Trigger isAtWheelSpeeds(WheelSpeeds wheelSpeeds, double varianceThreshold) {
            
            return new Trigger(
                () -> Turret.this.isAtWheelSpeeds(wheelSpeeds, varianceThreshold)
            );
            
        }
        
        public Trigger isAtHeading(Heading heading, Angle tolerance) {
            
            return new Trigger(() ->
                Turret.this.getHeading().getHeading().isNear(
                    heading.getHeading(),
                    tolerance
                )
            );
            
        }
        
        public Trigger isAtHeading(Heading heading) {
            
            return this.isAtHeading(heading, Turret.DEFAULT_HEADING_TOLERANCE);
            
        }
        
        public Trigger isAtPitch(Pitch pitch, Angle tolerance) {
            
            return new Trigger(() ->
                Turret.this.getPitch().getPitch().isNear(
                    pitch.getPitch(),
                    tolerance
                )
            );
            
        }
        
        public Trigger isAtPitch(Pitch pitch) {
            
            return this.isAtPitch(pitch, Turret.DEFAULT_PITCH_TOLERANCE);
            
        }
        
    }
    
    public static class WheelSpeeds {
        
        public static final WheelSpeeds STOPPED =
            WheelSpeeds.fromStaticAngularWheelVelocities(
                RotationsPerSecond.zero(),
                RotationsPerSecond.zero()
            );
        
        public static final WheelSpeeds CLOSE_SHOT =
            WheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(14.5),
                FeetPerSecond.of(14.5).plus(FeetPerSecond.of(20))
            );
        
        public static final WheelSpeeds MID_SHOT =
            WheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(17.5),
                FeetPerSecond.of(17.5).plus(FeetPerSecond.of(20))
            );
        
        public static final WheelSpeeds FAR_SHOT =
            WheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(30),
                FeetPerSecond.of(30).plus(FeetPerSecond.of(15))
            );
        
        Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier;
        
        Supplier<AngularVelocity> upperWheelAngularVelocitySupplier;
        
        public WheelSpeeds(
            Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
            Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
        ) {
            
            this.lowerWheelAngularVelocitySupplier = lowerWheelAngularVelocitySupplier;
            this.upperWheelAngularVelocitySupplier = upperWheelAngularVelocitySupplier;
            
        }
        
        public static WheelSpeeds fromDynamicAngularMotorShaftVelocities(
            Supplier<AngularVelocity> lowerMotorShaftAngularVelocitySupplier,
            Supplier<AngularVelocity> upperMotorShaftAngularVelocitySupplier
        ) {
            
            return new WheelSpeeds(
                lowerMotorShaftAngularVelocitySupplier,
                upperMotorShaftAngularVelocitySupplier
            );
            
        }
        
        public static WheelSpeeds fromDynamicAngularWheelVelocities(
            Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
            Supplier<AngularVelocity> upperWheelAngularVelocitySupplier
        ) {
            
            return new WheelSpeeds(
                lowerWheelAngularVelocitySupplier,
                upperWheelAngularVelocitySupplier
            );
            
        }
        
        public static WheelSpeeds fromStaticAngularWheelVelocities(
            AngularVelocity lowerWheelAngularVelocity,
            AngularVelocity upperWheelAngularVelocity
        ) {
            
            return new WheelSpeeds(
                () -> lowerWheelAngularVelocity,
                () -> upperWheelAngularVelocity
            );
            
        }
        
        public static WheelSpeeds fromRelativeDynamicAngularWheelVelocities(
            Supplier<AngularVelocity> lowerWheelAngularVelocitySupplier,
            DoubleSupplier upperWheelRelativeSpeedMultiplierSupplier
        ) {
            
            return new WheelSpeeds(
                lowerWheelAngularVelocitySupplier,
                () -> lowerWheelAngularVelocitySupplier.get()
                    .times(upperWheelRelativeSpeedMultiplierSupplier.getAsDouble())
            );
            
        }
        
        public static WheelSpeeds fromRelativeStaticAngularWheelVelocities(
            AngularVelocity lowerWheelAngularVelocity,
            double upperWheelRelativeSpeedMultiplier
        ) {
            
            return new WheelSpeeds(
                () -> lowerWheelAngularVelocity,
                () -> lowerWheelAngularVelocity
                    .times(upperWheelRelativeSpeedMultiplier)
            );
            
        }
        
        public static WheelSpeeds fromDynamicWheelSurfaceVelocities(
            Supplier<LinearVelocity> lowerWheelSurfaceVelocitySupplier,
            Supplier<LinearVelocity> upperWheelSurfaceVelocitySupplier
        ) {
            
            return new WheelSpeeds(
                () -> RotationsPerSecond.of(
                    lowerWheelSurfaceVelocitySupplier.get().in(InchesPerSecond) /
                        RobotDimensions.TURRET_LOWER_WHEEL_CIRCUMFERENCE.in(Inches)
                ),
                () -> RotationsPerSecond.of(
                    upperWheelSurfaceVelocitySupplier.get().in(InchesPerSecond) /
                        RobotDimensions.TURRET_UPPER_WHEEL_CIRCUMFERENCE.in(Inches)
                )
            );
            
        }
        
        public static WheelSpeeds fromStaticWheelSurfaceVelocities(
            LinearVelocity lowerWheelSurfaceVelocity,
            LinearVelocity upperWheelSurfaceVelocity
        ) {
            
            return WheelSpeeds.fromDynamicWheelSurfaceVelocities(
                () -> lowerWheelSurfaceVelocity,
                () -> upperWheelSurfaceVelocity
            );
            
        }
        
        public WheelSpeeds withScaling(double scalingFactor) {
            
            return new WheelSpeeds(
                () -> this.getLowerWheelAngularVelocity().times(scalingFactor),
                () -> this.getUpperWheelAngularVelocity().times(scalingFactor)
            );
            
        }
        
        public AngularVelocity getLowerWheelMotorShaftAngularVelocity() {
            
            return this.lowerWheelAngularVelocitySupplier.get();
            
        }
        
        public AngularVelocity getLowerWheelAngularVelocity() {
            
            return this.lowerWheelAngularVelocitySupplier.get();
            
        }
        
        public LinearVelocity getLowerWheelSurfaceSpeed() {
            
            return RobotDimensions.TURRET_LOWER_WHEEL_CIRCUMFERENCE
                .times(Hertz.of(this.getLowerWheelAngularVelocity().in(RotationsPerSecond)));
            
        }
        
        public AngularVelocity getUpperWheelMotorShaftAngularVelocity() {
            
            return this.upperWheelAngularVelocitySupplier.get();
            
        }
        
        public AngularVelocity getUpperWheelAngularVelocity() {
            
            return this.upperWheelAngularVelocitySupplier.get();
            
        }
        
        public LinearVelocity getUpperWheelSurfaceSpeed() {
            
            return RobotDimensions.TURRET_UPPER_WHEEL_CIRCUMFERENCE
                .times(Hertz.of(this.getUpperWheelAngularVelocity().in(RotationsPerSecond)));
            
        }
        
    }
    
    public static class Heading {
        
        public static final Heading ZERO_POSITION = new Heading(Degrees.zero());
        
        protected final Angle heading;
        
        protected Heading(Angle turretAngle) {
            
            this.heading = turretAngle;
            
        }
        
        public static Heading fromHeading(Angle heading) {
            
            return new Heading(heading);
            
        }
        
        public static Heading fromMotorShaftAngle(Angle motorShaftAngle) {
            
            return new Heading(
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
    
    public static class Pitch {
        
        public static final Pitch ZERO_POSITION = new Pitch(Degrees.zero());
        
        protected final Angle pitch;
        
        protected Pitch(Angle turretAngle) {
            
            this.pitch = turretAngle;
            
        }
        
        public static Pitch fromPitch(Angle pitch) {
            
            return new Pitch(pitch);
            
        }

//        public static Pitch fromMotorShaftAngle(Angle motorShaftAngle) {
//    
//            return new Pitch(
//    
//            );
//    
//        }
        
        public Angle getPitch() {
            
            return this.pitch;
            
        }

//        public Angle getMotorShaftAngle() {
//    
//            return this.pitch
//                .times(RobotDimensions.TURRET_ROTATION_DRIVEN_PULLEY_TOOTH_COUNT)
//                .div(RobotDimensions.TURRET_ROTATION_DRIVE_PULLEY_TOOTH_COUNT);
//    
//        }
        
    }
    
}
