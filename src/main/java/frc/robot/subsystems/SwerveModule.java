package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.configuration.RobotDimensions;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.math.DoubleUtilities;

import static edu.wpi.first.units.Units.*;

public class SwerveModule {
    
    public static final Time TIME_TO_MAX_DRIVE_ANGULAR_VELOCITY =
        Seconds.of(0.25);
    
    public static final AngularAcceleration MAX_DRIVE_ANGULAR_ACCELERATION =
        RobotDimensions.KRAKEN_X60_MAX_FREE_SPEED
            .div(SwerveModule.TIME_TO_MAX_DRIVE_ANGULAR_VELOCITY);
    
//    public static final Time TIME_TO_MAX_DRIVE_ANGULAR_ACCELERATION =
//        Seconds.of(0.1);
//    
//    public static final Velocity<AngularAccelerationUnit>
//        MAX_DRIVE_ANGULAR_JERK = SwerveModule.MAX_DRIVE_ANGULAR_ACCELERATION
//            .div(SwerveModule.TIME_TO_MAX_DRIVE_ANGULAR_ACCELERATION);
    
    public static final AngularVelocity MAX_STEER_ANGULAR_VELOCITY =
        RotationsPerSecond.of(30);
    
    public static final AngularAcceleration MAX_STEER_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(100);
    
//    public static final Velocity<AngularAccelerationUnit>
//        MAX_STEER_ANGULAR_JERK = RotationsPerSecondPerSecond.per(Second).of(24);
    
    protected final SwerveModuleConfiguration configuration;
    
    // Motors for the swerve module
    protected final TalonFX driveMotor;
    
    protected final TalonFX steerMotor;
    
    protected final CANcoder steeringEncoder;
    
    protected LinearVelocity driveVelocitySetpoint;
    
    protected Angle steerAngleSetpoint;

    protected SwerveModuleState state;

    /**
     * Constructor method for the swerve subsystem
     */
    public SwerveModule(SwerveModuleConfiguration configuration) {
        
        this.configuration = configuration;
        this.driveMotor = new TalonFX(this.configuration.driveMotorControllerCANDevice.id);
        this.steerMotor = new TalonFX(this.configuration.steerMotorControllerCANDevice.id);
        this.steeringEncoder = new CANcoder(this.configuration.steerEncoderCANDevice.id);
        this.driveVelocitySetpoint = InchesPerSecond.of(0);
        this.steerAngleSetpoint = Degrees.of(0);
        
        // Create and configure the drive and steer motors
        this.driveMotor.getConfigurator()
            .apply(this.getDriveMotorConfig());
        this.steerMotor.getConfigurator()
            .apply(this.getSteeringMotorConfig());
        this.steeringEncoder.getConfigurator()
            .apply(this.getSteeringEncoderConfig());
        
        this.configuration.steerEncoderOffset
            .useValue((double steerEncoderOffsetInRotations) -> {
                
                CANcoderConfiguration encoderConfig =
                    new CANcoderConfiguration();
                this.steeringEncoder.getConfigurator().refresh(encoderConfig);
                
                encoderConfig.MagnetSensor.MagnetOffset =
                    steerEncoderOffsetInRotations;
                
                this.steeringEncoder.getConfigurator().apply(encoderConfig);
                
            });
        
    }

    /**
     * @return The configuration to be applied to the drive motor
     */
    protected TalonFXConfiguration getDriveMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
//        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
//        config.Slot0.kA = 0.1;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        
        // Cruise velocity is intentionally unconfigured -- the control mode we
        // are using (velocity) does not need a separate max velocity to be set.
        // config.MotionMagic.MotionMagicCruiseVelocity =
        //    RobotDimensions.KRAKEN_X60_MAX_FREE_SPEED
        //        .in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
            SwerveModule.MAX_DRIVE_ANGULAR_ACCELERATION
                .in(RotationsPerSecondPerSecond);
        // Jerk is left intentionally unconfigured -- the Motion Magic profiler
        // will instead generate an S-curve to match the provided maximum
        // velocity and acceleration.
        // config.MotionMagic.MotionMagicJerk =
        //    SwerveModule.MAX_DRIVE_ANGULAR_JERK
        //        .in(RotationsPerSecondPerSecond.per(Second));
        
        return config;
        
    }

    protected TalonFXConfiguration getSteeringMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        config.Feedback.FeedbackRemoteSensorID =
            this.configuration.steerEncoderCANDevice.id;
        // config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio =
            RobotDimensions.SWERVE_STEERING_GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1;
        // config.Feedback.VelocityFilterTimeConstant = 0;
        
        config.ClosedLoopGeneral.ContinuousWrap = true;
        
        config.Slot0.kS = 0.37;
//        config.Slot0.kV = 0.12;
//        config.Slot0.kA = 0.01;
        
        // kP is in volts per unit of proportional error, which is, in this
        // case, rotations of the motor shaft.
        config.Slot0.kP = 24;
        // kD is in volts per unit of velocity error, which is, in this case,
        // rotations per unit of time of the motor shaft.
        config.Slot0.kD = 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        config.MotionMagic.MotionMagicCruiseVelocity =
            SwerveModule.MAX_STEER_ANGULAR_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
            SwerveModule.MAX_STEER_ANGULAR_ACCELERATION
                .in(RotationsPerSecondPerSecond);
        // Jerk is left intentionally unconfigured -- the Motion Magic profiler
        // will instead generate an S-curve to match the provided maximum
        // velocity and acceleration.
        // config.MotionMagic.MotionMagicJerk =
        //    SwerveModule.MAX_STEER_ANGULAR_JERK
        //        .in(RotationsPerSecondPerSecond.per(Second));
        
        return config;
        
    }
    
    protected CANcoderConfiguration getSteeringEncoderConfig() {
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = this.configuration.steerEncoderOffset.get();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        
        return config;
        
    }
    
    /**
     * Returns the unique ID of this swerve module.
     * 
     * @return The unique ID of this swerve module.
     */
    public int getID() {
        
        return this.configuration.moduleID;
        
    }
    
    /**
     * Returns the position of this swerve module, which consists of the
     * distance the drive wheel has spun and the current heading of the steering
     * motor.
     * 
     * @return The position of this swerve module.
     */
    public SwerveModulePosition getPosition() {
        
        return new SwerveModulePosition(
            this.getDrivePosition(),
            new Rotation2d(this.getSteeringHeading())
        );
        
    }
    
    /**
     * Returns the state of this swerve module, which consists of the surface
     * speed of the drive wheel and the current heading of the steering motor.
     * 
     * @return The state of this swerve module.
     */
    public SwerveModuleState getState() {
        
        return new SwerveModuleState(
            this.getVelocity(),
            new Rotation2d(this.getSteeringHeading())
        );
        
    }
    
    /**
     * @return The amount the drive wheel has spun in meters
     */
    public Distance getDrivePosition() {
        
        return RobotDimensions.SWERVE_WHEEL_CIRCUMFERENCE.times(
            this.configuration.gearRatio.convertMotorShaftThetaToWheelTheta(
                this.driveMotor.getPosition().getValue()
            ).in(Rotations)
        );
        
    }
    
    /**
     * Returns the current heading of this swerve module, oriented in the
     * standard FRC 'northwest-up' ('NWU') coordinate system.
     *
     * @return The current heading of this swerve module.
     */
    public Angle getSteeringHeading() {
        
        return Degrees.of(DoubleUtilities.normalizeToRange(
            this.steeringEncoder.getAbsolutePosition().getValue().in(Degrees),
            -180,
            180
        ));
        
    }
    
    /**
     * Calibrates the steering heading for this swerve module such that it will
     * return the given angle in its current physical position after this method
     * is called.
     *
     * @param currentHeading The heading to calibrate this swerve module at.
     */
    public void calibrateSteeringHeading(Angle currentHeading) {
        
        Angle existingOffset =
            Rotations.of(this.configuration.steerEncoderOffset.get());
        
        this.configuration.steerEncoderOffset.set(
            existingOffset
                .minus(this.getSteeringHeading())
                .minus(currentHeading)
                .in(Rotations)
        );
        
    }
    
    /**
     * Calibrates the steering heading for this swerve module such that it will
     * return 0 degrees in its current physical position after this method is
     * called.
     *
     * @see #calibrateSteeringHeading(Angle)
     */
    public void calibrateSteeringHeading() {
        
        this.calibrateSteeringHeading(Degrees.of(0));
        
    }
    
    /**
     * Returns the setpoint of the steering PID controller, which is the angle
     * that the steering motor is trying to reach.
     * 
     * @return The setpoint of the steering PID controller.
     */
    public Angle getSteeringHeadingSetpoint() {
        
        return this.steerAngleSetpoint;
        
    }
    
    /**
     * Returns a measurement of the error between the current steering heading
     * and the setpoint of the steering PID controller.
     *
     * @return A measurement of the error between the current steering heading
     * and the setpoint of the steering PID controller.
     */
    public Angle getSteeringHeadingError() {
        
        Angle steerHeadingError = this.getSteeringHeading()
            .minus(this.getSteeringHeadingSetpoint());
        
        return Degrees.of(DoubleUtilities.normalizeToRange(
            steerHeadingError.in(Degrees),
            -180,
            180
        ));
        
    }
    
    /**
     * Returns the current surface speed of the drive wheel.
     * 
     * @return The current surface speed of the drive wheel.
     */
    public LinearVelocity getVelocity() {
        
        AngularVelocity motorShaftAngularVelocity =
            this.driveMotor.getVelocity().getValue();
        
        return this.configuration.gearRatio
            .withMotorShaftAngularVelocity(motorShaftAngularVelocity)
            .getWheelSurfaceSpeed();
        
    }
    
    /**
     * Returns the setpoint of the drive motor's velocity controller, which is
     * the surface speed that the drive wheel is trying to reach.
     * 
     * @return The setpoint of the drive motor's velocity controller.
     */
    public LinearVelocity getVelocitySetpoint() {
        
        return this.driveVelocitySetpoint;
        
    }
    
    public void updateModuleState(SwerveModuleState newState) {
        
        Rotation2d currentSteeringHeading =
            new Rotation2d(this.getSteeringHeading());
        
        newState.optimize(currentSteeringHeading);
        
        // Perform cosine speed compensation.
        newState.speedMetersPerSecond *= newState.angle
            .minus(currentSteeringHeading)
            .getCos();
        
        this.steerAngleSetpoint = newState.angle.getMeasure();
        this.driveVelocitySetpoint =
            MetersPerSecond.of(newState.speedMetersPerSecond);
        
        this.steerMotor.setControl(new MotionMagicVoltage(
            this.steerAngleSetpoint
        ));
        
        this.driveMotor.setControl(new MotionMagicVelocityVoltage(
            this.configuration.gearRatio
                .withWheelSurfaceSpeed(this.driveVelocitySetpoint)
                .getMotorShaftAngularVelocity()
        ));
        
    }
    
    public void addSendableFields(SendableBuilder builder, String moduleName) {
        
        builder.addDoubleProperty(
            moduleName + " Angle",
            () -> this.getSteeringHeading().in(Degrees),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Angle Setpoint",
            () -> this.getSteeringHeadingSetpoint().in(Degrees),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Steering Speed (RPS)",
            () -> this.steeringEncoder.getVelocity().getValue().in(RotationsPerSecond),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Velocity",
            () -> this.getVelocity().in(MetersPerSecond),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Velocity (inches per second)",
            () -> this.getVelocity().in(InchesPerSecond),
            null
        );
        
        LinearFilter velocityFilter = LinearFilter.movingAverage(5);
        
        builder.addDoubleProperty(
            moduleName + " Velocity Avg. (inches per second)",
            () -> velocityFilter.calculate(this.getVelocity().in(InchesPerSecond)),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Velocity Setpoint (inches per second)",
            () -> this.getVelocitySetpoint().in(InchesPerSecond),
            null
        );
        
        builder.addDoubleProperty(
            moduleName + " Velocity Error (inches per second)",
            () -> this.getVelocity().minus(this.getVelocitySetpoint()).in(InchesPerSecond),
            null
        );
        
    }
    
}
