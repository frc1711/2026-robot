package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Motors for the swerve module
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // PID controller for turning and limiter for drive velocity
    private final PIDController turningPidController;
    private final SlewRateLimiter driveVelocityLimiter = new SlewRateLimiter(15, -20, 0);

    // Encoder values
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private SwerveModuleState state;

    private final int swerveID;

    private final double driveRotationToMeter;
    private final double driveMaxSpeed;

    /**
     * Constructor method for the swerve subsystem
     * 
     * @param driveMotorId CAN ID of drive motor
     * @param turningMotorId CAN ID of turning motor
     * @param driveMotorReversed If the drive motor is reversed
     * @param absoluteEncoderId CAN ID of the {@link com.ctre.phoenix6.hardware.CANcoder CANCoder}
     * @param absoluteEncoderReversed If the {@link com.ctre.phoenix6.hardware.CANcoder CANCoder} is reversed
     * @param swerveID Number used for identifying swerve module in preferences and the dashboard
     * @param gearRatio The gear ratio of the drive motor (1, 2, or 3)
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed, int swerveID, int gearRatio) {
        this.swerveID = swerveID;

        // Create and configure the absolute encoder
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Units.radiansToRotations(Preferences.getDouble("Module" + this.swerveID + "Zero", 0));
        this.absoluteEncoder.getConfigurator().apply(config);

        // Create and configure the drive and turn motors
        this.driveMotor = new TalonFX(driveMotorId);
        this.turningMotor = new TalonFX(turningMotorId);

        this.driveMotor.getConfigurator().apply(getDriveConfig(driveMotorReversed));
        this.turningMotor.getConfigurator().apply(getTurningConfig());

        // Create pid controller for turning motor
        this.turningPidController = new PIDController(
            Preferences.getDouble("Module" + this.swerveID + "TurningP", SwerveConstants.SWERVETURNINGP),
            0,
            Preferences.getDouble("Module" + this.swerveID + "TurningD", SwerveConstants.SWERVETURNINGD)
        );
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        this.turningPidController.setTolerance(0.1);

        // Based on gear ratio, make conversions the correct conversion
        this.driveRotationToMeter = (gearRatio == 1) ? SwerveConstants.ROTATIONSTOMETERSR1 : (gearRatio == 2) ? SwerveConstants.ROTATIONSTOMETERSR2 : SwerveConstants.ROTATIONSTOMETERSR3;
        this.driveMaxSpeed = (gearRatio == 1) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR1 : (gearRatio == 2) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR2 : SwerveConstants.PHYSICALMAXSPEEDMPERSECR3;

        this.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(getAbsoluteEncoderRad())));
    }

    /**
     * @param reversed If the drive motor is reversed
     * @return The configuration to be applied to the drive motor
     */
    private TalonFXConfiguration getDriveConfig(boolean reversed) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = reversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        return config;
    }

    /**
     * @return The configuration to be applied to the turning motor
     */
    private TalonFXConfiguration getTurningConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        return config;
    }

    /**
     * @return The amount the drive wheel has spun in meters
     */
    public double getDrivePosition() {
        return this.driveMotor.getPosition().getValueAsDouble() * this.driveRotationToMeter;
    }

    /**
     * @return What the current velocity of the drive motor is in m/s
     */
    public double getDriveVelocity() {
        return this.driveMotor.getVelocity().getValueAsDouble() * this.driveRotationToMeter;
    }

    /**
     * @return What the absolute encoder is reading as the angle in degrees
     */
    public double getAbsoluteEncoderDeg() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absoluteEncoderReversed) {
            angle = angle.unaryMinus();
        }

        return angle.getDegrees();
    }

    /**
     * @return What the absolute encoder is reading as the angle in radians
     */
    public double getAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absoluteEncoderReversed) {
            angle = angle.unaryMinus();
        }

        return angle.getRadians();
    }

    /**
     * Resets the drive motor position and makes the current absolute encoder angle 0
     */
    public void resetEncoders() {
        this.driveMotor.setPosition(0);

        double offset = Rotation2d.fromRadians(Preferences.getDouble("Module" + this.swerveID + "Zero", 0)).minus(Rotation2d.fromRadians(getAbsoluteEncoderRad())).getRotations();
        Preferences.setDouble("Module" + this.swerveID + "Zero", Rotation2d.fromRotations(offset).getRadians());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        this.absoluteEncoder.getConfigurator().apply(config);
    }

    /**
     * @return The {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} using the motor velocity and absolute encoder position
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    public void runVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public void periodic() {
        // Add some speed limiting to the drive motor getting up to speed
        double limitedSpeed = driveVelocityLimiter.calculate(this.state.speedMetersPerSecond);
        this.driveMotor.set(limitedSpeed / this.driveMaxSpeed);

        // Set the turning motor voltage to the pid controller output
        this.turningMotor.setVoltage(this.turningPidController.calculate(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber(this.swerveID + " Velocity", this.getDriveVelocity());
    }

    /**
     * Sets the current {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} to the new desired state
     * 
     * @param desiredState The state to get the module to be at
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.05) {
            desiredState.speedMetersPerSecond = 0;
        }
        desiredState.optimize(getState().angle);
        this.state = desiredState;
        SmartDashboard.putNumber(this.swerveID+ " Velocity Setpoint", this.state.speedMetersPerSecond);
        turningPidController.setSetpoint(this.state.angle.getRadians());
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
        return position;
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    public int getSwerveID() {
        return this.swerveID;
    }

    public PIDController getTurningController() {
        return this.turningPidController;
    }

    public void setTurningControllerP(double p) {
        Preferences.setDouble("Module" + swerveID + "TurningP", p);
        this.turningPidController.setP(p);
    }

    public void setTurningControllerD(double d) {
        Preferences.setDouble("Module" + swerveID + "TurningD", d);
        this.turningPidController.setD(d);
    }
}