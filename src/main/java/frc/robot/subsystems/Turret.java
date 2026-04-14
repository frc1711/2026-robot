package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.CANDevice;
import frc.robot.state.TurretHeading;
import frc.robot.state.TurretPitch;
import frc.robot.state.FlyWheelSpeeds;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    
    protected static final AngleUnit DEFAULT_HEADING_UNITS = Degrees;
    
    protected static final AngleUnit DEFAULT_PITCH_UNITS = Degrees;
    
    protected static final Angle DEFAULT_HEADING_TOLERANCE = Degrees.of(1);
    
    protected static final Angle DEFAULT_PITCH_TOLERANCE = Degrees.of(1);
    
    protected final TalonFX headingMotor;

    // protected final TalonFX hoodMotor;

    public final Commands commands;
    
    public final Triggers triggers;

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    public SysIdRoutine sysIdRoutine;

    public Turret() {
        
        this.headingMotor = new TalonFX(CANDevice.TURRET_HEADING_MOTOR_CONTROLLER.id);
        this.commands = new Commands();
        this.triggers = new Triggers();

        this.sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    Seconds.of(5),        // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> this.headingMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                    null,
                    this
                )
            );
        
        this.headingMotor.getConfigurator().apply(this.getHeadingMotorConfiguration());
        
        SmartDashboard.putData("Turret", this);
        
    }
    
    protected TalonFXConfiguration getHeadingMotorConfiguration() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.Slot0.kS = 0.23125;
        config.Slot0.kV = 0.097048;
        config.Slot0.kA = 0.0013363;
        config.Slot0.kP = 25;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        
        config.MotionMagic.MotionMagicCruiseVelocity = 25;
        config.MotionMagic.MotionMagicAcceleration = 25;
        config.MotionMagic.MotionMagicJerk = 1000;
        
        return config;
        
    }

    public void stopMotor() {
        
        this.headingMotor.stopMotor();

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
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.addDoubleProperty(
            "Turret Angles/Turret Heading (Degrees)",
            () -> this.getHeading().getHeading().in(Turret.DEFAULT_HEADING_UNITS),
            (double angle) -> this.goToHeading(
                TurretHeading.fromHeading(Turret.DEFAULT_HEADING_UNITS.of(angle))
            )
        );
        
    //    builder.addDoubleArrayProperty(
    //        "Turret PID", 
    //        () -> new double[]{this.anglePidController.getP(), this.anglePidController.getD()}, 
    //        (double[] pd) -> {
    //            this.anglePidController.setP(pd[0]);
    //            this.anglePidController.setD(pd[1]);
    //        }
    //    );
        
    }

    public class Commands {
        
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

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {

            return sysIdRoutine.quasistatic(direction);

        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {

            return sysIdRoutine.dynamic(direction);

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
        
    }
}
