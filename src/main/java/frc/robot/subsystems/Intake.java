package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.state.IntakePosition;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

public class Intake extends SubsystemBase {
    
    protected TalonFX leftExtensionMotor;
    
    protected TalonFX rightExtensionMotor;

    protected TalonFX rollerMotor;
    
    protected MotionMagicVoltage extensionRequest;

    public final Commands commands;
    
    public final Triggers triggers;

    public Intake() {
        
        this.leftExtensionMotor = new TalonFX(24);
        this.rightExtensionMotor = new TalonFX(19);
        this.rollerMotor = new TalonFX(20);
        this.extensionRequest = new MotionMagicVoltage(Rotations.zero());
        this.commands = new Commands();
        this.triggers = new Triggers();
        
        this.leftExtensionMotor.getConfigurator().apply(Intake.getLeftExtensionMotorConfig());
        this.rightExtensionMotor.getConfigurator().apply(Intake.getRightExtensionMotorConfig());
        this.extensionRequest.Slot = 0;
        
    }

    protected static TalonFXConfiguration getLeftExtensionMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kS = 0.1;
        config.Slot0.kV = 0.5;
        
        config.Slot0.kP = 6;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 50;
        config.MotionMagic.MotionMagicJerk = 1000;
        
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        
        return config;
        
    }
    
    protected static TalonFXConfiguration getRightExtensionMotorConfig() {
        
        TalonFXConfiguration config = Intake.getLeftExtensionMotorConfig();
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        return config;
        
    }
    
    public IntakePosition getPosition() {
        
        return IntakePosition.fromMotorShaftAngle(
            this.leftExtensionMotor.getPosition().getValue()
        );
        
    }
    
    public void goToPosition(IntakePosition position) {

        Intake.this.leftExtensionMotor.setControl(
            Intake.this.extensionRequest.withPosition(
                position.getMotorShaftAngle()
            )
        );
        
        Intake.this.rightExtensionMotor.setControl(
            Intake.this.extensionRequest.withPosition(
                position.getMotorShaftAngle()
            )
        );
        
    }

    public class Commands {
        
        public Command calibrateExtensionLimits() {
            
            return Intake.this.runOnce(() -> {
                Intake.this.leftExtensionMotor.setPosition(Rotations.zero());
                Intake.this.rightExtensionMotor.setPosition(Rotations.zero());
            });
            
        }

        public Command extend(double speed) {

            return Intake.this.startEnd(
                    () -> {
                        Intake.this.leftExtensionMotor.set(speed);
                        Intake.this.rightExtensionMotor.set(speed);
                    },
                    () -> {
                        Intake.this.leftExtensionMotor.stopMotor();
                        Intake.this.rightExtensionMotor.stopMotor();
                    }
            );

        }
        
        public Command pulse() {
            
            IntakePosition innerPosition = new IntakePosition(Inches.of(5));
            IntakePosition outerPosition = IntakePosition.PARTIALLY_STOWED;
            
            return this.goToPosition(innerPosition)
                .andThen(this.goToPosition(outerPosition))
                .repeatedly();
            
        }
        
        public Command waitUntilAtPosition(
            IntakePosition position,
            Distance tolerance
        ) {
            
            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(
                Intake.this.triggers.isAtPosition(position, tolerance)
            );
            
        }
        
        public Command goToPosition(IntakePosition position) {
            
            return Intake.this
                .runOnce(() -> Intake.this.goToPosition(position))
                .andThen(this.waitUntilAtPosition(position, Inches.of(0.25)));
            
        }
        
        public Command intake(DoubleSupplier speed) {
            
            return Intake.this.startEnd(
                () -> Intake.this.rollerMotor.set(speed.getAsDouble()),
                () -> Intake.this.rollerMotor.stopMotor()
            );
            
        }
        
        public Command outtake(DoubleSupplier speed) {
            
            return Intake.this.startEnd(
                () -> Intake.this.rollerMotor.set(-speed.getAsDouble()),
                () -> Intake.this.rollerMotor.stopMotor()
            );
            
        }
        
    }
    
    public class Triggers {
        
        public Trigger isAtPosition(IntakePosition position, Distance tolerance) {
            
            return new Trigger(() -> Intake.this.getPosition().getOffsetFromFullyStowed().isNear(
                position.getOffsetFromFullyStowed(),
                tolerance
            ));
            
        }
        
    }
    
}
