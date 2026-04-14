package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.Seconds;

public class Agitator extends SubsystemBase {
    
    protected static final double DEFAULT_SPEED = 0.5;
    
    protected final TalonFX motor;

    public final Commands commands;

    public double rollerSpeed = 0.5;

    public Agitator() {
        
        this.motor = new TalonFX(CANDevice.AGITATOR_MOTOR_CONTROLLER.id);
        this.commands = new Commands();
        
        this.motor.getConfigurator().apply(Agitator.getMotorConfig());

        SmartDashboard.putData("Indexer", this);
        
    }
    
    protected static TalonFXConfiguration getMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        
        return config;
        
    }
    
    public void stop() {
        
        this.motor.stopMotor();
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "IndexerSpeeds/Roller Speeds", 
            () -> this.rollerSpeed, 
            (double d) -> this.rollerSpeed = d
        );
    }
    
    public class Commands {
        
        public Command spin(boolean reversed) {
            
            return Agitator.this.startEnd(
                () -> Agitator.this.motor.set(reversed ? Agitator.this.rollerSpeed : -Agitator.this.rollerSpeed),
                Agitator.this::stop
            );
            
        }
        
        public Command agitate(double speed) {
            
            return this.spin(false).withTimeout(Seconds.of(0.75))
                .andThen(this.spin(true).withTimeout(Seconds.of(0.25)))
                .repeatedly();
            
        }
        
        public Command agitate() {
            
            return this.agitate(Agitator.DEFAULT_SPEED);
            
        }
        
    }
    
}
