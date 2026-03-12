package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;

public class Indexer extends SubsystemBase {
    
    protected static final double DEFAULT_SPEED = 0.8;
    
    protected final TalonFX motor;
    
    public final Commands commands;

    public Indexer() {
        
        this.motor = new TalonFX(CANDevice.INDEXER_MOTOR_CONTROLLER.id);
        this.commands = new Commands();
        
        this.motor.getConfigurator().apply(Indexer.getMotorConfig());
        
    }
    
    protected static TalonFXConfiguration getMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        
        return config;
        
    }
    
    public void stop() {
        
        this.motor.stopMotor();
        
    }
    
    public class Commands {
        
        public Command spin(double speed) {
            
            return Indexer.this.startEnd(
                () -> Indexer.this.motor.set(speed),
                Indexer.this::stop
            );
            
        }
        
        public Command spin() {
            
            return this.spin(Indexer.DEFAULT_SPEED);
            
        }
        
        public Command forward() {
            
            return this.spin(Indexer.DEFAULT_SPEED);
        }

        public Command backward() {
            
            return this.spin(-Indexer.DEFAULT_SPEED);
            
        }
    }
    
}
