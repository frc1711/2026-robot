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

public class Indexer extends SubsystemBase {
    
    protected static final double BELT_DEFAULT_SPEED = 0.7;
    
    protected final TalonFX beltMotor;
    
    public final Commands commands;

    public Indexer() {
        
        this.beltMotor = new TalonFX(CANDevice.INDEXER_MOTOR_CONTROLLER.id);
        this.commands = new Commands();
        
        this.beltMotor.getConfigurator().apply(Indexer.getMotorConfig());

        SmartDashboard.putData("Indexer", this);
        
    }
    
    protected static TalonFXConfiguration getMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        
        return config;
        
    }
    
    public void stop() {
        
        this.beltMotor.stopMotor();
        
    }
    
    public class Commands {
        
        public Command spin(boolean reversed) {
            
            return Indexer.this.startEnd(
                () -> Indexer.this.beltMotor.set(reversed ? -Indexer.BELT_DEFAULT_SPEED : Indexer.BELT_DEFAULT_SPEED),
                Indexer.this::stop
            );
            
        }
        
        public Command forward() {
            
            return this.spin(true);
        }

        public Command backward() {
            
            return this.spin(false);
            
        }
    }
    
}
