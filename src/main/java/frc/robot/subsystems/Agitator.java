package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;

import static edu.wpi.first.units.Units.Seconds;

public class Agitator extends SubsystemBase {
    
    protected static final double DEFAULT_SPEED = 0.3;
    
    protected final TalonFX motor;

    public final Commands commands;

    public Agitator() {
        
        this.motor = new TalonFX(CANDevice.AGITATOR_MOTOR_CONTROLLER.id);
        this.commands = new Commands();
        
        this.motor.getConfigurator().apply(Agitator.getMotorConfig());
        
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
    
    public class Commands {
        
        public Command spin(double speed) {
            
            return Agitator.this.startEnd(
                () -> Agitator.this.motor.set(speed),
                Agitator.this::stop
            );
            
        }
        
        public Command spin() {
            
            return this.spin(Agitator.DEFAULT_SPEED);
            
        }
        
        public Command agitate(double speed) {
            
            return this.spin(speed).withTimeout(Seconds.of(0.25))
                .andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(0.25)))
                .repeatedly();
            
        }
        
        public Command agitate() {
            
            return this.agitate(Agitator.DEFAULT_SPEED);
            
        }
        
    }
    
}
