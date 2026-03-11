package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;

public class Highway extends SubsystemBase {
    
    protected final TalonFX highwayMotor;
    
    protected final TalonFX agitatorOfDoomMotor;

    public final Commands commands = new Commands();

    public Highway() {
        this.highwayMotor = new TalonFX(CANDevice.INDEXER_MOTOR_CONTROLLER.id);
        this.agitatorOfDoomMotor = new TalonFX(21);
    }

    private void setSpeed(double speed) {
        this.highwayMotor.set(-speed);
        this.agitatorOfDoomMotor.set(speed);
    }
    
    public class Commands {
        public Command forward() {
            return Highway.this.startEnd(
                () -> Highway.this.setSpeed(0.8),
                () -> Highway.this.setSpeed(0)
            );
        }

        public Command backward() {
            return Highway.this.startEnd(
                () -> Highway.this.setSpeed(-0.8), 
                () -> Highway.this.setSpeed(0)
            );
        }
    }
    
}
