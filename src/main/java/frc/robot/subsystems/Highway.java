package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Highway extends SubsystemBase {
    private final TalonFX highwayMotor;
    private final TalonFX agitatorOfDoomMotor;

    public final Commands commands = new Commands();

    public Highway(int highwayID, int agitatorID) {
        this.highwayMotor = new TalonFX(highwayID);
        this.agitatorOfDoomMotor = new TalonFX(agitatorID);
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
