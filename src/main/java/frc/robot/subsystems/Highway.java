package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Highway extends SubsystemBase {
    private final TalonFX highwayMotor;

    public final Commands commands = new Commands();

    public Highway(int highwayID) {
        this.highwayMotor = new TalonFX(highwayID);
    }

    public class Commands {
        public Command forward() {
            return Highway.this.startEnd(
                () -> Highway.this.highwayMotor.set(-0.6),
                () -> Highway.this.highwayMotor.set(0)
            );
        }

        public Command backward() {
            return Highway.this.run(
                () -> Highway.this.highwayMotor.set(0.6)
            ).finallyDo(
                () -> Highway.this.highwayMotor.set(0)
            );
        }
    }
    
}
