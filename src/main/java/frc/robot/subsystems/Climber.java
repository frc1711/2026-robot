package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final TalonFX climberMotor;

    private final PIDController climberPidController;
    
    public Climber(int climberMotorID) {
        this.climberMotor = new TalonFX(climberMotorID);

        this.climberPidController = new PIDController(
            1, 
            0, 
            0
        );
    }
}
