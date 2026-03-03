package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    private double intakeTargetRPS = 0;
    private final MotionMagicVelocityVoltage intakeRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    private final SlewRateLimiter intakeLimiter = new SlewRateLimiter(25);

    public final Commands commands = new Commands();

    public Intake(int intakeMotorPort) {
        this.intakeMotor = new TalonFX(intakeMotorPort);
        this.intakeMotor.getConfigurator().apply(getIntakeConfig());
    }

    private TalonFXConfiguration getIntakeConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kV = IntakeConstants.INTAKEV;
        config.Slot0.kP = IntakeConstants.INTAKEP;

        config.MotionMagic.MotionMagicAcceleration = 150;
        config.MotionMagic.MotionMagicJerk = 1500;

        return config;
    }

    public void intake(double speed) {
        System.out.println("Spinning");
        this.intakeMotor.set(speed);
    }

    /*@Override
    public void periodic() {
        double limitedSpeed = this.intakeLimiter.calculate(this.intakeTargetRPS);

        this.intakeMotor.setControl(this.intakeRequest.withVelocity(limitedSpeed));
    }*/

    public class Commands {
        public Command intake(DoubleSupplier speed) {
            return Intake.this.startEnd(
                () -> Intake.this.intake(speed.getAsDouble()),
                () -> Intake.this.intake(0)
            );
        }
    }
}
