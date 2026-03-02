package frc.robot.subsystems;

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

    private final SlewRateLimiter intakeLimiter = new SlewRateLimiter(20);

    public Intake(int intakeMotorPort) {
        this.intakeMotor = new TalonFX(intakeMotorPort);
        this.intakeMotor.getConfigurator().apply(getIntakeConfig());
    }

    private TalonFXConfiguration getIntakeConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = config.Slot0;
        slot0.kV = IntakeConstants.INTAKEV;
        slot0.kP = IntakeConstants.INTAKEP;

        return config;
    }

    public void intake(double speed) {
        this.intakeTargetRPS = speed;
    }

    @Override
    public void periodic() {
        double limitedSpeed = this.intakeLimiter.calculate(this.intakeTargetRPS);

        this.intakeMotor.setControl(this.intakeRequest.withVelocity(limitedSpeed));
    }

    public class Commands {
        public Command intake(double speed) {
            return Intake.this.runOnce(
                () -> Intake.this.intake(speed * IntakeConstants.INTAKEMAXRPS)
            );
        }
    }
}
