package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    
    private final TalonFX turretMotor;

    private final PIDController anglePidController;

    private final boolean encoderReversed;

    public final Commands commands = new Commands();

    /**
     * The constructor method for the turret subsystem
     * 
     * @param turretID The CAN ID of the turret motor
     * @param encoderReversed If the encoder should be reversed
     */
    public Turret(int turretID, boolean encoderReversed) {
        // Initializations
        this.turretMotor = new TalonFX(turretID);

        this.anglePidController = new PIDController(
            Preferences.getDouble("TurretP", TurretConstants.TURRETP),
            0,
            Preferences.getDouble("TurretD", TurretConstants.TURRETD)
        );

        this.anglePidController.setTolerance(0.05);

        this.anglePidController.setSetpoint(0);
        
        this.encoderReversed = encoderReversed;
    }

    /**
     * 
     * @return The angle of the turret in radians
     */
    public double getAngle() {
        double rotations = this.turretMotor.getPosition().getValueAsDouble();
        double turretRotations = rotations;
        Rotation2d angle = Rotation2d.fromRotations(turretRotations);

        if (encoderReversed) {
            angle = angle.unaryMinus();
        }

        return angle.getRadians() * TurretConstants.TURRETGEARRATIO;
    }

    @Override
    public void periodic() {
        // Gets the pid controller's output
        double currentAngle = this.getAngle();
        double calculatedOutput = this.anglePidController.calculate(currentAngle);

        // Makes sure the turret doesn't over extend
        boolean overrun = (
            (calculatedOutput < 0 && currentAngle < TurretConstants.TURRETMINANGLE) ||
            (calculatedOutput > 0 && currentAngle > TurretConstants.TURRETMAXANGLE)
        );

        // Sets the correct voltage for the motor
        double finalOutput = overrun ? 0 : calculatedOutput;
        this.turretMotor.setVoltage(finalOutput);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Turret Angle",
            () -> Units.radiansToDegrees(getAngle()),
            (double angle) -> this.anglePidController.setSetpoint(Units.degreesToRadians(angle))
        );
        builder.addDoubleArrayProperty(
            "Turret PID", 
            () -> new double[]{this.anglePidController.getP(), this.anglePidController.getD()}, 
            (double[] pd) -> {
                this.anglePidController.setP(pd[0]);
                this.anglePidController.setD(pd[1]);
            }
        );
    }

    public boolean atSetpoint() {
        return this.anglePidController.atSetpoint();
    }

    public class Commands {
        /**
         * Increases the angle of the turret by the given value
         * 
         * @param additionSupplier The {@link java.util.function.DoubleSupplier DoubleSupplier} method for getting the increase in the angle, which should be in degrees
         */
        public Command changeAngle(DoubleSupplier additionSupplier) {
            return Turret.this.run(
                () -> Turret.this.anglePidController.setSetpoint(Turret.this.anglePidController.getSetpoint() + Units.degreesToRadians(additionSupplier.getAsDouble()))
            ).finallyDo(
                () -> {}
            );
        }

        public Command moveTillZero(DoubleSupplier supplier) {
            return Turret.this.run(
                () -> this.changeAngle(() -> Units.degreesToRadians(Math.signum(supplier.getAsDouble()) * 2))
            );
        }
    }
}
