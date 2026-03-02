package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FunctionUtilities;

public class SwerveDriveCommand extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    private final SwerveSubsystem swerve;


    public SwerveDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, SwerveSubsystem swerve) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.swerve = swerve;
        this.addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double rotation = rotSupplier.getAsDouble();

        x = Math.pow(FunctionUtilities.applyDeadband(x, OperatorConstants.DEADBAND), 2);
        y = Math.pow(FunctionUtilities.applyDeadband(y, OperatorConstants.DEADBAND), 2);
        rotation = Math.pow(FunctionUtilities.applyDeadband(rotation, OperatorConstants.DEADBAND), 2);

        x *= swerve.maxSpeed;
        y *= swerve.maxSpeed;
        rotation *= swerve.maxSpeed  * (Math.PI / 2);

        //System.out.println("X: " + x + " Y: " + y + " Rotation: " + rotation);

        swerve.drive(x, y, rotation);
    }
}
