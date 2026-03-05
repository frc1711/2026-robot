package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AdvancedCommands {
    private final RobotContainer robotContainer;

    public AdvancedCommands(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /*public Command aimTurretWithVision() {
        double turretAngleRad = Units.degreesToRadians(robotContainer.vision.getAngleFromHub());

        return robotContainer.turret.commands.changeAngle(() -> turretAngleRad);
    }*/
}
