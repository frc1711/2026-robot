package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public interface InputScheme {
	
	/**
	 * Configures the controls for this control scheme.
	 */
	void configureControllerInputs(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	);
	
	default void periodic(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {}
	
	default void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {}
	
}
