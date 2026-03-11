package frc.robot.input.inputschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.input.InputScheme;
import frc.robot.input.InputSchemeBuilder;

public class TestingTeleoperativeInputsScheme implements InputScheme {
	
	@Override
	public void configureControllerInputs(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new InputSchemeBuilder(robotContainer))
			.configureDefaultRobotCommands();
//			.useControllerJoysticksForDriving(controller1)
//			.useTriggersToLoad(controller1)
//			.useABXYToScoreCoral(controller1)
//			.usePOVButtonsToSwitchReefScoringModes(controller1)
//			.useBumpersToClimb(controller1)
//			.useBackButtonToUnclimb(controller1)
//			.useStartToResetFieldHeading(controller1);
		
	}
	
	@Override
	public void periodic(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		InputScheme.super.periodic(robot, controller1, controller2);
		
	}
	
	@Override
	public void exit(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		InputScheme.super.exit(robot, controller1, controller2);
		
	}
	
}
