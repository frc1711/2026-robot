package frc.robot.input.inputschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.input.InputScheme;
import frc.robot.input.InputSchemeBuilder;
import frc.robot.state.IntakePosition;

public class StandardTeleoperativeInputsScheme implements InputScheme {
	
	@Override
	public void configureControllerInputs(
		RobotContainer robotContainer,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		(new InputSchemeBuilder(robotContainer))
			.configureDefaultRobotCommands()
			.useControllerJoysticksForDriving(controller1)
			.useStartButtonToResetFieldHeading(controller1)
			.useBumpersToEnableHeadingLock(controller1)
			.useAButtonToShoot(controller2)
			.useBButtonToPulse(controller2)
			.useXButtonToIntake(controller2)
			.useBumpersToControlIntakeExtension(controller2)
			.useTriggersForIndexing(controller2)
			.useBackButtonToCalibrateIntakeExtension(controller2);
		
	}
	
	@Override
	public void init(
		RobotContainer robot,
		CommandXboxController controller1,
		CommandXboxController controller2
	) {
		
		InputScheme.super.init(robot, controller1, controller2);
		
		robot.intake.goToPosition(IntakePosition.FULLY_STOWED);
		
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
