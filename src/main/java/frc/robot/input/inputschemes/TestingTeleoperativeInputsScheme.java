package frc.robot.input.inputschemes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.input.InputScheme;
import frc.robot.input.InputSchemeBuilder;
import frc.robot.state.IntakePosition;

public class TestingTeleoperativeInputsScheme implements InputScheme {
	
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
			.useAButtonToShoot(controller1)
			.useBButtonToPulse(controller1)
			.useXButtonToIntake(controller1)
			.useDPadToControlIntakeExtension(controller1)
			.useBumpersToControlIntakeExtension(controller1)
			.useBackButtonToCalibrateIntakeExtension(controller1);
		
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
