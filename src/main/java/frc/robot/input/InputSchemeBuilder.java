package frc.robot.input;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.configuration.Direction;
import frc.robot.state.IntakePosition;
import frc.robot.subsystems.Swerve;
import frc.robot.util.VirtualField;

import static edu.wpi.first.units.Units.Degrees;

public class InputSchemeBuilder {
	
	/**
	 * The threshold at which the triggers of the controller should be
	 * considered to be pressed.
	 */
	protected static final double TRIGGER_THRESHOLD = 0.5;
	
	protected final RobotContainer robot;
	
	public InputSchemeBuilder(RobotContainer robot) {
		
		this.robot = robot;
		
	}
	
	public InputSchemeBuilder configureDefaultRobotCommands() {
		
//		this.robot.intake.setDefaultCommand(this.robot.intake.commands.goToPosition(IntakePosition.PARTIALLY_STOWED));
		
		return this;
		
	}
	
	public InputSchemeBuilder useControllerJoysticksForDriving(
		CommandXboxController controller
	) {
		
		this.robot.swerve.setDefaultCommand(
			this.robot.complexCommands.drive(controller)
		);
		
		return this;
		
	}
	
//	public InputSchemeBuilder useRBButtonForSlowMode(
//		CommandXboxController controller
//	) {
//		
//		controller.rightBumper().onTrue(new InstantCommand(() -> this.robot.swerve.isSlowModeEnabled = true));
//		controller.rightBumper().onFalse(new InstantCommand(() -> this.robot.swerve.isSlowModeEnabled = false));
//		
//		return this;
//		
//	}
	
	public InputSchemeBuilder useStartButtonToResetFieldHeading(CommandXboxController controller) {
		
		controller.start().onTrue(
			this.robot.swerve.commands.calibrateFieldRelativeHeading()
		);
		
		return this;
		
	}
	
	public InputSchemeBuilder useBumpersToEnableHeadingLock(CommandXboxController controller) {
		
		Angle increment = Degrees.of(90);
		Swerve.Commands swerve = this.robot.swerve.commands;
		
		controller.leftBumper()
			.onTrue(swerve.jumpToNextHeadingLockAngle(increment, false));
		
		controller.rightBumper()
			.onTrue(swerve.jumpToNextHeadingLockAngle(increment, true));
		
		return this;
		
	}
	
	public InputSchemeBuilder useABXYButtonsToUseHeadingLock(CommandXboxController controller) {
		
		controller.y().onTrue(this.robot.swerve.commands.enableStaticHeadingLock(Direction.FORWARDS));
		controller.x().onTrue(this.robot.swerve.commands.enableStaticHeadingLock(Direction.LEFT));
		controller.b().onTrue(this.robot.swerve.commands.enableStaticHeadingLock(Direction.RIGHT));
		controller.a().onTrue(this.robot.swerve.commands.enableStaticHeadingLock(Direction.BACKWARDS));
		
		return this;
		
	}
	
	public InputSchemeBuilder useYButtonToLockToHub(CommandXboxController controller) {
		
		controller.y().onTrue(this.robot.swerve.commands.enabledPOIHeadingLock(
			VirtualField.getHubCenterPoint(),
			Direction.RIGHT
		));
		
		return this;
		
	}
	
	public InputSchemeBuilder useAButtonToShoot(CommandXboxController controller) {
		
		controller.a().whileTrue(this.robot.complexCommands.shoot());
		
		return this;
		
	}
	
	public InputSchemeBuilder useBButtonToPulse(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.intake.commands.pulseV1());
		
		return this;
		
	}
	
	public InputSchemeBuilder useXButtonToIntake(CommandXboxController controller) {
		
		controller.x().whileTrue(this.robot.complexCommands.intake());
		
		return this;
		
	}
	
	public InputSchemeBuilder useDPadToControlRawIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.povDown().whileTrue(this.robot.intake.commands.extend(-0.1));
		controller.povUp().whileTrue(this.robot.intake.commands.extend(0.1));
		
		return this;
		
	}
	
	public InputSchemeBuilder useDPadToControlRawTurretRotation(
		CommandXboxController controller
	) {
		
		controller.povLeft().whileTrue(this.robot.turret.commands.adjustHeading(-0.1));
		controller.povRight().whileTrue(this.robot.turret.commands.adjustHeading(0.1));
		
		return this;
		
	}
	
	public InputSchemeBuilder useTriggersToControlRawIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.leftTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD)
			.onTrue(this.robot.intake.commands.extend(-0.1));
		
		controller.rightTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD)
			.onTrue(this.robot.intake.commands.extend(0.1));
		
		return this;
		
	}
	
	public InputSchemeBuilder useTriggersToControlIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.leftTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD)
			.onTrue(this.robot.intake.commands.goToPosition(
				IntakePosition.FULLY_STOWED
			));
		
		controller.rightTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD)
			.onTrue(this.robot.intake.commands.goToPosition(
				IntakePosition.PARTIALLY_STOWED
			));
		
		return this;
		
	}
	
	public InputSchemeBuilder useTriggersForIndexing(
		CommandXboxController controller
	) {
		
		controller.leftTrigger()
			.whileTrue(this.robot.indexer.commands.backward());
		
		controller.rightTrigger()
			.whileTrue(this.robot.indexer.commands.forward());
		
		return this;
		
	}
	
	public InputSchemeBuilder useBackButtonToCalibrateIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.back()
			.onTrue(this.robot.intake.commands.calibrateExtensionLimits());
		
		return this;
		
	}
	
}
