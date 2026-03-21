package frc.robot.input;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.configuration.Direction;
import frc.robot.math.DoubleSupplierBuilder;
import frc.robot.math.Point;
import frc.robot.math.PointSupplierBuilder;
import frc.robot.state.IntakePosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.VirtualField;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

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
	
	public InputSchemeBuilder disableRadiusLockOnJoystickInput(
		CommandXboxController controller
	) {
		
		Supplier<Point> translationInput = PointSupplierBuilder.getTranslationPointSupplier(controller);
		Trigger driverIsTryingToManuallyTranslate = new Trigger(() -> translationInput.get().getNorm() != 0);
		
		driverIsTryingToManuallyTranslate.whileTrue(this.robot.swerve.commands.disablePOIRadiusLock());
		
		return this;
		
	}
	
	public InputSchemeBuilder disableHeadingLockOnJoystickInput(
		CommandXboxController controller
	) {
		
		DoubleSupplier rotationInput = DoubleSupplierBuilder.getRotationDoubleSupplier(controller);
		Trigger driverIsTryingToManuallyRotate = new Trigger(() -> rotationInput.getAsDouble() != 0);
		
		driverIsTryingToManuallyRotate.whileTrue(this.robot.swerve.commands.disableHeadingLock());
		
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
		
		controller.start().onTrue(this.robot.complexCommands.resetFieldHeading());
		
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
	
	public InputSchemeBuilder useYButtonToHeadingLockToHub(CommandXboxController controller) {
		
		controller.y().onTrue(this.robot.swerve.commands.enablePOIHeadingLock(
			VirtualField.getHubCenterPoint(),
			Direction.RIGHT
		));
		
		return this;
		
	}
	
	public InputSchemeBuilder useBButtonToRadiusLockToHub(CommandXboxController controller) {
		
		controller.b().onTrue(this.robot.swerve.commands.enablePOIRadiusLock(
			VirtualField.getHubCenterPoint(),
			Feet.of(8)
		));
		
		controller.b().onFalse(this.robot.swerve.commands.disablePOIRadiusLock());
		
		return this;
		
	}
	
	public InputSchemeBuilder useTriggersForSlowMode(CommandXboxController controller) {
		
		controller.leftTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD)
			.or(controller.rightTrigger(InputSchemeBuilder.TRIGGER_THRESHOLD))
			.whileTrue(this.robot.swerve.commands.useDriveSpeedMultiplier(0.4));
		
		return this;
		
	}
	
	public InputSchemeBuilder useAButtonToShoot(CommandXboxController controller) {
		
		controller.a().whileTrue(this.robot.complexCommands.shoot());
		
		return this;
		
	}
	
	public InputSchemeBuilder useBButtonToPulse(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.intake.commands.pulseV3());
		
		return this;
		
	}
	
	public InputSchemeBuilder useBButtonToSafelyShoot(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.complexCommands.shoot(
			Turret.WheelSpeeds.MID_SHOT,
			Seconds.of(0.5),
			false,
			false
		));
		
		return this;
		
	}
	
	public InputSchemeBuilder useXButtonToIntake(CommandXboxController controller) {
		
		controller.x().whileTrue(this.robot.complexCommands.intake());
		
		return this;
		
	}
	
	public InputSchemeBuilder useDPadDownToOuttake(CommandXboxController controller) {
		
		controller.povDown().whileTrue(this.robot.complexCommands.outtake());
		
		return this;
		
	}
	
	public InputSchemeBuilder useYButtonToShootManually(CommandXboxController controller) {
		
		controller.y().whileTrue(
			this.robot.complexCommands.shoot(Turret.WheelSpeeds.MID_SHOT, false)
		);
		
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
	
	public InputSchemeBuilder useBumpersToControlIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.leftBumper().onTrue(this.robot.intake.commands.goToPosition(
			IntakePosition.FULLY_STOWED
		));
		
		controller.rightBumper().onTrue(this.robot.intake.commands.goToPosition(
			IntakePosition.PARTIALLY_STOWED
		));
		
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
