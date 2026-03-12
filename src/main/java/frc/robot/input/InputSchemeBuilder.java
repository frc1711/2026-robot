package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.state.IntakePosition;

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
		
//		this.robot.intake.setDefaultCommand(this.robot.complexCommands.autofeedMailbox());
		
		return this;
		
	}
	
	public InputSchemeBuilder useControllerJoysticksForDriving(
		CommandXboxController controller
	) {
		
		this.robot.swerve.setDefaultCommand(this.robot.complexCommands.drive(controller));
		
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
	
//	public InputSchemeBuilder useXButtonForEnablingHeadingLock(
//		CommandXboxController controller
//	) {
//		
//		controller.x().onTrue(this.robot.swerve.commands.enabledPOIHeadingLock(
//			VirtualField.getReefCenterPoint()
//		));
//		
//		DoubleSupplier rotationInput = DoubleSupplierBuilder.getRotationDoubleSupplier(controller);
//		(new Trigger(() -> rotationInput.getAsDouble() != 0)).onTrue(
//			this.robot.swerve.commands.disableHeadingLock()
//		);
//		
//		return this;
//		
//	}
	
//	public ControlsSchemeBuilder useDPadForRobotRelativeDriving(CommandXboxController controller) {
//
//		Swerve.DriveMode mode = Swerve.DriveMode.FIELD_RELATIVE;
//		LinearVelocity speed = FeetPerSecond.of(1.5);
//		DoubleSupplier rotation = InputUtilities.getRotationDoubleSupplier(controller);
//
//		controller.povUp().whileTrue(this.robot.swerve.commands.drive(() -> new ChassisSpeeds(speed, InchesPerSecond.of(0), rotation.getAsDouble()), mode, false));
//		controller.povDown().whileTrue(this.robot.swerve.commands.drive(() -> new ChassisSpeeds(speed.times(-1), InchesPerSecond.of(0), InputUtilities.getRotationDoubleSupplier(controller)), mode, false));
//		controller.povDown().whileTrue(this.robot.swerve.commands.driveForward(speed.times(-1), mode));
//		controller.povLeft().whileTrue(this.robot.swerve.commands.driveLeft(speed, mode));
//		controller.povRight().whileTrue(this.robot.swerve.commands.driveLeft(speed.times(-1), mode));
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
	
	public InputSchemeBuilder useAButtonToShoot(CommandXboxController controller) {
		
		controller.a().whileTrue(
			this.robot.complexCommands.shoot()
				.alongWith(this.robot.intake.commands.pulse())
		);
		
		return this;
		
	}
	
	public InputSchemeBuilder useBButtonToPulse(CommandXboxController controller) {
		
		controller.b().whileTrue(this.robot.intake.commands.pulse());
		
		return this;
		
	}
	
	public InputSchemeBuilder useXButtonToIntake(CommandXboxController controller) {
		
		controller.x().whileTrue(this.robot.complexCommands.intake());
		
		return this;
		
	}
	
	public InputSchemeBuilder useDPadToControlIntakeExtension(
		CommandXboxController controller
	) {
		
		controller.povDown().whileTrue(this.robot.intake.commands.extend(-0.1));
		controller.povUp().whileTrue(this.robot.intake.commands.extend(0.1));
		
		return this;
		
	}
	
	public InputSchemeBuilder useDPadToControlTurretRotation(
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
