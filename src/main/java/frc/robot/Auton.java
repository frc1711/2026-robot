package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.configuration.Direction;
import frc.robot.configuration.RobotDimensions;
import frc.robot.subsystems.Turret;
import frc.robot.util.ChassisSpeedsSupplierBuilder;
import frc.robot.util.PoseBuilder;
import frc.robot.util.VirtualField;

import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public enum Auton {

	NONE(
		"None (No Auton)",
		robot -> new WaitCommand(0)
	),
	
	TRENCH_SHOT("Trench Shot", robot ->
		robot.complexCommands.shoot(Turret.WheelSpeeds.FAR_SHOT, false)
	),

	BUMP_SHOT("Bump Shot", robot ->
		robot.complexCommands.shoot(Turret.WheelSpeeds.CLOSE_SHOT, false)
	),
	
	DEPOT_RUN("Depot Run", robot ->
		robot.swerve.commands.goToPosition3(
			PoseBuilder.fromPose(new Pose2d(VirtualField.getDepotFaceCenterPoint(), Rotation2d.kZero))
				.withTranslation(PoseBuilder.CoordinateSystem.FIELD_RELATIVE, Inches.of(20), Direction.LEFT)
				.withHeading(PoseBuilder.CoordinateSystem.FIELD_RELATIVE, Degrees.of(225))
				.get()
		).andThen(
			robot.complexCommands.intake()
				.alongWith(
					robot.swerve.commands.drive(ChassisSpeedsSupplierBuilder.right(FeetPerSecond.of(1))
						.withFieldRelative(robot.swerve))
				)
				.withTimeout(3)
		).andThen(
//			robot.swerve.commands.drive(
//				ChassisSpeedsSupplierBuilder.forwards(FeetPerSecond.of(2))
//					.withFieldRelative(robot.swerve)
//			).withTimeout(2)
//		/*)*/.andThen(
			robot.swerve.commands.goToPosition3(PoseBuilder.getHubShootingPose(Feet.of(9), Degrees.of(-30)).get())
		).andThen(
			robot.complexCommands.shoot(Turret.WheelSpeeds.MID_SHOT, false)
				.withTimeout(Seconds.of(15))
		)
	),
	
	DEPOT_RUN_WITH_INITIAL_BARRAGE("Depot Run (with initial barrage)", robot ->
		robot.complexCommands.shoot(Turret.WheelSpeeds.CLOSE_SHOT, false)
			.withTimeout(Seconds.of(5))
			.andThen(robot.swerve.commands.goToPosition3(
				PoseBuilder.fromPose(new Pose2d(VirtualField.getDepotFaceCenterPoint(), Rotation2d.kZero))
					.withTranslation(PoseBuilder.CoordinateSystem.FIELD_RELATIVE, RobotDimensions.ROBOT_LENGTH.div(2), Direction.FORWARDS)
					.withHeading(PoseBuilder.CoordinateSystem.FIELD_RELATIVE, Direction.BACKWARDS)
					.get()
			))
			.andThen(
				robot.complexCommands.intake()
					.alongWith(robot.swerve.commands.drive(ChassisSpeedsSupplierBuilder.forwards(InchesPerSecond.of(9))))
					.withTimeout(2)
			).andThen(
				robot.swerve.commands.drive(ChassisSpeedsSupplierBuilder.backwards(FeetPerSecond.of(2)))
					.withTimeout(2)
			).andThen(
				robot.swerve.commands.goToPosition3(PoseBuilder.getHubShootingPose(Feet.of(9), Degrees.of(-30)).get())
			).andThen(
				robot.complexCommands.shoot(Turret.WheelSpeeds.MID_SHOT, false)
					.withTimeout(Seconds.of(15))
			)
	);

	/**
	 * The Shuffleboard widget used for selecting the auton to run.
	 */
	private static final SendableChooser<Auton> SHUFFLEBOARD_SELECTOR =
		new SendableChooser<>();

	/**
	 * The default Alliance color to assume if no alliance color is able to be
	 * fetched from the FMS.
	 */
	private static final Alliance DEFAULT_AUTON_ALLIANCE = Alliance.Red;

	/**
	 * The default auton to run if no auton is explicitly selected.
	 */
	private static final Auton DEFAULT_AUTON = Auton.NONE;

	/**
	 * A flag indicating whether or not the Shuffleboard auton selector has been
	 * initialized.
	 */
	private static boolean hasShuffleboardSelectorBeenInitialized = false;

	/**
	 * The human-readable name of this auton.
	 */
	private final String humanReadableName;

	/**
	 * The function that supplies the Command object for this auton.
	 */
	private final Function<RobotContainer, Command> commandSupplier;

	/**
	 * Constructs a new Auton with the given name and command supplier.
	 *
	 * @param name The human-readable name of this auton.
	 * @param commandFunction The function that supplies the Command object for
	 * this auton.
	 */
	Auton(String name, Function<RobotContainer, Command> commandFunction) {

		this.humanReadableName = name;
		this.commandSupplier = commandFunction;

	}

	/**
	 * Initializes the Shuffleboard widget used for selecting the starting
	 * position of the robot.
	 */
	public static void initializeShuffleboardSelector() {

		if (Auton.hasShuffleboardSelectorBeenInitialized) return;
		else Auton.hasShuffleboardSelectorBeenInitialized = true;

		Auton.SHUFFLEBOARD_SELECTOR.setDefaultOption(
			Auton.DEFAULT_AUTON.getHumanReadableName(),
			Auton.DEFAULT_AUTON
		);

		for (Auton auton: Auton.values()) {

			if (auton == Auton.DEFAULT_AUTON) continue;

			Auton.SHUFFLEBOARD_SELECTOR.addOption(
				auton.getHumanReadableName(),
				auton
			);

		}

		Shuffleboard.getTab("Pre-match Tab").add(
			"Auton Chooser",
			Auton.SHUFFLEBOARD_SELECTOR
		);

	}

	/**
	 * Returns the auton selected by the driver from the Shuffleboard widget.
	 *
	 * @return The auton selected by the driver from the Shuffleboard widget.
	 */
	public static Auton getSelectedAuton() {

		return Auton.SHUFFLEBOARD_SELECTOR.getSelected();

	}

	/**
	 * Runs the auton currently selected by the Shuffleboard widget.
	 *
	 * @param robot The robot on which to run the auton.
	 */
	public static Command runSelectedAuton(RobotContainer robot) {

		Command autonCommand = Auton.getSelectedAuton().getCommand(robot);

		CommandScheduler.getInstance().schedule(autonCommand);

		return autonCommand;

	}

	/**
	 * Returns the Command object for this auton.
	 *
	 * @param robot The robot on which to run the auton.
	 * @return The Command object for this auton.
	 */
	public Command getCommand(RobotContainer robot) {

		return this.commandSupplier.apply(robot);

	}

	/**
	 * Returns the human-readable name of this auton.
	 *
	 * @return The human-readable name of this auton.
	 */
	public String getHumanReadableName() {

		return this.humanReadableName;

	}

}
