package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.configuration.RobotDimensions;
import frc.robot.math.Point;

import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import static frc.robot.configuration.Direction.*;
import static frc.robot.util.PoseBuilder.CoordinateSystem.*;

/**
 * A builder class for creating and manipulating robot poses.
 */
public class PoseBuilder implements Supplier<Pose2d> {

	/**
	 * The underlying pose representing the current 'state' of the pose being
	 * built.
	 */
	protected final Supplier<Pose2d> poseSupplier;

	/**
	 * Initializes a new RobotPoseBuilder with the given pose.
	 *
	 * @param poseSupplier The initial pose to build upon.
	 */
	protected PoseBuilder(Supplier<Pose2d> poseSupplier) {
	
		this.poseSupplier = poseSupplier;
	
	}

	/**
	 * Returns a new RobotPoseBuilder with the given pose.
	 *
	 * @param pose The pose to initialize the builder with.
	 * @return A RobotPoseBuilder with the given pose.
	 */
	public static PoseBuilder fromPose(Pose2d pose) {
		
		return new PoseBuilder(() -> pose);
		
	}

	/**
	 * Returns a RobotPoseBuilder representing a pose at the center of the
	 * virtual field, facing the positive X direction (0 degrees).
	 *
	 * @return A RobotPoseBuilder representing a pose at the center of the
	 * virtual field.
	 */
	public static PoseBuilder fromCenterFieldPose() {

		return new PoseBuilder(() -> new Pose2d(
			VirtualField.FIELD_LENGTH.div(2),
			VirtualField.FIELD_WIDTH.div(2),
			Rotation2d.kZero
		));

	}

	/**
	 * Returns a RobotPoseBuilder representing the pose of the AprilTag with
	 * the given ID.
	 *
	 * @param tagID The ID of the AprilTag to get the pose for.
	 * @return A RobotPoseBuilder representing the pose of the AprilTag with
	 * the given ID.
	 */
	public static PoseBuilder getAprilTagPose(IntSupplier tagID) {

		return new PoseBuilder(
			() -> VirtualField.getAprilTagByID(tagID.getAsInt()).pose.toPose2d()
		);

	}

	/**
	 * Returns a pose facing the AprilTag with the given ID.
	 *
	 * @param tagID The ID of the AprilTag to face.
	 * @return A RobotPoseBuilder representing a pose facing the AprilTag with
	 * the given ID.
	 */
	public static PoseBuilder getAprilTagFacingPose(IntSupplier tagID) {

		return PoseBuilder.getAprilTagPose(tagID)
			.withRotation(BACKWARDS)
			.withTranslation(ROBOT_RELATIVE, RobotDimensions.ROBOT_LENGTH.div(2), BACKWARDS);

	}
	
	public static PoseBuilder getHubShootingPose(
		Distance shootingRadius,
		Angle shootingAngle
	) {
		
		return PoseBuilder.fromPose(new Pose2d(VirtualField.getHubCenterPoint(), Rotation2d.kZero))
			.withHeading(FIELD_RELATIVE, RIGHT)
			.withRotation(shootingAngle)
			.withTranslation(ROBOT_RELATIVE, shootingRadius, RIGHT);
		
	}

	public PoseBuilder with(Function<Pose2d, Pose2d> function) {

		return new PoseBuilder(() -> function.apply(this.get()));

	}
	
	public PoseBuilder withTranslation(
		CoordinateSystem coordinateSystem,
		Translation2d translation
	) {

		boolean isRedOrField = coordinateSystem == RED_OUT ||
			coordinateSystem == CoordinateSystem.FIELD_RELATIVE;
		Translation2d adjustedTranslation = isRedOrField
			? translation.times(VirtualField.isRedAlliance() ? -1 : 1)
			: translation;
		CoordinateSystem adjustedCoordinateSystem = isRedOrField
			? CoordinateSystem.BLUE_OUT
			: coordinateSystem;

		return this.with(pose -> pose.plus(new Transform2d(
			adjustedTranslation.rotateBy(
				adjustedCoordinateSystem == CoordinateSystem.BLUE_OUT
					? pose.getRotation().times(-1)
					: Rotation2d.kZero
			),
			Rotation2d.kZero
		)));

	}
	
	public PoseBuilder withTranslation(
		CoordinateSystem coordinateSystem,
		Distance distance,
		Angle angle
	) {

		return this.withTranslation(
			coordinateSystem,
			new Point(distance, angle)
		);

	}
	
	public PoseBuilder withHeading(
		CoordinateSystem coordinateSystem,
		Rotation2d heading
	) {
		
		return switch (coordinateSystem) {
			
			case BLUE_OUT -> this.with(pose -> new Pose2d(
				pose.getTranslation(),
				heading
			));
			
			case RED_OUT -> this.with(pose -> new Pose2d(
				pose.getTranslation(),
				heading.plus(Rotation2d.k180deg)
			));
			
			case FIELD_RELATIVE -> this.with(pose -> new Pose2d(
				pose.getTranslation(),
				heading.plus(VirtualField.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)
			));
			
			case ROBOT_RELATIVE -> this.with(pose -> new Pose2d(
				pose.getTranslation(),
				pose.getRotation().plus(heading)
			));
			
		};
		
	}
	
	public PoseBuilder withHeading(
		CoordinateSystem coordinateSystem,
		Angle heading
	) {
		
		return this.withHeading(coordinateSystem, new Rotation2d(heading));
		
	}
	
	public PoseBuilder withRotation(Rotation2d rotation) {

		return this.with(pose -> new Pose2d(
			pose.getTranslation(),
			pose.getRotation().plus(rotation)
		));

	}
	
	public PoseBuilder withRotation(Angle angle) {

		return this.withRotation(new Rotation2d(angle));

	}
	
//	public Command go(RobotContainer robot) {
//		
//		return robot.swerve.commands.goToPosition(this, null);
//		
//	}
	
//	public Command go(RobotContainer robot, Distance distanceTolerance) {
//		
//		return robot.swerve.commands.goToPosition(
//			this, 
//			null,
//			distanceTolerance,
//			Degrees.of(1)
//		);
//		
//	}
	
//	public Command isInPosition(
//		RobotContainer robot,
//		Distance distanceTolerance,
//		Angle angularTolerance
//	) {
//		
//		return robot.swerve.commands.waitUntilAtPosition(
//			this,
//			distanceTolerance,
//			angularTolerance
//		);
//		
//	}
	
	@Override
	public Pose2d get() {

		return this.poseSupplier.get();

	}
	
	public enum CoordinateSystem {

		BLUE_OUT,

		RED_OUT,

		FIELD_RELATIVE,

		ROBOT_RELATIVE
		
	}
	
}
