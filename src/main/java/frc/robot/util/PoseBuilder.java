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

import static edu.wpi.first.units.Units.*;
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
			.withRobotRelativeHeading(Rotation2d.k180deg)
			.withTranslation(ROBOT_RELATIVE, RobotDimensions.ROBOT_LENGTH.div(2), BACKWARDS);

	}

//	/**
//	 * Returns a pose for scoring on the reef, facing the AprilTag with the
//	 * given ID, with the robot positioned relative to the tag based on the
//	 * specified alignment.
//	 *
//	 * @param tagID The ID of the AprilTag to face.
//	 * @param alignment The reef branch to align the mailbox with, or CENTER if
//	 * the robot should be centered on the tag.
//	 * @return A RobotPoseBuilder representing a pose for scoring on the reef.
//	 */
//	public static PoseBuilder getReefScoringPose(
//		IntSupplier tagID,
//		ReefAlignment alignment
//	) {
//		
//		Angle direction = alignment.equals(ReefAlignment.LEFT) ? LEFT : RIGHT;
//		Distance sidestep = RobotDimensions.REEF_BRANCH_SEPARATION_DISTANCE.div(2)
//			.times(alignment.equals(ReefAlignment.CENTER) ? 0 : 1);
//		
//		PoseBuilder mailboxCenteredPose = PoseBuilder.getAprilTagFacingPose(tagID)
//			.withTranslation(ROBOT_RELATIVE, RobotDimensions.MAILBOX_LR_OFFSET_TO_ROBOT_CENTER, RIGHT);
//		
//		return mailboxCenteredPose
//			.withTranslation(ROBOT_RELATIVE, sidestep, direction)
//			.withTranslation(ROBOT_RELATIVE, Inches.of(0.5), BACKWARDS);
//		
//	}

	/**
	 * Returns a RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 *
	 * @param tagID The ID of the AprilTag representing the coral loading
	 * station.
	 * @return A RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 */
//	public static PoseBuilder getCoralStationLoadingPose(IntSupplier tagID) {
//		
//		return PoseBuilder.getAprilTagFacingPose(tagID)
//			.withRobotRelativeHeading(Rotation2d.k180deg);
//		
//	}

	/**
	 * Returns a RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 *
	 * @param robot The RobotContainer instance for which to return the
	 * requested pose.   
	 * @return A RobotPoseBuilder representing a pose for loading coral from the
	 * nearest coral loading station.
	 */
//	public static PoseBuilder getCoralStationLoadingPose(
//		RobotContainer robot
//	) {
//		
//		return PoseBuilder.getCoralStationLoadingPose(
//			() -> robot.odometry.getFieldThird().getCoralStationAprilTagID()
//		);
//
//	}

	/**
	 * Returns a RobotPoseBuilder representing a pose for calibrating the
	 * coral station loading pose, which is the same as the loading pose but
	 * offset by 24 inches in the positive X direction (away from the tag).
	 *
	 * @param tagID The ID of the AprilTag representing the coral loading
	 * station.
	 * @return A RobotPoseBuilder representing a pose for calibrating the coral
	 * station loading pose.
	 */
//	public static PoseBuilder getCoralStationCalibrationPose(IntSupplier tagID) {
//		
//		return PoseBuilder.getCoralStationLoadingPose(tagID)
//			.withRobotRelativeTranslation(new Translation2d(
//				Inches.of(-24),
//				Inches.of(0)
//			));
//		
//	}
	
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
	
	public PoseBuilder withRotation(Rotation2d rotation) {

		return this.with(pose -> new Pose2d(
			pose.getTranslation(),
			pose.getRotation().plus(rotation)
		));

	}
	
	public PoseBuilder withRotation(Angle angle) {

		return this.withRotation(new Rotation2d(angle));

	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * applied the given transformation to it under a 'robot relative' UCS
	 * (where positive X faces in the direction of the robot's current heading).
	 *
	 * @param translation The 'robot relative' translation to apply to the
	 * current pose.
	 * @return A new RobotPoseBuilder with the applied translation.
	 */
	public PoseBuilder withRobotRelativeTranslation(
		Translation2d translation
	) {
		
		return new PoseBuilder(() -> this.poseSupplier.get().plus(new Transform2d(
			translation,
			Rotation2d.kZero
		)));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'blue out'
	 * UCS (where positive X faces outwards from the blue alliance wall).
	 *
	 * @param heading The 'blue out' relative heading to set for the current
	 * pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withBlueOutHeading(Rotation2d heading) {
		
		return new PoseBuilder(() -> this.poseSupplier.get().plus(new Transform2d(
			new Translation2d(),
			heading
		)));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'blue out'
	 * UCS (where positive X faces outwards from the blue alliance wall).
	 *
	 * @param heading The 'blue out' relative heading to set for the current
	 * pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withBlueOutHeading(Angle heading) {
		
		return this.withBlueOutHeading(new Rotation2d(heading));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the specified
	 * alliance's wall).
	 *
	 * @param alliance The alliance to use for determining the field-relative
	 * coordinate system.
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Rotation2d heading
	) {
		
		boolean shouldInvert = alliance == DriverStation.Alliance.Red;
		
		return this.withBlueOutHeading(heading.times(shouldInvert ? -1 : 1));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param alliance The alliance to use for determining the field-relative
	 * coordinate system.
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withFieldRelativeHeading(
		DriverStation.Alliance alliance,
		Angle heading
	) {
		
		return this.withFieldRelativeHeading(alliance, new Rotation2d(heading));
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withFieldRelativeHeading(Rotation2d heading) {

		return this.withFieldRelativeHeading(
			VirtualField.getAlliance(),
			heading
		);

	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'field
	 * relative' UCS (where positive X faces outwards from the alliance wall,
	 * as informed by the FMS).
	 *
	 * @param heading The 'field relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withFieldRelativeHeading(Angle heading) {

		return this.withFieldRelativeHeading(new Rotation2d(heading));

	}
	
	public PoseBuilder withRobotRelativeHeading(Rotation2d heading) {
		
		return new PoseBuilder(() -> {
			
			Pose2d pose = this.poseSupplier.get();

			return new Pose2d(
				pose.getTranslation(),
				pose.getRotation().plus(heading)
			);
			
		});
		
	}

	/**
	 * Returns a modified version of the current RobotPoseBuilder pose, having
	 * set the heading of the pose to the specified heading under a 'robot
	 * relative' UCS (where positive X faces in the direction of the robot's
	 * current heading).
	 *
	 * @param heading The 'robot relative' heading to set for the current pose.
	 * @return A new RobotPoseBuilder with the applied heading.
	 */
	public PoseBuilder withRobotRelativeHeading(Angle heading) {
		
		return this.withRobotRelativeHeading(new Rotation2d(heading));
		
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
