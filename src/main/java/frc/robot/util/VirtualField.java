package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.configuration.FieldThird;
import frc.robot.math.Point;

import java.util.List;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;

public class VirtualField {
	
	public static final Distance FIELD_LENGTH = Meters.of(17.5482504);
	
	public static final Distance FIELD_WIDTH = Meters.of(8.0519016);
	
	public static final Distance HUB_WIDTH = Inches.of(47);
	
	public static final AprilTagFieldLayout APRIL_TAGS =
		AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	
	public static final DriverStation.Alliance DEFAULT_ALLIANCE =
		DriverStation.Alliance.Red;
	
	public static DriverStation.Alliance getAlliance() {
		
		return DriverStation.getAlliance()
			.orElse(VirtualField.DEFAULT_ALLIANCE);
		
	}

	public static boolean isRedAlliance() {

		return DriverStation.Alliance.Red.equals(VirtualField.getAlliance());

	}

	public static boolean isBlueAlliance() {

		return DriverStation.Alliance.Blue.equals(VirtualField.getAlliance());

	}
	
	public static AprilTag getAprilTagByID(int id) {
		
		return VirtualField.APRIL_TAGS.getTags().get(id - 1);
		
	}
	
	public static Stream<AprilTag> getHubAprilTags(DriverStation.Alliance alliance) {

		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(18, 19, 20, 21, 24, 25, 26, 27)
			: Stream.of( 2,  3,  4,  5,  8,  9, 10, 11)
		).map(VirtualField::getAprilTagByID);

	}

	public static Stream<AprilTag> getHubAprilTags() {

		return VirtualField.getHubAprilTags(VirtualField.getAlliance());

	}

	public static Stream<AprilTag> getOutpostAprilTags(DriverStation.Alliance alliance) {

		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(29, 30)
			: Stream.of(13, 14)
		).map(VirtualField::getAprilTagByID);

	}

	public static Stream<AprilTag> getOutpostAprilTags() {

		return VirtualField.getOutpostAprilTags(VirtualField.getAlliance());

	}
	
	public static Stream<AprilTag> getTowerAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(31, 32)
			: Stream.of(15, 16)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getTowerAprilTags() {
		
		return VirtualField.getTowerAprilTags(VirtualField.getAlliance());
		
	}
	
	public static Stream<AprilTag> getLeftTrenchAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(22, 23)
			: Stream.of( 6,  7)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getLeftTrenchAprilTags() {
		
		return VirtualField.getLeftTrenchAprilTags(VirtualField.getAlliance());
		
	}
	
	public static Stream<AprilTag> getRightTrenchAprilTags(DriverStation.Alliance alliance) {
		
		return (alliance == DriverStation.Alliance.Blue
			? Stream.of(17, 28)
			: Stream.of( 1, 12)
		).map(VirtualField::getAprilTagByID);
		
	}
	
	public static Stream<AprilTag> getRightTrenchAprilTags() {
		
		return VirtualField.getRightTrenchAprilTags(VirtualField.getAlliance());
		
	}

	public static Point getHubCenterPoint(DriverStation.Alliance alliance) {

		return Point.average(
			VirtualField.getHubAprilTags(alliance)
				.map(tag -> tag.pose.getTranslation().toTranslation2d())
				.toArray(Translation2d[]::new)
		);

	}

	public static Translation2d getHubCenterPoint() {

		return VirtualField.getHubCenterPoint(VirtualField.getAlliance());

	}
	
	public static FieldThird getFieldThirdForPosition(
		DriverStation.Alliance alliance,
		Translation2d position
	) {
		
		Distance leftSideCutoff = FIELD_WIDTH.minus(HUB_WIDTH).div(2);
		Distance rightSideCutoff = FIELD_WIDTH.minus(leftSideCutoff);
		Distance robotYPosition = position.getMeasureY();
		
		if (robotYPosition.lt(leftSideCutoff)) {
			
			return alliance == DriverStation.Alliance.Red
				? FieldThird.LEFT
				: FieldThird.RIGHT;
			
		} else if (robotYPosition.gt(rightSideCutoff)) {
			
			return alliance == DriverStation.Alliance.Red
				? FieldThird.RIGHT
				: FieldThird.LEFT;
			
		} else return FieldThird.CENTER;
		
	}
	
	public static FieldThird getFieldThirdForPosition(Translation2d position) {
		
		return VirtualField.getFieldThirdForPosition(
			VirtualField.getAlliance(),
			position
		);
		
	}
	
}
