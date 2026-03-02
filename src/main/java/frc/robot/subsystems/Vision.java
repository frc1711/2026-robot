package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    private int[] aprilTagFilter = null;

    private int[] hubAprilTagFilter = {2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27};

    public Vision() {}

    private final String[] limelights = {
        "limelight-shooter" // IP: 10.17.11.11
    };

    public Pose2d estimatePose() {
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[0]);

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            return llMeasurement.pose;
        }

        return null;
    }

    public void applyAprilTagFilter(int[] filter) {
        this.aprilTagFilter = filter;

        for (String ll : limelights) {
            LimelightHelpers.SetFiducialIDFiltersOverride(ll, this.aprilTagFilter);
        }
    }

    public void resetAprilTagFilter() {
        this.aprilTagFilter = null;

        for (String ll : limelights) {
            LimelightHelpers.SetFiducialIDFiltersOverride(ll, this.aprilTagFilter);
        }
    }

    public double getDistance() {
        double distance = 0;
        
        applyAprilTagFilter(hubAprilTagFilter);
        String ll = limelights[0];

        if (LimelightHelpers.getTV(ll)) {
            double targetOffsetAngle = LimelightHelpers.getTY(ll);

            Pose3d llPose = LimelightHelpers.getCameraPose3d_RobotSpace(ll);

            double llAngleOffsetRadians = llPose.getRotation().getY();
            double llLensHeightMeters = llPose.getZ();

            double targetHeightMeters = Units.inchesToMeters(44.25);

            double angleToTargetRadians = llAngleOffsetRadians + Units.degreesToRadians(targetOffsetAngle);
            double llToTargetMeters = (targetHeightMeters - llLensHeightMeters) / Math.tan(angleToTargetRadians);

            double robotCenterToTargetMeters = llToTargetMeters - Math.hypot(llPose.getX(), llPose.getY());

            distance += robotCenterToTargetMeters;
        }

        resetAprilTagFilter();

        return distance / limelights.length;
    }

    public List<VisionMeasurement> getVisionMeasurements() {
        List<VisionMeasurement> measurements = new ArrayList<>();

        for (String ll : limelights) {
            var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

            if (estimate != null && estimate.tagCount >= 2 && ll != limelights[0]) {
                measurements.add(
                    new VisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds
                    )
                );
            }
            //System.out.println(ll + " getting Measurements");
        }

        return measurements;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Distance to Hub", 
            this::getDistance,
            null
        );
    }

    public record VisionMeasurement(Pose2d pose, double timestampSeconds) {}
}