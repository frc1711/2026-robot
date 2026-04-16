package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.LimelightIMUMode;
import frc.robot.devicewrappers.LimelightHelpers;
import frc.robot.devicewrappers.RaptorsLimelight;
import frc.robot.util.VirtualField;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Vision extends SubsystemBase {
	
	protected static final RaptorsLimelight[] LIMELIGHTS = new RaptorsLimelight[] {
		RaptorsLimelight.FRONT_LIMELIGHT,
		RaptorsLimelight.REAR_LIMELIGHT,
	};
	
	protected static final LinearVelocity MAX_ALLOWABLE_LINEAR_VELOCITY_FOR_VISION_UPDATES = FeetPerSecond.of(10);
	
	protected static final AngularVelocity MAX_ALLOWABLE_ANGULAR_VELOCITY_FOR_VISION_UPDATES = RotationsPerSecond.of(2);
	
	protected static final double MAX_SINGLE_TAG_AMBIGUITY_FOR_USING_VISION_UPDATES = 0.7;
	
	protected static final Distance MAX_SINGLE_TAG_DISTANCE_FOR_USING_VISION_UPDATES = Feet.of(10);
	
	protected static final Distance MAX_ALLOWABLE_VISION_TRANSLATION_ERROR = Feet.of(3);
	
	protected static final Angle MAX_ALLOWABLE_VISION_ROTATION_ERROR = Degrees.of(20);
	
	protected final Supplier<Pose2d> currentPoseSupplier;
	
	protected final Supplier<Angle> headingSupplier;
	
	protected final Supplier<LinearVelocity> linearVelocitySupplier;
	
	protected final Supplier<AngularVelocity> angularVelocitySupplier;
	
	public final Commands commands;
	
	public Vision(
		Supplier<Pose2d> currentPoseSupplier,
		Supplier<Angle> headingSupplier,
		Supplier<LinearVelocity> linearVelocitySupplier,
		Supplier<AngularVelocity> angularVelocitySupplier
	) {
		
		this.currentPoseSupplier = currentPoseSupplier;
		this.headingSupplier = headingSupplier;
		this.linearVelocitySupplier = linearVelocitySupplier;
		this.angularVelocitySupplier = angularVelocitySupplier;
		this.commands = new Commands();
	
	}
	
	public void initializeCameras() {
		
		for (RaptorsLimelight limelight: Vision.LIMELIGHTS) {
			
			limelight.initialize();
			
		}
		
	}
	
	public void beginStableSeeding() {
		
		System.out.println("Beginning stable seeding mode...");
		
		for (RaptorsLimelight limelight: Vision.LIMELIGHTS) {
			
			limelight.setIMUMode(LimelightIMUMode.EXTERNAL_SEED);
			
		}
		
	}
	
	public void beginUsingInternalLL4IMUAssist() {
		
		System.out.println("Beginning to use internal LL4 IMU assist...");
		
		for (RaptorsLimelight limelight: Vision.LIMELIGHTS) {
			
			limelight.setIMUMode(LimelightIMUMode.INTERNAL_EXTERNAL_ASSIST);
			
		}
		
	}
	
	public boolean shouldMeasurementBeUsed(
		LimelightHelpers.PoseEstimate estimate,
		Pose2d currentPose,
		LinearVelocity currentLinearVelocity,
		AngularVelocity currentAngularVelocity
	) {
		
		if (
			estimate == null ||
			estimate.pose == null ||
			estimate.tagCount <= 0 ||
			estimate.rawFiducials == null ||
			estimate.rawFiducials.length <= 0 ||
			Double.isNaN(estimate.timestampSeconds) ||
			estimate.timestampSeconds <= 0
		) return false;
		
		if (
			Math.abs(currentLinearVelocity.in(MetersPerSecond)) >
			Vision.MAX_ALLOWABLE_LINEAR_VELOCITY_FOR_VISION_UPDATES.in(MetersPerSecond)
		) return false;
		
		if (
			Math.abs(currentAngularVelocity.in(DegreesPerSecond)) >
			Vision.MAX_ALLOWABLE_ANGULAR_VELOCITY_FOR_VISION_UPDATES.in(DegreesPerSecond)
		) return false;
		
		if (estimate.tagCount == 1) {
			
			if (
				estimate.rawFiducials[0].ambiguity >
				Vision.MAX_SINGLE_TAG_AMBIGUITY_FOR_USING_VISION_UPDATES
			) return false;
			
			if (
				estimate.rawFiducials[0].distToCamera >
				Vision.MAX_SINGLE_TAG_DISTANCE_FOR_USING_VISION_UPDATES.in(Meters)
			) return false;
			
		}
		
		if (
			currentLinearVelocity.lt(InchesPerSecond.of(0.1)) &&
			currentAngularVelocity.lt(DegreesPerSecond.of(1)) &&
			estimate.tagCount >= 2
		) return true;
		
		Transform2d delta = estimate.pose.minus(currentPose);
		Distance translationError = Meters.of(delta.getTranslation().getNorm());
		Angle rotationError = Degrees.of(Math.abs(delta.getRotation().getDegrees()));
		
		return (
			translationError.lt(Vision.MAX_ALLOWABLE_VISION_TRANSLATION_ERROR) &&
			rotationError.lt(Vision.MAX_ALLOWABLE_VISION_ROTATION_ERROR)
		);
		
	}
	
	public Matrix<N3, N1> calculateVisionMeasurementStdDevs(
		LimelightHelpers.PoseEstimate estimate,
		LinearVelocity currentLinearVelocity,
		AngularVelocity currentAngularVelocity
	) {
		
		boolean isSingleTagMeasurement = estimate.tagCount == 1;
		double xyStdDevMeters = isSingleTagMeasurement
			? Inches.of(18).in(Meters)
			: Inches.of(6).in(Meters);
		
		// Distance penalties
		xyStdDevMeters += 0.12 * Math.max(0, estimate.avgTagDist - 1);
		
		// Motion penalties
		xyStdDevMeters += 0.06 * Math.abs(currentLinearVelocity.in(MetersPerSecond));
		xyStdDevMeters += 0.03 * Math.abs(currentAngularVelocity.in(RadiansPerSecond));
		
		// Favor larger tag spans when possible
		if (estimate.tagSpan > 0) {
			xyStdDevMeters *= 1 / MathUtil.clamp(estimate.tagSpan / 1.5, 0.75, 1.25);
		}
		
		// Single tag penalties
		if (isSingleTagMeasurement && estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
			xyStdDevMeters += 0.25 * estimate.rawFiducials[0].ambiguity;
		}
		
		// Clamp the standard deviation to a reasonable range to prevent
		// outliers from completely throwing off the pose estimator.
		xyStdDevMeters = MathUtil.clamp(xyStdDevMeters, 0.08, 2);
		
		// The vision-based heading is very noisy, so we ignore it in favor of
		// the IMU heading.
		double thetaStdDevRadians = 1_000_000;
		
		return VecBuilder.fill(
			xyStdDevMeters,
			xyStdDevMeters,
			thetaStdDevRadians
		);
		
	}
	
	public Optional<VisionMeasurement> buildMeasurement(
		LimelightHelpers.PoseEstimate estimate,
		Pose2d currentPose,
		LinearVelocity currentLinearVelocity,
		AngularVelocity currentAngularVelocity
	) {
		
		if (!this.shouldMeasurementBeUsed(
			estimate,
			currentPose,
			currentLinearVelocity,
			currentAngularVelocity
		)) return Optional.empty();
		
		Matrix<N3, N1> stdDevs = this.calculateVisionMeasurementStdDevs(
			estimate,
			currentLinearVelocity,
			currentAngularVelocity
		);
		
		return Optional.of(new VisionMeasurement(
			estimate.pose,
			estimate.timestampSeconds,
			stdDevs
		));
		
	}
	
	public List<VisionMeasurement> getMeasurements() {
		
		Pose2d currentPose = this.currentPoseSupplier.get();
		LinearVelocity linearVelocity = this.linearVelocitySupplier.get();
		AngularVelocity angularVelocity = this.angularVelocitySupplier.get();
		
		return Arrays.stream(Vision.LIMELIGHTS)
			.map(RaptorsLimelight::getBotPoseEstimate)
			.map(estimate -> this.buildMeasurement(
				estimate,
				currentPose,
				linearVelocity,
				angularVelocity
			))
			.flatMap(Optional::stream)
			.toList();
		
	}
	
	@Override
	public void periodic() {
		
		if (this.headingSupplier != null) {
			
			Angle limelightYaw = this.headingSupplier.get()
				.plus(Degrees.of(VirtualField.isRedAlliance() ? 180 : 0));
			
			for (RaptorsLimelight limelight: Vision.LIMELIGHTS) {
				
				limelight.setRobotOrientation(limelightYaw);
				
			}
			
		}
		
	}
	
	public record VisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {}
	
	public class Commands {
		
		public Command beginStableSeeding() {
			
			return Vision.this.runOnce(Vision.this::beginStableSeeding);
			
		}
		
		public Command beginUsingInternalLL4IMUAssist() {
			
			return Vision.this.runOnce(Vision.this::beginUsingInternalLL4IMUAssist);
			
		}
		
	}
	
}
