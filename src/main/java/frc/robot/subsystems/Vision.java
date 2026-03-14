package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.devicewrappers.RaptorsLimelight;
import frc.robot.util.VirtualField;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Vision {
	
	protected static final RaptorsLimelight[] LIMELIGHTS = new RaptorsLimelight[] {
		RaptorsLimelight.LEFT_LIMELIGHT,
		RaptorsLimelight.RIGHT_LIMELIGHT,
	};
	
	protected final Supplier<Angle> headingSupplier;
	
	protected final Supplier<LinearVelocity> linearVelocitySupplier;
	
	protected final Supplier<AngularVelocity> angularVelocitySupplier;
	
	public Vision(
		Supplier<Angle> headingSupplier,
		Supplier<LinearVelocity> linearVelocitySupplier,
		Supplier<AngularVelocity> angularVelocitySupplier
	) {
		
		this.headingSupplier = headingSupplier;
		this.linearVelocitySupplier = linearVelocitySupplier;
		this.angularVelocitySupplier = angularVelocitySupplier;
	
	}
	
	public List<VisionMeasurement> getMeasurements() {
		
		boolean shouldUpdateVision = (
			this.angularVelocitySupplier.get().lt(DegreesPerSecond.of(80)) &&
			this.linearVelocitySupplier.get().lt(FeetPerSecond.of(1.5))
		);
		
		if (!shouldUpdateVision) return List.of();
		
		Angle limelightYaw = this.headingSupplier.get()
			.plus(Degrees.of(VirtualField.isRedAlliance() ? 180 : 0));
		
		for (RaptorsLimelight limelight: Vision.LIMELIGHTS) {
			
			limelight.setRobotOrientation(limelightYaw);
			
		}
		
		double maxTagDistInMeters = Feet.of(10).in(Meters);
		double maxTagAmbiguity = 0.6;
		LinearVelocity linearVelocity = this.linearVelocitySupplier.get();
		AngularVelocity angularVelocity = this.angularVelocitySupplier.get();
		
		return Arrays.stream(Vision.LIMELIGHTS)
			.map(RaptorsLimelight::getBotPoseEstimate)
			.filter(estimate -> (
				estimate != null &&
				estimate.pose != null &&
				estimate.tagCount > 0 &&
				estimate.avgTagDist < maxTagDistInMeters &&
				estimate.rawFiducials[0].ambiguity < maxTagAmbiguity
			))
			.map(estimate -> {
				
				double ambiguityScaling = estimate.rawFiducials[0].ambiguity + 1;
				Time kLatency = Milliseconds.of(estimate.latency).times(3);
				Distance baselineLinearDeviation = Inches.of(4).times(ambiguityScaling);
				Distance possibleRobotMovement = linearVelocity.times(kLatency);
				Distance linearDeviation = baselineLinearDeviation.plus(possibleRobotMovement);
				
				double linearDeviationMeters = linearDeviation.in(Meters);
				Angle baselineAngularDeviation = Degrees.of(30);
				Angle possibleRobotRotation = angularVelocity.times(kLatency);
				Angle angularDeviation = baselineAngularDeviation.plus(possibleRobotRotation);
				
				return new VisionMeasurement(
					estimate.pose,
					estimate.timestampSeconds,
					VecBuilder.fill(
						linearDeviationMeters,
						linearDeviationMeters,
						angularDeviation.in(Degrees)
					)
				);
				
			})
			.toList();
		
	}
	
	public record VisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {}
	
}
