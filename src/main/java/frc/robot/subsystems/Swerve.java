package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.devicewrappers.RaptorsPigeon2;
import frc.robot.util.LogCommand;

import java.util.function.Supplier;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class Swerve extends SubsystemBase {
    
    public static final LinearAcceleration MAX_LINEAR_ACCELERATION = FeetPerSecondPerSecond.of(12);
    
    public static final LinearAcceleration MAX_LINEAR_DECELERATION = FeetPerSecondPerSecond.of(-16);
    
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1);
    
    public static final AngularAcceleration MAX_ANGULAR_DECELERATION = RotationsPerSecondPerSecond.of(-2);
    
    public static final LinearVelocity MAX_LINEAR_VELOCITY = InchesPerSecond.of(100);
    
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.5);
    
    public static final LinearVelocity SLOW_MODE_MAX_LINEAR_VELOCITY = InchesPerSecond.of(30);
    
    public static final AngularVelocity SLOW_MODE_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(60);
    
    protected final SwerveModule[] modules;
    
    protected final PIDController headingPIDController;

    protected final RaptorsPigeon2 gyro;
    
    protected final SwerveDriveKinematics kinematics;
    
    protected double speedMultiplier;
    
    protected ChassisSpeeds chassisSpeeds;
    
    public Supplier<Angle> headingLockSupplier;
    
    public final Commands commands;

    public Swerve() {
        
        this.modules = SwerveModuleConfiguration.getModuleConfigurations()
            .map(SwerveModule::new)
            .toArray(SwerveModule[]::new);
        this.headingPIDController = new PIDController(5, 0, 0);
        this.gyro = new RaptorsPigeon2(CANDevice.PIGEON_IMU);
        this.kinematics = new SwerveDriveKinematics(
            SwerveModuleConfiguration.getModuleConfigurations()
                .map(config -> config.positionInRobot)
                .toArray(Translation2d[]::new)
        );
        this.speedMultiplier = 1;
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        this.headingLockSupplier = null;
        this.commands = new Commands();
        
//        this.resetGyro();

        headingPIDController.enableContinuousInput(0, 360);
        
        ShuffleboardTab shuffleboardCalibrationTab =
            Shuffleboard.getTab("Calibration");
        
        shuffleboardCalibrationTab.add(
            this.commands.calibrateModuleSteeringHeadings()
        );
        
        shuffleboardCalibrationTab.add(
            this.commands.calibrateFieldRelativeHeading()
        );
        
        SmartDashboard.putData("Swerve Drive", this.getSwerveStateSendable());
        
    }
    
    public Stream<SwerveModule> getModuleStream() {
        
        return Stream.of(this.modules);
        
    }
    
    public SwerveModulePosition[] getModulePositions() {
        
        return this.getModuleStream()
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
        
    }
    
    public SwerveDriveKinematics getKinematics() {
        
        return this.kinematics;
        
    }
    
    public ChassisSpeeds getActualChassisSpeeds() {
        
        return this.kinematics.toChassisSpeeds(
            this.getModuleStream()
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new)
        );
        
    }
    
    public void setDriveSpeedMultiplier(double speedMultiplier) {
        
        this.speedMultiplier = MathUtil.clamp(speedMultiplier, 0, 1);
        
    }
    
    public void setDriveMotorIdleState(NeutralModeValue neutralMode) {
        
        this.getModuleStream().forEach(module -> {
            
            TalonFXConfiguration config = module.getDriveMotorConfig();
            
            config.MotorOutput.NeutralMode = neutralMode;
            
            module.driveMotor.getConfigurator().apply(config);
            
        });
        
    }
    
    protected void applyModuleStates(SwerveModuleState[] moduleStates) {
        
        this.getModuleStream().forEach(module -> {
            module.updateModuleState(moduleStates[module.getID()]);
        });
        
    }
    
    public void stop() {
        
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        
    }
    
    public void calibrateFieldRelativeHeading() {
        
        this.calibrateFieldRelativeHeading(Degrees.of(0));
        
    }
    
    public void calibrateFieldRelativeHeading(Angle currentHeading) {
        
        this.gyro.yaw.calibrate(currentHeading);
        this.setFieldRelativeHeadingSetpoint(currentHeading.times(-1));
        
    }
    
    public Angle getFieldRelativeHeading() {
        
        return this.gyro.yaw.getAngle();
        
    }
    
    public void setFieldRelativeHeadingSetpoint(Angle heading) {

//        Pose2d existingPose = this.odometry.getPose();

        this.headingPIDController.setSetpoint(heading.in(Degrees));

//        this.odometry.resetPose(existingPose);

    }
    
    public LinearVelocity getLinearVelocity() {
        
        ChassisSpeeds actualChassisSpeeds = this.getActualChassisSpeeds();
        Translation2d speedsTranslation = new Translation2d(
            actualChassisSpeeds.vxMetersPerSecond,
            actualChassisSpeeds.vyMetersPerSecond
        );
        
        return MetersPerSecond.of(speedsTranslation.getNorm());
        
    }
    
    public AngularVelocity getAngularVelocity() {
        
        return this.gyro.getYawAngularVelocity();
        
    }
    
    @Override
    public void periodic() {
        
        SwerveModuleState[] newModuleStates =
            this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
        
        this.applyModuleStates(newModuleStates);
        
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.addDoubleProperty(
            "Heading",
            () -> this.getFieldRelativeHeading().in(Degrees),
            (double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
        );
        
        builder.addDoubleProperty(
            "Heading Setpoint",
            this.headingPIDController::getSetpoint,
            (double headingDegrees) -> this.setFieldRelativeHeadingSetpoint(Degrees.of(headingDegrees))
        );
        
        builder.addDoubleProperty(
            "Swerve Linear Velocity (in/sec)",
            () -> this.getLinearVelocity().in(InchesPerSecond),
            null
        );
        
        builder.addDoubleProperty(
            "Swerve Module Velocity kP",
            () -> {
                
                TalonFXConfiguration config = new TalonFXConfiguration();
                this.modules[0].driveMotor.getConfigurator().refresh(config);
                
                return config.Slot0.kP;
                
            },
            (double kP) -> {
                
                TalonFXConfiguration config = new TalonFXConfiguration();
                this.modules[0].driveMotor.getConfigurator().refresh(config);
                
                config.Slot0.kP = kP;
                
                this.getModuleStream().forEach(module ->
                    module.driveMotor.getConfigurator().apply(config)
                );
                
            }
        );
        
        builder.addDoubleProperty(
            "Swerve Module Velocity kD",
            () -> {
                
                TalonFXConfiguration config = new TalonFXConfiguration();
                this.modules[0].driveMotor.getConfigurator().refresh(config);
                
                return config.Slot0.kD;
                
            },
            (double kD) -> {
                
                TalonFXConfiguration config = new TalonFXConfiguration();
                this.modules[0].driveMotor.getConfigurator().refresh(config);
                
                config.Slot0.kD = kD;
                
                this.getModuleStream().forEach(module ->
                    module.driveMotor.getConfigurator().apply(config)
                );
                
            }
        );
        
//        builder.addDoubleProperty(
//            "Distance to Scoring Pose (in)",
//            () -> {
//                
//                Pose2d currentPose = this.odometry.getPose();
//                IntSupplier tagID = () -> this.odometry.getNearestReefAprilTag().ID;
//                Pose2d leftScoringPose = PoseBuilder.getReefScoringPose(tagID, ReefAlignment.LEFT).get();
//                Pose2d rightScoringPose = PoseBuilder.getReefScoringPose(tagID, ReefAlignment.RIGHT).get();
//                double distanceToLeft = currentPose.minus(leftScoringPose).getTranslation().getNorm();
//                double distanceToRight = currentPose.minus(rightScoringPose).getTranslation().getNorm();
//                
//                return distanceToLeft < distanceToRight
//                    ? Meters.of(distanceToLeft).in(Inches)
//                    : Meters.of(distanceToRight).in(Inches);
//                
//            },
//            null
//        );
        
        builder.addDoubleProperty(
            "Average Module Output Current",
            () -> this.getModuleStream().mapToDouble((module) ->
                module.driveMotor.getSupplyCurrent().getValueAsDouble()
            ).average().orElse(0),
            null
        );
        
    }
    
    public Sendable getSwerveStateSendable() {
        
        return builder -> {
            
            builder.setSmartDashboardType("SwerveDrive");
            
            this.modules[0].addSendableFields(builder, "Front Left");
            this.modules[1].addSendableFields(builder, "Front Right");
            this.modules[2].addSendableFields(builder, "Rear Left");
            this.modules[3].addSendableFields(builder, "Rear Right");
            
            builder.addDoubleProperty(
                "Robot Angle",
                () -> this.getFieldRelativeHeading().in(Degrees),
                null
            );
            
            builder.addDoubleProperty(
                "Chassis Speeds (vX in inches per second)",
                () -> MetersPerSecond.of(this.getActualChassisSpeeds().vxMetersPerSecond).in(InchesPerSecond),
                null
            );
            
            builder.addDoubleProperty(
                "Chassis Speeds (vY in inches per second)",
                () -> MetersPerSecond.of(this.getActualChassisSpeeds().vyMetersPerSecond).in(InchesPerSecond),
                null
            );
            
            builder.addDoubleProperty(
                "Chassis Speeds (vXY in inches per second)",
                () -> this.getLinearVelocity().in(InchesPerSecond),
                null
            );
            
            builder.addDoubleProperty(
                "Chassis Speeds (Rotation in degrees per second)",
                () -> RadiansPerSecond.of(this.getActualChassisSpeeds().omegaRadiansPerSecond).in(DegreesPerSecond),
                null
            );
            
//            builder.addStringProperty(
//                "Field Position",
//                () -> this.odometry.getFieldThird().name(),
//                null
//            );
            
        };
        
    }
    
    public class Commands {
        
        public Command useDriveSpeedMultiplier(double multiplier) {
            
            return edu.wpi.first.wpilibj2.command.Commands
                .runOnce(() -> Swerve.this.setDriveSpeedMultiplier(Math.min(multiplier, 1)))
                .finallyDo(() -> Swerve.this.setDriveSpeedMultiplier(1));
            
        }
        
        public Command calibrateModuleSteeringHeadings() {
            
            return Swerve.this
                .runOnce(() -> Swerve.this.getModuleStream().forEach(SwerveModule::calibrateSteeringHeading))
                .andThen(new LogCommand("Swerve module steering headings calibrated."))
                .withName("Calibrate Swerve Module Steering Headings")
                .ignoringDisable(true);
            
        }
        
        public Command calibrateFieldRelativeHeading(Angle currentHeading) {
            
            return Swerve.this
                .runOnce(() -> Swerve.this.calibrateFieldRelativeHeading(currentHeading))
                .andThen(new LogCommand("Swerve field-relative heading calibrated."))
                .withName("Calibrate Swerve Field-relative Heading")
                .ignoringDisable(true);
            
        }
        
        public Command calibrateFieldRelativeHeading() {
            
            return this.calibrateFieldRelativeHeading(Degrees.of(0));
            
        }
        
//        public Command setFieldRelativeHeading(Angle heading) {
//            
//            return Swerve.this.runOnce(
//                () -> Swerve.this.setFieldRelativeHeadingSetpoint(heading)
//            );
//            
//        }
//        
//        public Command disableHeadingLock() {
//            
//            return new InstantCommand(
//                () -> Swerve.this.headingLockSupplier = null
//            );
//            
//        }
//        
//        public Command enableStaticHeadingLock(Angle heading) {
//            
//            return new InstantCommand(
//                () -> Swerve.this.headingLockSupplier = () -> heading
//            );
//            
//        }
//        
//        public Command enabledPOIHeadingLock(Translation2d pointOfInterest) {
//            
//            return new InstantCommand(() -> {
//                
//                Swerve.this.headingLockSupplier = () -> {
//                    
//                    Pose2d currentPose = Swerve.this.odometry.getPose();
//                    
//                    if (currentPose == null) return Degrees.zero();
//                    
//                    Translation2d deltaTranslation = pointOfInterest
//                        .minus(currentPose.getTranslation());
//                    
//                    return deltaTranslation.getAngle().getMeasure();
//                    
//                };
//                
//            });
//            
//        }
        
        public Command xMode(LinearVelocity outwardDriveSpeed) {
            
            return Swerve.this.startEnd(
                () -> Swerve.this.getModuleStream().forEach(module ->
                    module.updateModuleState(new SwerveModuleState(
                        outwardDriveSpeed,
                        module.configuration.positionInRobot.getAngle()
                    ))
                ),
                Swerve.this::stop
            );
            
        }
        
        public Command xMode() {
            
            return this.xMode(InchesPerSecond.of(0));
            
        }
        
        public Command stop() {
            
            return new InstantCommand(Swerve.this::stop, Swerve.this);
            
        }
        
        public Command drive(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
            
            return Swerve.this.run(() -> Swerve.this.chassisSpeeds = chassisSpeedsSupplier.get());
            
        }
        
//        public Command goToPosition(
//            Supplier<Pose2d> poseSupplier,
//            Supplier<int[]> aprilTagFilter,
//            Distance distanceTolerance,
//            Angle angularTolerance
//        ) {
//            
//            AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(90);
//            
//            LinearMotionProfiler trajectory = new LinearMotionProfiler(
//                /* Max Linear Velocity: */ InchesPerSecond.of(200),
//                /* Max Linear Acceleration: */ FeetPerSecondPerSecond.of(80),
//                /* Max Linear Deceleration: */ FeetPerSecondPerSecond.of(6)
//            );
//            PIDController thetaController = new PIDController(8, 0, 0);
//            thetaController.enableContinuousInput(-180, 180);
//            
//            Command command = new Command() {
//                
//                Pose2d desiredPose;
//                Pose2d currentPose;
//                
//                @Override
//                public void initialize() {
//                    
//                    if (aprilTagFilter != null) {
//                        Swerve.this.odometry.vision.setAprilTagFilter(aprilTagFilter.get());
//                    }
//                    
//                }
//                
//                Distance getRemainingLinearDistance() {
//                    
//                    return Meters.of(
//                        currentPose.getTranslation()
//                            .minus(desiredPose.getTranslation())
//                            .getNorm()
//                    );
//                    
//                }
//                
//                @Override
//                public void execute() {
//                    
//                    Pose2d newCurrentPose = Swerve.this.odometry.getPose();
//                    Pose2d newDesiredPose = poseSupplier.get();
//                    
//                    if (newCurrentPose != null) this.currentPose = newCurrentPose;
//                    if (newDesiredPose != null) this.desiredPose = newDesiredPose;
//                    
//                    Swerve.this.odometry.setDisplaySetpoint(this.desiredPose);
//                    thetaController.setSetpoint(
//                        this.desiredPose.getRotation().getMeasure().in(Degrees)
//                    );
//                    
//                    LinearVelocity velocity = trajectory.calculate(
//                        this.getRemainingLinearDistance(),
//                        Swerve.this.getLinearVelocity()
//                    );
//                    
//                    Translation2d deltaTranslation = this.desiredPose.getTranslation()
//                        .minus(this.currentPose.getTranslation());
//                    
//                    Translation2d chassisSpeedTranslation = new Translation2d(
//                        velocity.in(MetersPerSecond),
//                        deltaTranslation.getAngle()
//                    ).times(VirtualField.isRedAlliance() ? -1 : 1);
//                    
//                    Swerve.this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
//                        MetersPerSecond.of(chassisSpeedTranslation.getX()),
//                        MetersPerSecond.of(chassisSpeedTranslation.getY()),
//                        DegreesPerSecond.of(MathUtil.clamp(
//                            thetaController.calculate(currentPose.getRotation().getDegrees()),
//                            -MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
//                            MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
//                        ))
//                    ), Swerve.this.gyro.yaw.getRotation());
//                    
//                }
//                
//                @Override
//                public boolean isFinished() {
//                    
//                    Pose2d relativePose = currentPose.relativeTo(desiredPose);
//                    Distance linearDistance = Meters.of(relativePose.getTranslation().getNorm());
//                    
//                    if (linearDistance.lt(Inches.of(0))) {
//                        
//                        linearDistance = linearDistance.times(-1);
//                        
//                    }
//                    
//                    System.out.println(
//                        "Distance remaining to target pose: " +
//                            linearDistance.in(Inches) + " inches"
//                    );
//                    
//                    return (
//                        linearDistance.lte(distanceTolerance) &&
//                            currentPose.getRotation().getMeasure().isNear(
//                                desiredPose.getRotation().getMeasure(),
//                                angularTolerance
//                            )
//                    );
//                    
//                }
//                
//                @Override
//                public void end(boolean interrupted) {
//                    
//                    Swerve.this.odometry.removeDisplaySetpoint();
//                    Swerve.this.odometry.vision.resetAprilTagFilter();
//                    Swerve.this.stop();
//                    
//                }
//                
//            };
//            
//            command.addRequirements(Swerve.this);
//            
//            return command;
//            
//        }
//        
//        public Command goToPosition(
//            Supplier<Pose2d> poseSupplier,
//            Supplier<int[]> aprilTagFilter
//        ) {
//            
//            return this.goToPosition(
//                poseSupplier,
//                aprilTagFilter,
//                Inches.of(0.25),
//                Degrees.of(1)
//            );
//            
//        }
//        
//        public Command waitUntilAtPosition(
//            Supplier<Pose2d> desiredPoseSupplier,
//            Distance distanceTolerance,
//            Angle angularTolerance
//        ) {
//            
//            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(() -> {
//                
//                Pose2d desiredPose = desiredPoseSupplier.get();
//                Pose2d currentPose = Swerve.this.odometry.getPose();
//                
//                return (
//                    currentPose.getMeasureX().isNear(desiredPose.getMeasureX(), distanceTolerance) &&
//                        currentPose.getMeasureY().isNear(desiredPose.getMeasureY(), distanceTolerance) &&
//                        currentPose.getRotation().getMeasure().isNear(desiredPose.getRotation().getMeasure(), angularTolerance)
//                );
//                
//            });
//            
//        }
        
    }
    
}
