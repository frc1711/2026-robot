package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.configuration.CANDevice;
import frc.robot.configuration.Direction;
import frc.robot.configuration.SwerveModuleConfiguration;
import frc.robot.devicewrappers.RaptorsPigeon2;
import frc.robot.math.DoubleUtilities;
import frc.robot.math.LinearMotionProfiler;
import frc.robot.math.Point;
import frc.robot.math.Vector;
import frc.robot.util.HeadingLock;
import frc.robot.util.LogCommand;
import frc.robot.util.RadiusLock;
import frc.robot.util.VirtualField;

import java.util.List;
import java.util.Set;
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

    protected final RaptorsPigeon2 gyro;
    
    protected final SwerveDriveKinematics kinematics;
    
    protected final RaptorsOdometry odometry;
    
    protected double driveSpeedMultiplier;
    
    protected ChassisSpeeds chassisSpeeds;
    
    public final HeadingLock headingLock;
    
    public final RadiusLock radiusLock;
    
    public final Commands commands;

    public Swerve(RaptorsOdometry odometry) {
        
        this.modules = SwerveModuleConfiguration.getModuleConfigurations()
            .map(SwerveModule::new)
            .toArray(SwerveModule[]::new);
        this.gyro = new RaptorsPigeon2(CANDevice.PIGEON_IMU);
        this.kinematics = new SwerveDriveKinematics(
            SwerveModuleConfiguration.getModuleConfigurations()
                .map(config -> config.positionInRobot)
                .toArray(Translation2d[]::new)
        );
        this.odometry = odometry;
        this.driveSpeedMultiplier = 1;
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        this.headingLock = new HeadingLock(this);
        this.radiusLock = new RadiusLock(this);
        this.commands = new Commands();
        
//        this.resetGyro();
        
        ShuffleboardTab shuffleboardCalibrationTab =
            Shuffleboard.getTab("Calibration");
        
        shuffleboardCalibrationTab.add(
            this.commands.calibrateModuleSteeringHeadings()
        );
        
        shuffleboardCalibrationTab.add(
            this.commands.calibrateFieldRelativeHeading()
        );
        
        SmartDashboard.putData("Swerve", this);
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
    
    public RaptorsOdometry getOdometry() {
        
        return this.odometry;
        
    }
    
    public ChassisSpeeds getActualChassisSpeeds() {
        
        return this.kinematics.toChassisSpeeds(
            this.getModuleStream()
                .map(SwerveModule::getState)
                .toArray(SwerveModuleState[]::new)
        );
        
    }
    
    public void setDriveSpeedMultiplier(double speedMultiplier) {
        
        this.driveSpeedMultiplier = MathUtil.clamp(speedMultiplier, 0, 1);
        
    }
    
    public double getDriveSpeedMultiplier() {
        
        return this.driveSpeedMultiplier;
        
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
    
    protected void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        
        this.chassisSpeeds = chassisSpeeds;
        
        SwerveModuleState[] newModuleStates =
            this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
        
        this.applyModuleStates(newModuleStates);
        
    }
    
    public void stop() {
        
        this.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        
    }
    
    public void calibrateFieldRelativeHeading() {
        
        this.calibrateFieldRelativeHeading(Degrees.of(0));
        
    }
    
    public void calibrateFieldRelativeHeading(Angle currentHeading) {
        
        Pose2d existingPose = this.odometry.getPose();
        
        this.gyro.yaw.calibrate(currentHeading);
        
        this.odometry.resetPose(new Pose2d(
            existingPose.getTranslation(),
            new Rotation2d(currentHeading)
                .plus(VirtualField.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)
        ));
        
    }
    
    public Angle getFieldRelativeHeading() {
        
        return Degrees.of(DoubleUtilities.normalizeToRange(
            this.gyro.yaw.getAngle().in(Degrees),
            -180,
            180
        ));
        
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
        
        this.headingLock.periodic();
        this.radiusLock.periodic();
        
    }
    
    public SysIdRoutine getDriveMotorsSysIdRoutine() {
        
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> this.getModuleStream().forEach((module) -> {
                    module.steerMotor.setControl(new MotionMagicVoltage(0));
                    module.driveMotor.setControl(new VoltageOut(volts.in(Volts)));
                }),
                null,
                this
            )
        );
        
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.addDoubleProperty(
            "Heading",
            () -> this.getFieldRelativeHeading().in(Degrees),
            (double headingDegrees) -> this.headingLock.enable(() -> Degrees.of(headingDegrees))
        );
        
        builder.addDoubleProperty(
            "Heading Setpoint",
            () -> this.headingLock.isEnabled() ? this.headingLock.getHeading().in(Degrees) : -1,
            (double headingDegrees) -> this.headingLock.enable(() -> Degrees.of(headingDegrees))
        );
        
        builder.addStringProperty(
            "Heading Lock",
            () -> {
                
                if (!this.headingLock.isEnabled()) return "DISABLED";
                
                String state = this.headingLock.hasLock() ? "LOCKED" : "HOMING";
                double errorDegrees = this.headingLock.getError().in(Degrees);
                
                return String.format("%s (%+.1f deg.)", state, errorDegrees);
                
            },
            null
        );
        
        builder.addStringProperty(
            "Radius Lock",
            () -> {
                
                if (!this.radiusLock.isEnabled()) return "DISABLED";
                
                String state = this.radiusLock.hasLock() ? "LOCKED" : "HOMING";
                double errorDegrees = this.radiusLock.getError().in(Inches);
                
                return String.format("%s (%+.1f in.)", state, errorDegrees);
                
            },
            null
        );
        
        builder.addDoubleProperty(
            "Distance to Hub (inches)",
            () -> VirtualField.getDistanceToHubCenterPoint(this.odometry.getTranslation()).in(Inches),
            null
//            (double radiusInches) -> this.radiusLockSupplier =
//                () -> new RadiusLock(VirtualField.getHubCenterPoint(), Inches.of(radiusInches))
        );
        
        builder.addDoubleProperty(
            "Swerve Linear Velocity (in/sec)",
            () -> this.getLinearVelocity().in(InchesPerSecond),
            null
        );
        
//        builder.addDoubleProperty(
//            "Swerve Module Velocity kP",
//            () -> {
//                
//                TalonFXConfiguration config = new TalonFXConfiguration();
//                this.modules[0].driveMotor.getConfigurator().refresh(config);
//                
//                return config.Slot0.kP;
//                
//            },
//            (double kP) -> {
//                
//                TalonFXConfiguration config = new TalonFXConfiguration();
//                this.modules[0].driveMotor.getConfigurator().refresh(config);
//                
//                config.Slot0.kP = kP;
//                
//                this.getModuleStream().forEach(module ->
//                    module.driveMotor.getConfigurator().apply(config)
//                );
//                
//            }
//        );
//        
//        builder.addDoubleProperty(
//            "Swerve Module Velocity kD",
//            () -> {
//                
//                TalonFXConfiguration config = new TalonFXConfiguration();
//                this.modules[0].driveMotor.getConfigurator().refresh(config);
//                
//                return config.Slot0.kD;
//                
//            },
//            (double kD) -> {
//                
//                TalonFXConfiguration config = new TalonFXConfiguration();
//                this.modules[0].driveMotor.getConfigurator().refresh(config);
//                
//                config.Slot0.kD = kD;
//                
//                this.getModuleStream().forEach(module ->
//                    module.driveMotor.getConfigurator().apply(config)
//                );
//                
//            }
//        );
        
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
            this.modules[2].addSendableFields(builder, "Back Left");
            this.modules[3].addSendableFields(builder, "Back Right");
            
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
            
            Runnable resetMultiplier = () -> Swerve.this.setDriveSpeedMultiplier(1);
            
            return edu.wpi.first.wpilibj2.command.Commands.startEnd(
                () -> Swerve.this.setDriveSpeedMultiplier(Math.min(multiplier, 1)),
                resetMultiplier
            ).finallyDo(resetMultiplier);
            
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

        public Command disableHeadingLock() {

            return new InstantCommand(Swerve.this.headingLock::disable);

        }
        
        public Command enableDynamicHeadingLock(
            Supplier<Angle> headingSupplier
        ) {
            
            return new InstantCommand(
                () -> Swerve.this.headingLock.enable(headingSupplier) 
            );
            
        }

        public Command enableStaticHeadingLock(Angle heading) {

            return this.enableDynamicHeadingLock(() -> heading);

        }
        
        public Command jumpToNextHeadingLockAngle(
            Angle increment,
            boolean toRight
        ) {
            
           return new DeferredCommand(() -> {
               
               Angle actualHeading = Swerve.this.getFieldRelativeHeading();
               Angle tolerance = Degrees.of(5);
               boolean isRobotAlreadyAtHeadingLock =
                   Swerve.this.headingLock.isEnabled() &&
                   actualHeading.isNear(Swerve.this.headingLock.getHeading(), tolerance);
               
               Angle originalHeading = isRobotAlreadyAtHeadingLock
                   ? Swerve.this.headingLock.getHeading()
                   : Swerve.this.getFieldRelativeHeading();
               
               Angle nextHeading = Degrees.of(DoubleUtilities.getNextIncrement(
                   originalHeading.in(Degrees),
                   increment.in(Degrees),
                   toRight
               ));
               
               return this.enableStaticHeadingLock(nextHeading);
               
           }, Set.of());
            
        }
        
        public Command enablePOIHeadingLock(
            Translation2d pointOfInterest,
            Angle relativeHeading
        ) {
            
            return this.enableDynamicHeadingLock(() -> {
                
                Pose2d currentPose = Swerve.this.odometry.getPose();
                
                if (currentPose == null) return Degrees.zero();
                
                Translation2d deltaTranslation = pointOfInterest
                    .minus(currentPose.getTranslation());
                
                return deltaTranslation.getAngle().getMeasure()
                    .plus(relativeHeading);
                
            });
            
        }
        
        public Command enablePOIHeadingLock(Translation2d pointOfInterest) {
            
            return this.enablePOIHeadingLock(
                pointOfInterest,
                Direction.FORWARDS
            );
            
        }
        
        public Command disablePOIRadiusLock() {
            
            return new InstantCommand(Swerve.this.radiusLock::disable);
            
        }
        
        public Command enablePOIRadiusLock(
            Point pointOfInterest,
            Distance radius
        ) {
            
            return new InstantCommand(
                () -> Swerve.this.radiusLock.enable(
                    () -> pointOfInterest,
                    () -> radius
                )
            );
            
        }
        
        public Command disablePOIHeadingAndRadiusLock() {
            
            return this.disableHeadingLock()
                .andThen(this.disablePOIRadiusLock());
            
        }
        
        public Command enablePOIHeadingAndRadiusLock(
            Point pointOfInterest,
            Distance radius,
            Angle relativeHeading
        ) {
            
            return this.enablePOIHeadingLock(pointOfInterest, relativeHeading)
                .andThen(this.enablePOIRadiusLock(pointOfInterest, radius));
            
        }
        
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
        
        public Command setModuleHeadings(Angle robotRelativeAngle) {
            
            return Swerve.this.runOnce(
                () -> Swerve.this.getModuleStream().forEach(
                    (module) -> module.updateModuleState(
                        new SwerveModuleState(0, new Rotation2d(robotRelativeAngle)),
                        false
                    )
                )
            );
            
        }
        
        public Command stop() {
            
            return new InstantCommand(Swerve.this::stop, Swerve.this);
            
        }
        
        public Command drive(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
            
            return Swerve.this.run(() -> Swerve.this.setChassisSpeeds(chassisSpeedsSupplier.get()));
            
        }
        
        public Command goToPosition(
            Supplier<Pose2d> poseSupplier,
            Distance distanceTolerance,
            Angle angularTolerance
        ) {

            AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(180);
            AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1);

            LinearMotionProfiler trajectory = new LinearMotionProfiler(
                /* Max Linear Velocity: */ FeetPerSecond.of(2),
                /* Max Linear Acceleration: */ FeetPerSecondPerSecond.of(10),
                /* Max Linear Deceleration: */ FeetPerSecondPerSecond.of(10)
            );
            PIDController thetaController = new PIDController(
                1,
                0,
                0
//                new TrapezoidProfile.Constraints(
//                    MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
//                    MAX_ANGULAR_ACCELERATION.in(DegreesPerSecondPerSecond)
//                )
            );
            thetaController.enableContinuousInput(-180, 180);

            Command command = new Command() {

                Pose2d desiredPose;
                Pose2d currentPose;

                @Override
                public void initialize() {
                    
                    System.out.println("goToPosition init");

//                    if (aprilTagFilter != null) {
//                        Swerve.this.odometry.vision.setAprilTagFilter(aprilTagFilter.get());
//                    }

                }

                Distance getRemainingLinearDistance() {

                    return Meters.of(
                        currentPose.getTranslation()
                            .minus(desiredPose.getTranslation())
                            .getNorm()
                    );

                }

                @Override
                public void execute() {

                    Pose2d newCurrentPose = Swerve.this.odometry.getPose();
                    Pose2d newDesiredPose = poseSupplier.get();

                    if (newCurrentPose != null) this.currentPose = newCurrentPose;
                    if (newDesiredPose != null) this.desiredPose = newDesiredPose;

                    Swerve.this.odometry.setDisplaySetpoint(this.desiredPose);
                    thetaController.setSetpoint(
                        this.desiredPose.getRotation().getMeasure().in(Degrees)
                    );
                    
                    System.out.println("-------------------");
                    System.out.println("remaining distance (ft): " + this.getRemainingLinearDistance().in(Feet));
                    System.out.println("current linear velocity (ft per sec): " + Swerve.this.getLinearVelocity().in(FeetPerSecond));

                    LinearVelocity velocity = trajectory.calculate(
                        this.getRemainingLinearDistance(),
                        Swerve.this.getLinearVelocity()
                    );
                    
                    System.out.println("positioning velocity (ft per sec): " + velocity.in(FeetPerSecond));

                    Translation2d deltaTranslation = this.desiredPose.getTranslation()
                        .minus(this.currentPose.getTranslation());

                    Translation2d chassisSpeedTranslation = new Translation2d(
                        velocity.in(MetersPerSecond),
                        deltaTranslation.getAngle()
                    ).times(VirtualField.isRedAlliance() ? -1 : 1);
                    
                    double thetaCalcResult = thetaController.calculate(currentPose.getRotation().getDegrees());
                    
//                    System.out.println("thetaCalcResult: " + thetaCalcResult + " deg per sec");
                    
                    double clampedThetaResult = MathUtil.clamp(
                        thetaCalcResult,
                        -MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
                        MAX_ANGULAR_VELOCITY.in(DegreesPerSecond)
                    );
                    
//                    System.out.println("clampedThetaResult: " + clampedThetaResult + " deg per sec");
                    
                    Swerve.this.setChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                            MetersPerSecond.of(chassisSpeedTranslation.getX()),
                            MetersPerSecond.of(chassisSpeedTranslation.getY()),
                            DegreesPerSecond.of(clampedThetaResult)
                        ), Swerve.this.gyro.yaw.getRotation())
                    );
                    
                }

                @Override
                public boolean isFinished() {

                    Pose2d relativePose = currentPose.relativeTo(desiredPose);
                    Distance linearDistance = Meters.of(relativePose.getTranslation().getNorm());

                    if (linearDistance.lt(Inches.of(0))) {

                        linearDistance = linearDistance.times(-1);

                    }

//                    System.out.println(
//                        "Distance remaining to target pose: " +
//                            linearDistance.in(Inches) + " inches"
//                    );

                    return (
                        linearDistance.lte(distanceTolerance) &&
                            currentPose.getRotation().getMeasure().isNear(
                                desiredPose.getRotation().getMeasure(),
                                angularTolerance
                            )
                    );

                }

                @Override
                public void end(boolean interrupted) {
                    
                    System.out.println("goToPosition ended");
                    Swerve.this.odometry.removeDisplaySetpoint();
//                    Swerve.this.odometry.vision.resetAprilTagFilter();
                    Swerve.this.stop();

                }

            };

            command.addRequirements(Swerve.this);

            return command;

        }

        public Command goToPosition(
            Supplier<Pose2d> poseSupplier
        ) {

            return this.goToPosition(
                poseSupplier,
                Inches.of(0.25),
                Degrees.of(1)
            );

        }
        
        public Command goToPosition2(
            Pose2d resultantPose
        ) {
            
            LinearVelocity maxLinearVelocity = FeetPerSecond.of(4);
            LinearAcceleration maxLinearAcceleration = FeetPerSecondPerSecond.of(2);
            AngularVelocity maxAngularVelocity = RotationsPerSecond.of(1);
            AngularAcceleration maxAngularAcceleration = RotationsPerSecondPerSecond.of(2);
            
            Command command = new Command() {
                
                HolonomicDriveController controller;
                
                Trajectory trajectory;
                
                double startTime;
                
                @Override
                public void initialize() {
                    
                    this.controller = new HolonomicDriveController(
                        new PIDController(0.001, 0, 0),
                        new PIDController(0.001, 0, 0),
                        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(
                            maxAngularVelocity.in(DegreesPerSecond),
                            maxAngularAcceleration.in(DegreesPerSecondPerSecond)
                        ))
                    );
                    
                    Pose2d initialPose = Swerve.this.odometry.getPose();
                    
                    initialPose = new Pose2d(
                        initialPose.getTranslation(),
                        resultantPose.getRotation()
                    );
                    
                    List<Pose2d> waypoints = List.of(initialPose, resultantPose);
                    
                    this.trajectory = TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        new TrajectoryConfig(
                            maxLinearVelocity,
                            maxLinearAcceleration
                        )
                    );
                    
                    this.startTime = Timer.getFPGATimestamp();
                    
                }
                
                double getElapsedSeconds() {
                    
                    return Timer.getFPGATimestamp() - this.startTime;
                    
                }
                
                @Override
                public void execute() {
                    
                    Trajectory.State goal =
                        this.trajectory.sample(this.getElapsedSeconds());
                    ChassisSpeeds adjustedSpeeds = this.controller.calculate(
                        Swerve.this.odometry.getPose(),
                        goal,
                        resultantPose.getRotation()
                    );
                    
                    Swerve.this.odometry.setDisplaySetpoint(goal.poseMeters);
                    
                    Swerve.this.setChassisSpeeds(adjustedSpeeds);
                    
                }
                
                @Override
                public boolean isFinished() {
                    
                    return this.getElapsedSeconds() >=
                        this.trajectory.getTotalTimeSeconds();
                    
                }
                
                @Override
                public void end(boolean interrupted) {
                    
                    
                    Swerve.this.stop();
                    Swerve.this.odometry.removeDisplaySetpoint();
                    
                }
                
            };
            
            command.addRequirements(Swerve.this);
            
            return command;
            
        }
        
        public Command goToPosition3(Pose2d resultantPose) {
            
            LinearVelocity maxLinearVelocity = FeetPerSecond.of(5);
            LinearAcceleration maxLinearAcceleration = FeetPerSecondPerSecond.of(8);
            AngularVelocity maxAngularVelocity = RotationsPerSecond.of(1);
            AngularAcceleration maxAngularAcceleration = RotationsPerSecondPerSecond.of(2);
            
            return new DeferredCommand(() -> {
                
                Pose2d initialPose = Swerve.this.odometry.getPose();
                List<Pose2d> waypoints = List.of(initialPose, resultantPose);
                
                TrajectoryConfig config = new TrajectoryConfig(
                    maxLinearVelocity,
                    maxLinearAcceleration
                );
                
                config.setKinematics(Swerve.this.getKinematics());
                
                Trajectory trajectory =
                    TrajectoryGenerator.generateTrajectory(waypoints, config);
                
                Swerve.this.odometry.setDisplaySetpoint(resultantPose);
                
                return new SwerveControllerCommand(
                    trajectory,
                    Swerve.this.odometry::getPose,
                    Swerve.this.kinematics,
                    new PIDController(1, 0, 0),
                    new PIDController(1, 0, 0),
                    new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(
                        maxAngularVelocity.in(DegreesPerSecond),
                        maxAngularAcceleration.in(DegreesPerSecondPerSecond)
                    )),
					resultantPose::getRotation,
					Swerve.this::applyModuleStates
                ).finallyDo(() -> {
                    Swerve.this.stop();
                    Swerve.this.odometry.removeDisplaySetpoint();
                });
                
            }, Set.of(Swerve.this));
            
        }

        public Command waitUntilAtPosition(
            Supplier<Pose2d> desiredPoseSupplier,
            Distance distanceTolerance,
            Angle angularTolerance
        ) {

            return edu.wpi.first.wpilibj2.command.Commands.waitUntil(() -> {

                Pose2d desiredPose = desiredPoseSupplier.get();
                Pose2d currentPose = Swerve.this.odometry.getPose();

                return (
                    currentPose.getMeasureX().isNear(desiredPose.getMeasureX(), distanceTolerance) &&
                        currentPose.getMeasureY().isNear(desiredPose.getMeasureY(), distanceTolerance) &&
                        currentPose.getRotation().getMeasure().isNear(desiredPose.getRotation().getMeasure(), angularTolerance)
                );

            });

        }
        
        public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
            
            return this.setModuleHeadings(Degrees.of(0))
                .andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(1)))
                .andThen(Swerve.this.getDriveMotorsSysIdRoutine().quasistatic(direction));
            
        }
        
        public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
            
            return this.setModuleHeadings(Degrees.of(0))
                .andThen(edu.wpi.first.wpilibj2.command.Commands.waitTime(Seconds.of(1)))
                .andThen(Swerve.this.getDriveMotorsSysIdRoutine().dynamic(direction));
            
        }
        
    }
    
}
