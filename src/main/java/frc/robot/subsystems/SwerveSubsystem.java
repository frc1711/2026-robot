package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.FunctionUtilities;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;


public class SwerveSubsystem extends SubsystemBase {

    public final double maxSpeed = SwerveConstants.PHYSICALMAXSPEEDMPERSECR2;

    // The swerve modules
    public final SwerveModule fl_module = new SwerveModule(
        SwerveConstants.FRONTLEFTDRIVEMOTORPORT,
        SwerveConstants.FRONTLEFTTURNMOTORPORT,
        SwerveConstants.FRONTLEFTDRIVEENCODERREVERSED,
        SwerveConstants.FRONTLEFTABSENCODERPORT,
        SwerveConstants.FRONTLEFTABSENCODERREVERSED,
        0, 2);

    public final SwerveModule fr_module = new SwerveModule(
        SwerveConstants.FRONTRIGHTDRIVEMOTORPORT,
        SwerveConstants.FRONTRIGHTTURNMOTORPORT,
        SwerveConstants.FRONTRIGHTDRIVEENCODERREVERSED,
        SwerveConstants.FRONTRIGHTABSENCODERPORT,
        SwerveConstants.FRONTRIGHTABSENCODERREVERSED,
        1, 2);

    public final SwerveModule bl_module = new SwerveModule(
        SwerveConstants.BACKLEFTDRIVEMOTORPORT,
        SwerveConstants.BACKLEFTTURNMOTORPORT,
        SwerveConstants.BACKLEFTDRIVEENCODERREVERSED,
        SwerveConstants.BACKLEFTABSENCODERPORT,
        SwerveConstants.BACKLEFTABSENCODERREVERSED,
        2, 2);

    public final SwerveModule br_module = new SwerveModule(
        SwerveConstants.BACKRIGHTDRIVEMOTORPORT,
        SwerveConstants.BACKRIGHTTURNMOTORPORT,
        SwerveConstants.BACKRIGHTDRIVEENCODERREVERSED,
        SwerveConstants.BACKRIGHTABSENCODERPORT,
        SwerveConstants.BACKRIGHTABSENCODERREVERSED,
        3, 2);

    private final Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON2PORT);

    // Acts like the swerve odometry
    private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kDriveKinematics,
        getRotation2d(), 
        getPositions(),
        new Pose2d()
    );

    private SwerveModuleState[] desiredStates = {};

    private double speedMultiplier = 1;
    private double maxSpeedMultiplier = 1;

    // PID Controllers for choreo autos
    private PIDController xController = new PIDController(3, 0, 0);
    private PIDController yController = new PIDController(3, 0, 0);
    private PIDController headingController = new PIDController(5, 0, 0);

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> {this.runVolts(voltage.in(Volts));}, 
            null, 
            this
        )
    );

    // Used for displaying swerve modules on advantagescope
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    private final Field2d field;

    public SwerveSubsystem() {
        System.out.println("Swerve Subsystem initialized");
        this.resetGyro();

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.field = new Field2d();

        SmartDashboard.putData("Field", field);
    }

    /**
     * @return Returns the velocity of the robot's rotation
     */
    public double getTurnRate() {
        return this.gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    /**
     * Method used by choreo for following a path
     * 
     * @param sample
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        setChassisSpeeds(speeds, false);
    }

    /**
     * Gives the swerve modules what their desired state should be
     * 
     * @param desiredSpeed The desired {@link edu.wpi.first.math.kinematics.ChassisSpeeds ChassisSpeeds} of swerve
     * @param fieldRelative If the robot should drive field relative or not
     */
    public void setChassisSpeeds(ChassisSpeeds desiredSpeed, boolean fieldRelative) {
        // Make the speeds field relative if they should be field relative
        ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeed, getRotation2d()) : desiredSpeed;
        //ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeed, getRotation2d());

        // Get the states
        SwerveModuleState[] newStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, this.maxSpeed);

        // Set the states of the modules
        fl_module.setDesiredState(newStates[0]);
        fr_module.setDesiredState(newStates[1]);
        bl_module.setDesiredState(newStates[2]);
        br_module.setDesiredState(newStates[3]);

        // Used for the publisher
        this.desiredStates = newStates;
    }

    private void runVolts(double volts) {
        fl_module.runVolts(volts/2);
        fr_module.runVolts(volts/2);
        bl_module.runVolts(volts/2);
        br_module.runVolts(volts/2);
    }

    /**
     * Converts speeds into {@link edu.wpi.first.math.kinematics.ChassisSpeeds ChassisSpeeds}
     * 
     * @param xSpeed The speed of forward/backwards of the robot
     * @param ySpeed The speed of left/right of the robot
     * @param rot The speed of the robot's rotation
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            xSpeed,
            ySpeed,
            rot
        );

        setChassisSpeeds(desiredSpeeds.times(this.speedMultiplier).times(SwerveConstants.TELEOPSPEEDMULTIPLIER), true);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        estimator.addVisionMeasurement(visionPose, timestamp);
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getRotation2d(), getPositions(), pose);
    }

    /**
     * @return Puts the {@link edu.wpi.first.math.kinematics.SwerveModulePosition SwerveModulePositions} into an array
     */
    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = fl_module.getPosition();
        positions[1] = fr_module.getPosition();
        positions[2] = bl_module.getPosition();
        positions[3] = br_module.getPosition();

        return positions;
    }

    /**
     * Used for setting the speed multiplier of swerve (for slowing it down)
     * 
     * @param mult What should the multiplier be
     */
    public void setSpeed(double mult) {

        this.speedMultiplier = FunctionUtilities.applyClamp(
            mult, 
            0, 
            this.maxSpeedMultiplier);
    }

    @Override
    public void periodic() {
        // Run the modules periodics
        fl_module.periodic();
        fr_module.periodic();
        bl_module.periodic();
        br_module.periodic();

        // Update the odometry and add to field
        estimator.update(getRotation2d(), getPositions());
        this.field.setRobotPose(getPose());

        // Get the states and send them with the publisher
        SwerveModuleState[] loggingStates = null;

        if (Robot.isSimulation()) {
            loggingStates = desiredStates;
        } else {
            loggingStates = new SwerveModuleState[]{fl_module.getState(), fr_module.getState(), bl_module.getState(), br_module.getState()};
            // loggingStates = desiredStates;
        }
        
        publisher.set(loggingStates);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("FL Module Angle", 
            () -> fl_module.getAbsoluteEncoderDeg(),
            null);
        builder.addDoubleProperty("FR Module Angle", 
            () -> fr_module.getAbsoluteEncoderDeg(),
            null);
        builder.addDoubleProperty("BL Module Angle", 
            () -> bl_module.getAbsoluteEncoderDeg(),
            null);
        builder.addDoubleProperty("BR Module Angle", 
            () -> br_module.getAbsoluteEncoderDeg(),
            null);
        builder.addBooleanProperty("New Driver Mode",
            () -> this.maxSpeedMultiplier < 1 ? true : false,
            (boolean mult) -> this.setMaxSpeed(mult ? 0.5 : 1));
        builder.addDoubleProperty("Gyro", 
            () -> getRotation2d().getDegrees(),
            null);
        builder.addDoubleArrayProperty("PID Values",
            () -> this.getPid(),
            (double[] pid) -> this.setPid(pid));
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Sets the max speed multiplier of swerve
     * 
     * @param mult The max value of the speed multiplier
     */
    private void setMaxSpeed(double mult) {
        System.out.println(mult);
        this.maxSpeedMultiplier = mult;
        this.setSpeed(mult);
    }

    /**
     * @return P and D values of the swerve modules
     */
    public double[] getPid() {
        PIDController turningController = fl_module.getTurningController();
        double[] pidValues = {turningController.getP(),turningController.getD()};

        return pidValues;

    }

    /**
     * Changes the P and D values of the swerve modules
     * 
     * @param pid The array used for setting P and D. Array is [pValue, dValue]
     */
    public void setPid(double[] pid) {
        this.setTurningP(pid[0]);
        this.setTurningD(pid[1]);
    }

    /**
     * Sets the P value of the turning controller for the swerve modules
     * 
     * @param p value
     */
    private void setTurningP(double p) {
        fl_module.setTurningControllerP(p);
        fr_module.setTurningControllerP(p);
        bl_module.setTurningControllerP(p);
        br_module.setTurningControllerP(p);
    }

    /**
     * Sets the D value of the turning controller for the swerve modules
     * 
     * @param d value
     */
    private void setTurningD(double d) {
        fl_module.setTurningControllerD(d);
        fr_module.setTurningControllerD(d);
        bl_module.setTurningControllerD(d);
        br_module.setTurningControllerD(d);
    }

    public void stopModules() {
        fl_module.stop();
        fr_module.stop();
        bl_module.stop();
        br_module.stop();
    }

    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Used to reset the zeroes of the swerve modules
     */
    public Command resetSwerveHeadings() {
        return Commands.runOnce(() -> {
            fl_module.resetEncoders();
            fr_module.resetEncoders();
            bl_module.resetEncoders();
            br_module.resetEncoders();
        }).ignoringDisable(true);
    }

    public Command getAbsoluteEncoder() {
        return Commands.runOnce(() -> {
            fl_module.getAbsoluteEncoderDeg();
            fr_module.getAbsoluteEncoderDeg();
            bl_module.getAbsoluteEncoderDeg();
            br_module.getAbsoluteEncoderDeg();
        }).ignoringDisable(true);
    }

    public Command resetHeading() {
        return Commands.runOnce(this::resetGyro);
    }

    public Command setSpeedMultiplier(double mult) {
        return Commands.runOnce(() -> this.setSpeed(mult));
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}