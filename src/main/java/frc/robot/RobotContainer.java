// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HighwayConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Highway;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // Initializing the subsystems
  private final Shooter shooter = new Shooter(ShooterConstants.TOPSHOOTERMOTORPORT, ShooterConstants.BOTTOMSHOOTERMOTORPORT);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Highway highway = new Highway(HighwayConstants.HIGHWAYPORT);
  private final Vision vision = new Vision();
  public final Autons autons = new Autons(swerve, shooter, highway);
  //private final LEDs leds1 = new LEDs(8,24);
  //private final LEDs leds2 = new LEDs(9,24);
  //private final Hood hood = new Hood(ShooterConstants.LEFTLINEARSERVOPORT, ShooterConstants.RIGHTLINEARSERVOPORT);
  private final Turret turret = new Turret(TurretConstants.TURRETMOTORPORT, TurretConstants.ENCODERREVERSED);
  private final Intake intake = new Intake(IntakeConstants.INTAKEPORT);

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVERCONTROLLERPORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATORCONTROLLERPORT);

  public RobotContainer() {
    SmartDashboard.putData("Reset Headings", swerve.resetSwerveHeadings());
    SmartDashboard.putData("Swerve", swerve);
    SmartDashboard.putData("Vision", vision);
    //SmartDashboard.putData("Hood", hood);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("Turret", turret);

    // Configure the controller bindings
    configureBindings();
  }

  /**
   * Method for adding the controller bindings to the controllers
   */
  private void configureBindings() {
    this.operatorController.a().whileTrue(this.shooter.commands.shootVelocity());
    //this.operatorController.povLeft().whileTrue(turret.commands.changeAngle(() -> 5));
    //this.operatorController.povRight().whileTrue(turret.commands.changeAngle(() -> -5));
    this.operatorController.rightTrigger().whileTrue(this.highway.commands.forward());
    this.driverController.rightTrigger().whileTrue(this.intake.commands.intake(() -> 0.4));

    swerve.setDefaultCommand(new SwerveDriveCommand(
      () -> -this.driverController.getLeftY(),
      () -> -this.driverController.getLeftX(),
      () -> -this.driverController.getRightX(),
      swerve
    ));

    //leds1.setDefaultCommand(leds1.runPattern(leds1.green()));

    this.driverController.start().onTrue(this.swerve.resetHeading());
  }

  public void periodic() {
    // Gets the limelight measurements and adds them to the swerve pose estimator
    for (var measurement : vision.getVisionMeasurements()) {
      swerve.addVisionMeasurement(
        measurement.pose(),
        measurement.timestampSeconds()
      );
    }
  }
}