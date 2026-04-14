// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.input.InputScheme;
import frc.robot.input.inputschemes.StandardTeleoperativeInputsScheme;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.HubShiftUtil;

public class RobotContainer {
  
  protected static final InputScheme CONTROLS_SCHEME =
      new StandardTeleoperativeInputsScheme();
  
  public final Swerve swerve;
  
  // public final Intake intake;
  
  public final Indexer indexer;

  public final Agitator agitator;
  
  public final Turret turret;

  public final Flywheel flyWheel;
  
  public final Vision vision;
  
  public final RaptorsOdometry odometry;
  
  public final ComplexCommands complexCommands;

  public final CommandXboxController driverController;
  
  public final CommandXboxController operatorController;
  
  public RobotContainer() {
    
    // this.intake = new Intake();
    this.indexer = new Indexer();
    this.turret = new Turret();
    this.flyWheel = new Flywheel();
    this.odometry = new RaptorsOdometry();
    this.swerve = new Swerve(this.odometry);
    this.vision = new Vision(
        this.swerve::getFieldRelativeHeading,
        this.swerve::getLinearVelocity,
        this.swerve::getAngularVelocity
    );
    this.agitator = new Agitator();
    this.complexCommands = new ComplexCommands(this);
    this.driverController = new CommandXboxController(0);
    this.operatorController = new CommandXboxController(1);
    
    this.odometry.injectSwerve(this.swerve);
    this.odometry.injectVision(this.vision);
    
    // Reset hub shift timer when enabling
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    
  }

  /**
   * Method for adding the controller bindings to the controllers
   */
  public void configureTeleoperativeControls() {
    
    CONTROLS_SCHEME.configureControllerInputs(
        this,
        this.driverController,
        this.operatorController
    );
  }
  
  public void init() {
    
    RobotContainer.CONTROLS_SCHEME.init(
        this,
        this.driverController,
        this.operatorController
    );
    
  }

  public void periodic() {
    
    RobotContainer.CONTROLS_SCHEME.periodic(
        this,
        this.driverController,
        this.operatorController
    );
    
  }

  public void updateDashboardOutputs() {
    SmartDashboard.putString("Hub/Shift Time Remaining",
            String.format("%.1f", Math.max(HubShiftUtil.getOfficialShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Hub/Hub Active", HubShiftUtil.getOfficialShiftInfo().active());
    SmartDashboard.putString("Hub/Current Shift", HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean("Hub/Won Auto", HubShiftUtil.getFirstActiveAlliance() != DriverStation.getAlliance().orElse(Alliance.Blue));
    SmartDashboard.putData("Swerve", swerve);
  }
}
