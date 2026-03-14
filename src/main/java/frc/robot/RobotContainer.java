// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.input.InputScheme;
import frc.robot.input.inputschemes.TestingTeleoperativeInputsScheme;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  
  protected static final InputScheme CONTROLS_SCHEME =
      new TestingTeleoperativeInputsScheme();
  
  public final Swerve swerve;
  
  public final Intake intake;
  
  public final Agitator agitator;
  
  public final Indexer indexer;
  
  public final Turret turret;
  
  public final Vision vision;
  
  public final RaptorsOdometry odometry;
  
  public final ComplexCommands complexCommands;

  public final CommandXboxController driverController;
  
  public final CommandXboxController operatorController;
  
  public RobotContainer() {
    
    
    this.intake = new Intake();
    this.agitator = new Agitator();
    this.indexer = new Indexer();
    this.turret = new Turret();
    this.odometry = new RaptorsOdometry();
    this.swerve = new Swerve(this.odometry);
    this.vision = new Vision(
        this.swerve::getFieldRelativeHeading,
        this.swerve::getLinearVelocity,
        this.swerve::getAngularVelocity
    );
    this.complexCommands = new ComplexCommands(this);
    this.driverController = new CommandXboxController(0);
    this.operatorController = new CommandXboxController(1);
    
    this.odometry.injectSwerve(this.swerve);
    this.odometry.injectVision(this.vision);
    
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
}
