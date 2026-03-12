// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.input.InputScheme;
import frc.robot.input.inputschemes.StandardTeleoperativeInputsScheme;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  
  protected static final InputScheme CONTROLS_SCHEME =
      new StandardTeleoperativeInputsScheme();

  // Initializing the subsystems
  public final Swerve swerve = new Swerve();
  public final Indexer indexer = new Indexer();
  public final Agitator agitator = new Agitator();
//  public final Vision vision = new Vision();
//  public final Autons autons = new Autons(swerve, shooter, highway);
//  public final LEDs leds = new LEDs(LedConstants.CANDLEPORT, 8, 10);
  public final Turret turret = new Turret();
  public final Intake intake = new Intake();
  public final ComplexCommands complexCommands = new ComplexCommands(this);

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVERCONTROLLERPORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATORCONTROLLERPORT);
  
  public RobotContainer() {
//    SmartDashboard.putData("Reset Headings", swerve.resetSwerveHeadings());
//    SmartDashboard.putData("Swerve", swerve);
//    SmartDashboard.putData("Vision", vision);
//    SmartDashboard.putData("Hood", hood);
//    SmartDashboard.putData("Shooter", shooter);
//    SmartDashboard.putData("Turret", turret);

//    this.leds.runSolid(this.leds.green());
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
    
    swerve.setDefaultCommand(new SwerveDriveCommand(
      () -> -this.driverController.getLeftY(),
      () -> -this.driverController.getLeftX(),
      () -> -this.driverController.getRightX(),
      swerve
    ));
    
  }

  public void periodic() {
    
    RobotContainer.CONTROLS_SCHEME.periodic(
        this,
        this.driverController,
        this.operatorController
    );
    
  }
}
