// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.DoublePreference;

public class Robot extends TimedRobot {

  protected RobotContainer robotContainer;

  protected Command autonCommand;

  public Robot() {

    this.robotContainer = new RobotContainer();
    
    DoublePreference.init();

  }

  @Override
  public void robotInit() {

    Auton.initializeShuffleboardSelector();
    this.robotContainer.init();
    this.robotContainer.vision.beginStableSeeding();
//    this.robotContainer.swerve.calibrateFieldRelativeHeading(Degrees.of(180));

  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    robotContainer.updateDashboardOutputs();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    
    this.robotContainer.vision.beginUsingInternalLL4IMUAssist();
    this.autonCommand = Auton.runSelectedAuton(this.robotContainer);

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    this.robotContainer.vision.beginUsingInternalLL4IMUAssist();
    this.robotContainer.configureTeleoperativeControls();

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();

  }

  @Override
  public void testPeriodic() {}

}
