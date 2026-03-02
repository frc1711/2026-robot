// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.FunctionUtilities;

public class Shooter extends SubsystemBase {

  // The motors on the shooter
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;

  public final Shooter.Commands commands = new Commands();

  // The control modes for getting to the set velocity (they adjust for voltage loss)
  private final MotionMagicVelocityVoltage topRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage bottomRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  
  // The different speeds to make the motors shoot at
  private double maxSpeed = 0.15;
  private double topTargetRPS = 0;
  private double bottomTargetRPS = 0;

  private final SlewRateLimiter topLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter bottomLimiter = new SlewRateLimiter(20);

  /**
   * The constructor method of the shooter subsytem
   * 
   * @param topShooterID The CAN ID of the top (left) shooter motor
   * @param bottomShooterID The CAN ID of the bottom (right) shooter motor
   */
  public Shooter(int topShooterID, int bottomShooterID) {
    // Make and configure the shooter motors
    this.topShooterMotor = new TalonFX(topShooterID);
    this.topShooterMotor.getConfigurator().apply(getTopMotorConfig());
    
    this.bottomShooterMotor = new TalonFX(bottomShooterID);
    this.bottomShooterMotor.getConfigurator().apply(getBottomMotorConfig());
  }

  /**
   * 
   * @return The configuration for the top shooter motor
   */
  private TalonFXConfiguration getTopMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kV = ShooterConstants.TOPMOTORKV;
    config.Slot0.kP = ShooterConstants.TOPMOTORKP;

    config.MotionMagic.MotionMagicAcceleration = ShooterConstants.SHOOTERMOTORACCELERATION;
    config.MotionMagic.MotionMagicJerk = ShooterConstants.SHOOTERMOTORJERK;

    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    return config;
  }

  /**
   * 
   * @return The configuration for the bottom shooter motor
   */
  private TalonFXConfiguration getBottomMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kV = ShooterConstants.BOTTOMMOTORKV;
    config.Slot0.kP = ShooterConstants.BOTTOMMOTORKP;

    config.MotionMagic.MotionMagicAcceleration = ShooterConstants.SHOOTERMOTORACCELERATION;
    config.MotionMagic.MotionMagicJerk = ShooterConstants.SHOOTERMOTORJERK;

    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    return config;
  }

  /**
   * Uses duty cycle to set the speed of the shooter motors (doesn't adjust for voltage loss)
   * 
   * @param topSpeed The target top shooter motor speed from -1.0 to 1.0
   * @param bottomSpeed The target bottom shooter motor speed from -1.0 to 1.0
   */
  public void setSpeed(double topSpeed, double bottomSpeed) {
    this.topShooterMotor.set(topSpeed);
    this.bottomShooterMotor.set(bottomSpeed);
  }

  /**
   * Uses {@link com.ctre.phoenix6.controls.MotionMagicVelocityVoltage MotionMagicVelocityVoltage} to set the target velocity of the shooter motors (adjusts for voltage loss)
   * 
   * @param topSpeed The target velocity of the top motor from -100 to 100 in RPS
   * @param bottomSpeed The target velocity of the bottom motor from -100 to 100 in RPS
   */
  public void setVelocity(double topSpeed, double bottomSpeed) {
    this.topTargetRPS = topSpeed;
    this.bottomTargetRPS = bottomSpeed;
  }

  @Override
  public void periodic() {
    double topSpeed = topLimiter.calculate(topTargetRPS);
    double bottomSpeed = bottomLimiter.calculate(bottomTargetRPS);

    this.topShooterMotor.setControl(topRequest.withVelocity(topSpeed));
    this.bottomShooterMotor.setControl(bottomRequest.withVelocity(bottomSpeed));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
      "Shooter Speed", 
      () -> this.maxSpeed, 
      (double d) -> {
        this.maxSpeed = d;
        System.out.println(maxSpeed + " RPS");
      }
    );
    builder.addDoubleProperty(
      "Top Motor RPS", 
      () -> this.topShooterMotor.getVelocity().getValueAsDouble(),
      null
    );
    builder.addDoubleProperty(
      "Bottom Motor RPS", 
      () -> this.bottomShooterMotor.getVelocity().getValueAsDouble(),
      null
    );
    builder.addDoubleArrayProperty(
      "Motor Current Draw", 
      () -> new double[]{this.topShooterMotor.getSupplyCurrent().getValueAsDouble(), this.bottomShooterMotor.getSupplyCurrent().getValueAsDouble()}, 
      null
    );
    builder.addDoubleArrayProperty(
      "Shooter Target RPS", 
      () -> new double[]{this.topTargetRPS, this.bottomTargetRPS}, 
      null
    );
    builder.addBooleanProperty(
      "At Speed", 
      this::atSpeed,
      null
    );
  }

  /**
   * 
   * @return If the shooter motors have reached their target RPS with some tolerance
   */
  public boolean atSpeed() {
    // If the shooter motors are not moving, don't return true
    if (Math.abs(topTargetRPS) < 1 && Math.abs(bottomTargetRPS) < 1) {
      return false;
    }

    // Gets the current shooter motor velocities
    double topVelocity = topShooterMotor.getVelocity().getValueAsDouble();
    double bottomVelocity = bottomShooterMotor.getVelocity().getValueAsDouble();

    // Uses 5% of target RPS as the tolerance for velocity of shooter motors
    double topTolerance = Math.abs(topTargetRPS) * 0.05;
    double bottomTolerance = Math.abs(bottomTargetRPS) * 0.05;

    SmartDashboard.putNumber("Top Tolerance", topTolerance);
    SmartDashboard.putNumber("Top Difference", topTargetRPS - topVelocity);

    return (topTargetRPS - topVelocity < topTolerance
      && bottomTargetRPS - bottomVelocity < bottomTolerance);
  }

  public class Commands {
    /**
     * A command that tries to set the target velocity of shooter motors to given values
     * 
     * @param topSupplier The {@link java.util.function.DoubleSupplier DoubleSupplier} for what the target RPS should be for the top shooter motor
     * @param bottomSupplier The {@link java.util.function.DoubleSupplier DoubleSupplier} for what the target RPS should be for the bottom shooter motor
     */
    public Command shootVelocity(DoubleSupplier topSupplier, DoubleSupplier bottomSupplier) {
      return Shooter.this.run(
        () -> Shooter.this.setVelocity(
          topSupplier.getAsDouble() * ShooterConstants.SHOOTERMAXRPS, 
          bottomSupplier.getAsDouble() * ShooterConstants.SHOOTERMAXRPS
        )
      ).finallyDo(
        () -> Shooter.this.setVelocity(0, 0)
      );
    }

    /**
     * A command that shoots using the maxSpeed variable
     */
    public Command shootVelocity() {
      return this.shootVelocity(
        () -> FunctionUtilities.applyClamp(Shooter.this.maxSpeed * ShooterConstants.TOPSHOOTERSPEEDMULTIPLIER, 0, 1), 
        () -> Shooter.this.maxSpeed
      ).finallyDo(() -> this.stopShooting());
    }

    /**
     * A command that sets the shooter motors back to 0 RPS
     */
    public Command stopShooting() {
      return Shooter.this.runOnce(
        () -> Shooter.this.setVelocity(0, 0)
      );
    }
  }
}
