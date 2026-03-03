// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVERCONTROLLERPORT = 0;
    public static final int OPERATORCONTROLLERPORT = 1;

    public static final double DEADBAND = 0.05;
  }

  public static class ShooterConstants {
    public static final int TOPSHOOTERMOTORPORT = 13;
    public static final int BOTTOMSHOOTERMOTORPORT = 14;

    public static final int LEFTLINEARSERVOPORT = 8;
    public static final int RIGHTLINEARSERVOPORT = 9;

    public static final double TOPMOTORKV = 0.37;
    public static final double TOPMOTORKP = 0.15;

    public static final double BOTTOMMOTORKV = 0.37;
    public static final double BOTTOMMOTORKP = 0.15;

    public static final double TOPSHOOTERSPEEDMULTIPLIER = 1.75;
    public static final double SHOOTERMAXRPS = 100;

    public static final double SHOOTERMOTORACCELERATION = 150;
    public static final double SHOOTERMOTORJERK = 1500;
  }

  public static class ClimberConstants {
    public static final int CLIMBERMOTORPORT = 18;
  }
  
  public static class TurretConstants {
    public static final double TURRETP = 1;
    public static final double TURRETI = 0;
    public static final double TURRETD = 0;
    public static final double TURRETV = 0.1;

    public static int TURRETMOTORPORT = 16;

    public static final double TURRETDIAMETER = Units.inchesToMeters(8.625);
    public static final double ROTATIONSTOMETERS = (TURRETDIAMETER * Math.PI);
    public static final double TURRETGEARRATIO = 11/136;

    public static final boolean ENCODERREVERSED = false;

    public static final double TURRETMAXANGLE = Units.degreesToRadians(80);
    public static final double TURRETMINANGLE = 0;
  }

  public static class HighwayConstants {
    public static final int HIGHWAYPORT = 17;
  }

  public static class IntakeConstants {
    public static final int HOPPEREXTENDPORT = 19;
    public static final int INTAKEPORT = 20;

    public static final double INTAKEV = 0.37;
    public static final double INTAKEP = 0.1;

    public static final double INTAKEMAXRPS = 100;

    public static final double INTAKEMOTORACCELERATION = 150;
    public static final double INTAKEMOTORJERK = 1500;
  }

  public static class SwerveConstants {
    public static final double WHEELDIAMETERMETERS = Units.inchesToMeters(4);
    public static final double WHEELBASELENGTH = Units.inchesToMeters(27.125);
    public static final double WHEELBASEWIDTH = Units.inchesToMeters(16);

    public static final double SWERVETURNINGP = 2.5;
    public static final double SWERVETURNINGI = 0;
    public static final double SWERVETURNINGD = 0;

    public static final double PHYSICALMAXSPEEDMPERSECR1 = Units.feetToMeters(14.4);
    public static final double PHYSICALMAXSPEEDMPERSECR2 = Units.feetToMeters(16.8);
    public static final double PHYSICALMAXSPEEDMPERSECR3 = Units.feetToMeters(19.2);

    public static final double TELEOPSPEEDMULTIPLIER = 0.5;

    public static final double DRIVEMOTORGEARRATIOR1 = 7.03;
    public static final double DRIVEMOTORGEARRATIOR2 = 6.03;
    public static final double DRIVEMOTORGEARRATIOR3 = 5.27;
    public static final double STEERINGGEARRATIO = 26;

    public static final double ROTATIONSTOMETERSR1 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR1;
    public static final double ROTATIONSTOMETERSR2 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR2;
    public static final double ROTATIONSTOMETERSR3 = (WHEELDIAMETERMETERS * Math.PI) / DRIVEMOTORGEARRATIOR3;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEELBASELENGTH / 2, WHEELBASEWIDTH / 2),
                new Translation2d(WHEELBASELENGTH / 2, -WHEELBASEWIDTH / 2),
                new Translation2d(-WHEELBASELENGTH / 2, WHEELBASEWIDTH / 2),
                new Translation2d(-WHEELBASELENGTH / 2, -WHEELBASEWIDTH / 2));

    public static final int PIGEON2PORT = 15;

    public static final int FRONTLEFTDRIVEMOTORPORT = 1;
    public static final int FRONTLEFTTURNMOTORPORT = 2;

    public static final int FRONTRIGHTDRIVEMOTORPORT = 3;
    public static final int FRONTRIGHTTURNMOTORPORT = 4;

    public static final int BACKLEFTDRIVEMOTORPORT = 5;
    public static final int BACKLEFTTURNMOTORPORT = 6;

    public static final int BACKRIGHTDRIVEMOTORPORT = 7;
    public static final int BACKRIGHTTURNMOTORPORT = 8;

    public static final boolean FRONTLEFTDRIVEENCODERREVERSED = false;
    public static final boolean FRONTRIGHTDRIVEENCODERREVERSED = false;
    public static final boolean BACKLEFTDRIVEENCODERREVERSED = false;
    public static final boolean BACKRIGHTDRIVEENCODERREVERSED = false;

    public static final int FRONTLEFTABSENCODERPORT = 9;
    public static final int FRONTRIGHTABSENCODERPORT = 10;
    public static final int BACKLEFTABSENCODERPORT = 11;
    public static final int BACKRIGHTABSENCODERPORT = 12;

    public static final boolean FRONTLEFTABSENCODERREVERSED = false;
    public static final boolean FRONTRIGHTABSENCODERREVERSED = false;
    public static final boolean BACKLEFTABSENCODERREVERSED = false;
    public static final boolean BACKRIGHTABSENCODERREVERSED = false;

    public static final double FRONTLEFTABSENCODEROFFSET = 0;
    public static final double FRONTRIGHTABSENCODEROFFSET = 0;
    public static final double BACKLEFTABSENCODEROFFSET = 0;
    public static final double BACKRIGHTABSENCODEROFFSET = 0;
  }
}
