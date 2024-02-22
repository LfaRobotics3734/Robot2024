// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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

    public static final int kDriverControllerPort = 0;

  }

  public static class IO{
    //CAN ID constants swerve drive
    public static int frontLeftDrive = 3;
    public static int frontLeftTurn = 4;

    public static int frontRightDrive = 1;
    public static int frontRightTurn = 2;
    
    public static int backLeftDrive = 7;
    public static int backLeftTurn = 8;

    public static int backRightDrive = 5;
    public static int backRightTurn = 6;

    public static int driveController = 0;
    public static int armController = 1;

    public static double kDriveDeadband = 0.05;
  }

  public static class Drive{
      public static double positionConversionFactor = PhysicalParams.wheelDiameters * Math.PI / PhysicalParams.drivingMotorReduction;
      public static double velocityConversionFactor = positionConversionFactor / 60.0;
      
      public static double p = 0.1;
      public static double i = 0;
      public static double d = 0;
      public static double ff = 1/PhysicalParams.drivingFreeSpeedRPS;

      //ratio for gears
      public static double lowGear = 0.225;
      public static double highGear = 1.0;
  }
  

  public static class Turn{

      public static double positionConversionFactor = 2 * Math.PI;
      public static double velocityConversionFactor = 2 * Math.PI/60;

      public static double p = 1;
      public static double i = 0;
      public static double d = 0;
      public static double ff = 0;

      public static double[] angleOffsets = {0, -Math.PI/2, Math.PI/2, Math.PI};
      //frontRight, frontLeft, backRight, backLeft

  }

  public static class PhysicalParams{

      //in meters
      public static double freeSpeedRPM = 5676;
      public static double freeSpeedRPS = freeSpeedRPM/60;
      public static double wheelDiameters = 0.0762;
      public static double wheelCircumference = wheelDiameters * Math.PI;
      public static double drivingMotorPinionTeeth = 13;
      public static double bevelGear = 45;
      public static double firstStageSpurGear = 22;
      public static double bevelPinion = 15;
      public static double drivingMotorReduction = (bevelGear * firstStageSpurGear)/(drivingMotorPinionTeeth*bevelPinion);
      public static double drivingFreeSpeedRPS = (freeSpeedRPS * wheelCircumference)/drivingMotorReduction;

  }

  public static class DriveConstants{
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%) (was 1.3)
    public static final double kRotationalSlewRate = 3; // percent per second (A = 100%)

    public static final double maxSpeed = 4;
    public static final double maxRotation = Math.PI;

    public static final double baseDimensions = Units.inchesToMeters(29.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(baseDimensions / 2, baseDimensions / 2),
        new Translation2d(baseDimensions / 2, -baseDimensions / 2),
        new Translation2d(-baseDimensions / 2, baseDimensions / 2),
        new Translation2d(-baseDimensions / 2, -baseDimensions / 2));
    
    public static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
  }

  public static class Autonomous{

    public static final double FIELD_HEIGHT = 8.0137;
    public static final int autoPath = 3;

    // Balance Constants
    public static final double balanceSpeed = 0.07; // In m/s
    public static final double balanceOnAngle = 2.5; // In degrees
    public static final double balanceOffAngle = 2;
    public static final int loops = 50;

    public static double TRANSLATION_PID=  11.4;
    public static double ROTATION_PID= 5;
  }

  public static class ShooterConstants{
    public static final double ELBOW_BASE_ANGLE = Math.PI;

    public static final double ELBOW_FLOOR_ANGLE = Math.PI;
    
    public static final double ELBOW_LENGTH = Math.PI;

    public static final double LIMELIGHT_ELBOW_ANGLE = Math.PI;

    public static final double ELBOW_MAX_SPEED = 0.5;
    public static final double ELBOW_FEED_ANGLE = Math.PI;

    public static final double[][] SHOOTER_ANGLES = new double[1][1];
    public static final double[][] SHOOTER_SPEEDS = new double[1][1];
    public static final double SPEAKER_X_POSITION = Math.PI;
    public static final double SPEAKER_Y_POSITION = Math.PI;

    public static final double ROTATE_SPEED = Math.PI;
  }

  public static class IntakeConstants{
    public static final double PINION_BASE_ANGLE = Math.PI;
    public static final double PINION_FLOOR_ANGLE = Math.PI;
    public static final double PINION_SOURCE_ANGLE = Math.PI;
    public static final double PINION_MAX_SPEED = Math.PI;
    public static final double GRAB_SPEED = Math.PI;
    public static final double TRANS_SPEED = Math.PI;
  }
}
