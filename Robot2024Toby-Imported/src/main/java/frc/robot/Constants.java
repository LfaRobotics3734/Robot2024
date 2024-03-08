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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class IO {
        // CAN ID constants swerve drive
        public static int frontLeftDrive = 3;
        public static int frontLeftTurn = 4;

        public static int frontRightDrive = 1;
        public static int frontRightTurn = 2;

        public static int backLeftDrive = 7;
        public static int backLeftTurn = 8;

        public static int backRightDrive = 5;
        public static int backRightTurn = 6;

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static double kDriveDeadband = 0.05;
    }

    public static class Drive {
        public static double positionConversionFactor = PhysicalParams.wheelDiameters * Math.PI
                / PhysicalParams.drivingMotorReduction;
        public static double velocityConversionFactor = positionConversionFactor / 60.0;

        public static double p = 0.1;
        public static double i = 0;
        public static double d = 0;
        public static double ff = 1 / PhysicalParams.drivingFreeSpeedRPS;

        // ratio for gears
        public static double lowGear = 0.225;
        public static double highGear = 1.0;
    }

    public static class Turn {

        public static double positionConversionFactor = 2 * Math.PI;
        public static double velocityConversionFactor = 2 * Math.PI / 60;

        public static double p = 1;
        public static double i = 0;
        public static double d = 0;
        public static double ff = 0;

        public static double[] angleOffsets = { 0, -Math.PI / 2, Math.PI / 2, Math.PI };
        // frontRight, frontLeft, backRight, backLeft

    }

    public static class PhysicalParams {

        // in meters
        public static double freeSpeedRPM = 5676;
        public static double freeSpeedRPS = freeSpeedRPM / 60;
        public static double wheelDiameters = 0.0762;
        public static double wheelCircumference = wheelDiameters * Math.PI;
        public static double drivingMotorPinionTeeth = 13;
        public static double bevelGear = 45;
        public static double firstStageSpurGear = 22;
        public static double bevelPinion = 15;
        public static double drivingMotorReduction = (bevelGear * firstStageSpurGear)
                / (drivingMotorPinionTeeth * bevelPinion);
        public static double drivingFreeSpeedRPS = (freeSpeedRPS * wheelCircumference) / drivingMotorReduction;

    }

    public static class DriveConstants {
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

    public static class ControlConstants {
        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.75;

        public static final double kPOVLeft = 270;
        public static final double kPOVUp1 = 315;
        public static final double kPOVUp2 = 0;
        public static final double kPOVDown1 = 225;
        public static final double kPOVDown2 = 180;
    }

    public static class Autonomous {

        public static final double FIELD_HEIGHT = 8.0137;
        public static final int autoPath = 3;

        // Balance Constants
        public static final double balanceSpeed = 0.07; // In m/s
        public static final double balanceOnAngle = 2.5; // In degrees
        public static final double balanceOffAngle = 2;
        public static final int loops = 50;

        public static double TRANSLATION_PID = 11.4;
        public static double ROTATION_PID = 5;
    }

    public static class ShooterConstants {
        /**
         * kG 0.21
         * kV 5.36
         * kA 0.01
         */

        // public static final double ELBOW_BASE_ANGLE = Math.PI;

        // public static final double ELBOW_FLOOR_ANGLE = Math.PI;

        // public static final double ELBOW_LENGTH = Math.PI;

        // public static final double LIMELIGHT_ELBOW_ANGLE = Math.PI;

        // public static final double ELBOW_MAX_SPEED = 0.5;
        // public static final double ELBOW_FEED_ANGLE = Math.PI;

        public static final double[][] SHOOTER_ANGLES = new double[1][1];
        public static final double[][] SHOOTER_SPEEDS = new double[1][1];
        public static final double SPEAKER_X_POSITION = Math.PI;
        public static final double SPEAKER_Y_POSITION = Math.PI;

        public static final double kKS = .09091;
        public static final double kKV = .002052;
        // .0013 .000082

        // Trip sensors
        public static final double kTrip2Threshold = 3000.0;
        // public static final double 

        public static final int kAngleMotorID = 10;
        public static final int kShooterMotorLeftID = 9;
        public static final int kShooterMotorRightID = 14;
        public static final int kShooterTriggerMotorID = 16;

        public static final int kEncoderChannel = 0;
        public static final double kDistancePerRotation = 360.0;

        public static final double kDefaultAngleKP = 0.6;
        public static final double kDefaultAngleKI = 0;
        public static final double kDefaultAngleKD = 0;

        public static final double kDefaultShooterLeftKP = 0;
        public static final double kDefaultShooterLeftKI = 0;
        public static final double kDefaultShooterLeftKD = 0;

        public static final double kDefaultShooterRightKP = 0;
        public static final double kDefaultShooterRightKI = 0;
        public static final double kDefaultShooterRightKD = 0;

        public static final String kAngleKPKey = "kAngleKP";
        public static final String kAngleKIKey = "kAngleKI";
        public static final String kAngleKDKey = "kAngleKD";

        public static final String kShooterLeftKPKey = "kShooterLeftKP";
        public static final String kShooterLeftKIKey = "kShooterLeftKI";
        public static final String kShooterLeftKDKey = "kShooterLeftKD";

        public static final String kShooterRightKPKey = "kShooterRightKP";
        public static final String kShooterRightKIKey = "kShooterRightKI";
        public static final String kShooterRightKDKey = "kShooterRightKD";

        public static final String kEncoderOffsetKey = "kShooterEncoderOffset";

        public static final double kMinAngle = 16.0;
        public static final double kMaxAngle = 76.0;
        public static final double kAmpScorerFeedAngle = 76.0;
        public static final double kStowedAngle = 45.0;

        public static final double kRightSpeedOffset = 100.0;

        public static final double kSubwooferShotAngle = 60;
        public static final double kFeedSpeed = 1000;
        public static final double kDropAngle = 35.0;
        public static final double kSubwooferShotSpeed = 3500;

        public static final double kShooterTriggerSpeed = 4.0;
        
        public static final double kTripDelay = .115;

        public static enum ShooterPosition {
            SUBWOOFER,
            AUTOTARGET,
            AMP,
            STOW
        }
        
    }

    public static class IntakeConstants {
        /** 
         * kG 0.35
         * kV 8.77
         * kA 0.03
         */

        public static final double kMinAngle = 0.0;
        public static final double kMaxAngle = 60.0;

        public static final int kIntakeMotorID = 12;
        public static final int kIndexerMotorID = 11;
        public static final int kAngleMotorID = 13;

        public static final int kEncoderChannel = 2;
        public static final double kDistancePerRotation = 360.0;

        public static final double kDefaultKP = .2;
        public static final double kDefaultKI = 0;
        public static final double kDefaultKD = 0;

        public static final double kIntakeMinVoltage = 0.55;

        public static final String kKPKey = "kIntakeAngleKP";
        public static final String kKIKey = "kIntakeAngleKI";
        public static final String kKDKey = "kIntakeAngleKD";
        public static final String kEncoderOffsetKey = "kIntakeEncoderOffset";

        public static final double kRetratctedAngle = 60.0;
        public static final double kFloorAngle = 0.0;
        public static final double kSourceAngle = 30;
        // public static final double kMaxSpeed = Math.PI;

        public static final double kFloorIntakeSpeed = 5;
        public static final double kFloorIndexerSpeed = 5;
        public static final double kSourceIntakeSpeed = -8;
        public static final double kSourceIndexerSpeed = 8;
        public static final double kIndexerBaseSpeed = Math.PI;

        public static enum IntakePosition {
            RETRACTED,
            FLOOR,
            SOURCE
        }
    }

    public static class AmpScorerConstants {
        public static final int kMotorID = 17;
        public static final int kAngleMotorID = -1;

        public static final double kRotateSpeed = 9;
        public static final double kAngleSpeed = 3.0;
        
        public static final double kMaxAngleMotorCurrent = 20.0;
    }
}
