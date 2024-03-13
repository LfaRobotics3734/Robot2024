package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.utils.AllianceFlipUtil;
import frc.utils.FieldConstants;
import frc.utils.LinearInterpolator;

public class Shooter extends SubsystemBase {

    private DutyCycleEncoder mEncoder = new DutyCycleEncoder(ShooterConstants.kEncoderChannel);

    private SimpleMotorFeedforward mShooterLeftFeedFwd = new SimpleMotorFeedforward(ShooterConstants.kKSLeft,
            ShooterConstants.kKVLeft, 0);
    private SimpleMotorFeedforward mShooterRightFeedFwd = new SimpleMotorFeedforward(ShooterConstants.kKSRight,
            ShooterConstants.kKVRight, 0);
    // private SimpleMotorFeedforward mRightFeedFwd = new
    // SimpleMotorFeedforward(ShooterConstants.kKS, ShooterConstants.kKV, 0);

    private PIDController mAnglePID = new PIDController(0.75, ShooterConstants.kDefaultAngleKI,
            ShooterConstants.kDefaultAngleKI);
    private PIDController mShooterLeftPID = new PIDController(.0011,
            ShooterConstants.kDefaultShooterLeftKI, .00007);
    private PIDController mShooterRightPID = new PIDController(.0011,
            ShooterConstants.kDefaultShooterRightKI, .00007);

    double mAngleKP, mAngleKI, mAngleKD, mShooterLeftKP, mShooterLeftKI, mShooterLeftKD, mShooterRightKP,
            mShooterRightKI, mShooterRightKD;
    boolean mShooterRunning = false;

    private CANSparkMax mAngleMotor = new CANSparkMax(ShooterConstants.kAngleMotorID, MotorType.kBrushless);
    private CANSparkMax mShooterMotorLeft = new CANSparkMax(ShooterConstants.kShooterMotorLeftID, MotorType.kBrushless);
    private CANSparkMax mShooterMotorRight = new CANSparkMax(ShooterConstants.kShooterMotorRightID,
            MotorType.kBrushless);
    private CANSparkMax mShooterTriggerMotor = new CANSparkMax(ShooterConstants.kShooterTriggerMotorID,
            MotorType.kBrushless);

    // private AnalogInput mTrip1 = new AnalogInput(0);
    private AnalogInput mTrip2 = new AnalogInput(1);
    private BooleanSupplier mTripped;

    private double voltageConstant = 0.0;

    // private double leftVoltage, rightVoltage = 0.0;

    // private Limelight limelight;
    private LinearInterpolator mAngleInterpolator;
    private LinearInterpolator mSpeedInterpolator;

    // private SwerveDrivePoseEstimator mPoseEstimator;

    private ShooterConstants.ShooterPosition mCurrentPosition = ShooterConstants.ShooterPosition.STOW;

    double shootSpeed, shootAngle;

    private boolean mCanStow = true;
    // private boolean shouldStow = false;

    public Shooter(Limelight limelight, SwerveDrivePoseEstimator poseEstimator) {
        // bruh
        mShooterTriggerMotor.setInverted(true);
        mAngleMotor.setInverted(true);
        mShooterMotorLeft.setInverted(true);
        mShooterMotorRight.setInverted(true);

        // I think this needs to be different because of the absolute encoder
        // Yes it did
        // - jamie :)
        // mPoseEstimator = poseEstimator;

        AnalogInput.setGlobalSampleRate(62500);
        mTrip2.setAverageBits(1);

        mTripped = () -> mTrip2.getAverageValue() < ShooterConstants.kTrip2Threshold;

        loadPreferences();
        // System.out.println("kP: " + mShooterLeftKP);

        // this.limelight = limelight;
        mAngleInterpolator = new LinearInterpolator(ShooterConstants.kShooterAngles);
        mSpeedInterpolator = new LinearInterpolator(ShooterConstants.kShooterSpeeds);

        // mEncoder.reverseDirection();
        mEncoder.setDistancePerRotation(ShooterConstants.kDistancePerRotation);

        // mShooterLeftKP = .0013;
        // mShooterLeftKD = .000082;
        // mShooterRightKP = .0013;
        // mShooterRightKD = .000082;
        // writePIDConstants();

        mAnglePID.setTolerance(1);
        mShooterLeftPID.setTolerance(25);
        mShooterRightPID.setTolerance(25);

        mAnglePID.setSetpoint(ShooterConstants.kStowedAngle);

        // PID tuning
        // SmartDashboard.putData("Burn PID constants (shoot)", new InstantCommand(() -> writePIDConstants()) {
        //     @Override
        //     public boolean runsWhenDisabled() {
        //         return true;
        //     }
        // });
        SmartDashboard.putData("Update shoota", new InstantCommand(() -> updatePIDValues()) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        SmartDashboard.putNumber("Shooter Setpoint", 16.0);
        SmartDashboard.putNumber("Shooter kP", 0.0);
        SmartDashboard.putNumber("Shooter kI", 0.0);
        SmartDashboard.putNumber("Shooter kD", 0.0);
        SmartDashboard.putNumber("Shooter Voltage constant", 0.0);
        // SmartDashboard.putNumber("Shooter Angle", 0.0);
        SmartDashboard.putNumber("Shot Angle", 45.0);
        SmartDashboard.putNumber("Shot Speed", 0.0);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Trip 1", mTrip1.getValue());
        // SmartDashboard.putNumber("Trip 2", mTrip2.getValue());
        // if(mTrip2.getValue() <= ShooterConstants.kTrip2Threshold){
        // stopShoot();
        // }
        // System.out.println(mTrip2.getAverageValue());
        // if(shouldStow) {
        //     stow();
        // }

        runMotors();
        // double x = mLeftFeedFwd.calculate(rmShooterLeftPID.getSetpoint()) +
        // mShooterLeftPID.calculate(mShooterMotorLeft.getEncoder().getVelocity());
        // System.out.println(x);
        // mShooterMotorLeft.setVoltage(x);
        // System.out.println(mShooterLeftKP);
        SmartDashboard.putNumber("Shooter Angle", getAbsoluteDistance(mEncoder));
        // SmartDashboard.getNumber("")
        // voltageConstant = SmartDashboard.getNumber("Shooter Voltage constant", 0.0);
        // updatePIDValues();
    }

    public void updatePIDValues() {
        // mAnglePID.setSetpoint(SmartDashboard.getNumber("Shooter Setpoint", 16.0));
        // mAngleKP = SmartDashboard.getNumber("Shooter kP", 0.0);
        // mAngleKI = SmartDashboard.getNumber("Shooter kI", 0.0);
        // mAngleKD = SmartDashboard.getNumber("Shooter kD", 0.0);
        // mAnglePID.setPID(mAngleKP, mAngleKI, mAngleKD);
        shootSpeed = SmartDashboard.getNumber("Shot Speed", 0.0);
        shootAngle = SmartDashboard.getNumber("Shot Angle", 35.0);
        // System.out.println(shootSpeed);

        // leftVoltage = SmartDashboard.getNumber("Shooter Voltage constant", 0.0);
        // rightVoltage = SmartDashboard.getNumber("Shooter Voltage constant", 0.0);
        // System.out.println(leftVoltage)
    }

    public BooleanSupplier getTripStatus() {
        return mTripped;
    }

    public void writePIDConstants() {
        Preferences.setDouble(ShooterConstants.kAngleKPKey, mAngleKP);
        Preferences.setDouble(ShooterConstants.kAngleKIKey, mAngleKI);
        Preferences.setDouble(ShooterConstants.kAngleKDKey, mAngleKD);
    }

    public void loadPreferences() {
        // mAngleKP = Preferences.getDouble(ShooterConstants.kAngleKPKey, ShooterConstants.kDefaultAngleKP);
        // mAngleKI = Preferences.getDouble(ShooterConstants.kAngleKIKey, ShooterConstants.kDefaultAngleKI);
        // mAngleKD = Preferences.getDouble(ShooterConstants.kAngleKDKey, ShooterConstants.kDefaultAngleKD);

        // mShooterLeftKP = Preferences.getDouble(ShooterConstants.kShooterLeftKPKey,
        //         ShooterConstants.kDefaultShooterLeftKP);
        // mShooterLeftKI = Preferences.getDouble(ShooterConstants.kShooterLeftKIKey,
        //         ShooterConstants.kDefaultShooterLeftKI);
        // mShooterLeftKD = Preferences.getDouble(ShooterConstants.kShooterLeftKDKey,
        //         ShooterConstants.kDefaultShooterLeftKD);

        // mShooterRightKP = Preferences.getDouble(ShooterConstants.kShooterRightKPKey,
        //         ShooterConstants.kDefaultShooterRightKP);
        // mShooterRightKI = Preferences.getDouble(ShooterConstants.kShooterRightKIKey,
        //         ShooterConstants.kDefaultShooterRightKI);
        // mShooterRightKD = Preferences.getDouble(ShooterConstants.kShooterRightKDKey,
        //         ShooterConstants.kDefaultShooterRightKD);

        // mAnglePID.setPID(mAngleKP, mAngleKI, mAngleKD);
        // mShooterLeftPID.setPID(mShooterLeftKP, mShooterLeftKI, mShooterLeftKD);
        // mShooterRightPID.setPID(mShooterRightKP, mShooterRightKI, mShooterRightKD);



        mEncoder.setPositionOffset(Preferences.getDouble(ShooterConstants.kEncoderOffsetKey, 0.0));
    }

    // public void saveShooterSetpoints() {
    //     Preferences.set
    // }

    // Sets encoder zero point and writes offset to flash.
    // Zero point (degrees) should be level with the ground
    public void resetEncoder() {
        mEncoder.reset();
        // mEncoder.setPositionOffset(mEncoder.getPositionOffset() + (16.0/360.0));
        Preferences.setDouble(ShooterConstants.kEncoderOffsetKey, mEncoder.getPositionOffset());
        logEncoderValues();
    }

    public ShooterConstants.ShooterPosition getCurrentPosition() {
        return mCurrentPosition;
    }

    // get left motor, & right motor up to speed - shoot after a second or two
    public void autotarget(Pose2d pose, ChassisSpeeds fieldRelativeSpeeds) {
        // Pose2d pose = mPoseEstimator.getEstimatedPosition();
        // double xVel = fieldRelativeSpeeds.vxMetersPerSecond;
        // double yVel = fieldRelativeSpeeds.vyMetersPerSecond;

        // if(AllianceFlipUtil.shouldFlip()) {
        //     yVel *= -1;
        // }


        double xDist = Math.abs(pose.getX() - AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getX());
        double yDist = Math.abs(pose.getY() - AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getY());
        double linearDist = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        // SmartDashboard.putNumber("Speaker Distance", linearDist);

        // double approxShotTimeToTarget = linearDist / (mSpeedInterpolator.getInterpolatedValue(linearDist) * ShooterConstants.kShotSpeedPerRPM);
        // xDist += approxShotTimeToTarget * xVel;
        // yDist += approxShotTimeToTarget * yVel;
        // linearDist = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
        // System.out.println("Time to target: " + approxShotTimeToTarget + " X velocity: " + xVel  + " Distance diff: " + (xDist - Math.abs(pose.getX() - AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getX())));
        // System.out.println("Pose: " + pose + "Current distance: " + xDist + ", " + yDist);
        // double yCoord = pose.getY() - ShooterConstants.SPEAKER_Y_POSITION;
        // shootSpeed = mSpeedInterpolator
        //         .getInterpolatedValue(linearDist);
        // shootAngle = MathUtil.clamp(mAngleInterpolator
                // .getInterpolatedValue(linearDist), ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
        // System.out.println("Speed: " + shootSpeed + " Angle: " + shootAngle + " Distance: " + linearDist);
        System.out.println("Set: " + shootSpeed + " Left: " + mShooterMotorLeft.getEncoder().getVelocity() + " Right: " + mShooterMotorRight.getEncoder().getVelocity());

        // System.out.println(shootSpeed);
        mAnglePID.setSetpoint(shootAngle);
        mShooterLeftPID.setSetpoint(shootSpeed + ShooterConstants.kRightSpeedOffset);
        mShooterRightPID.setSetpoint(shootSpeed - ShooterConstants.kRightSpeedOffset);
        mShooterRunning = true;
        mCurrentPosition = ShooterConstants.ShooterPosition.AUTOTARGET;
    }

    public void subwooferShot() {
        mShooterRunning = true;
        mShooterLeftPID.setSetpoint(ShooterConstants.kSubwooferShotSpeed);
        mShooterRightPID.setSetpoint(ShooterConstants.kSubwooferShotSpeed - ShooterConstants.kRightSpeedOffset);
        mAnglePID.setSetpoint(ShooterConstants.kSubwooferShotAngle);
        mCurrentPosition = ShooterConstants.ShooterPosition.SUBWOOFER;
        // System.out.println("running " + leftVoltage);

    }

    // stop moving the elbow
    // public void stopAngle() {
    // elbow.set(0.001);
    // }

    public void stow() {
        // System.out.println("" + mCanStow + shouldStow);
        // if(mCanStow) {
            // shouldStow = false;
        mShooterRunning = false;
        stopShoot();
        mAnglePID.setSetpoint(ShooterConstants.kStowedAngle);
        mCurrentPosition = ShooterConstants.ShooterPosition.STOW;
        // } 
        // else {
        //     shouldStow = true;
        // }
    }

    public void stopThingsButNoStow() {
        if(mCanStow) {
            mShooterRunning = false;
            stopShoot();
        } 

    }

    public void panic() {
        mShooterRunning = false;
        stopShoot();
        stopTrigger();
        mAnglePID.setSetpoint(60);
    }

    public void feed() {
        mShooterLeftPID.setSetpoint(ShooterConstants.kFeedSpeed);
        mShooterRightPID.setSetpoint(ShooterConstants.kFeedSpeed);
        mAnglePID.setSetpoint(ShooterConstants.kAmpScorerFeedAngle);
        mCurrentPosition = ShooterConstants.ShooterPosition.AMP;
        mShooterRunning = true;
    }

    // stop the shooter from rotating
    public void stopShoot() {
        mShooterMotorLeft.set(0.0);
        mShooterMotorRight.set(0.0);
        mShooterRunning = false;
        // stopTrigger();
    }

    // Print encoder values for debugging
    public void logEncoderValues() {
        System.out.println("Elbow Degrees: " + mEncoder.getDistance());
    }

    // turn potentiometer values to degrees
    // private double potentiometerToDegrees(double potValue) {
    // return (potValue - Math.PI) * Math.PI;
    // }

    // for more autonomous commands & working with limelight

    // Drops the piece at a low speed on the ground
    public void dropPiece() {
        mAnglePID.setSetpoint(ShooterConstants.kDropAngle);
        mShooterRightPID.setSetpoint(ShooterConstants.kFeedSpeed);
        mShooterLeftPID.setSetpoint(ShooterConstants.kFeedSpeed);
        mShooterRunning = true;
    }

    public void runTrigger() {
        mShooterTriggerMotor.setVoltage(ShooterConstants.kShooterTriggerSpeed);
    }

    public void stopTrigger() {
        mShooterTriggerMotor.setVoltage(0.0);
    }

    public void load() {
        mAnglePID.setSetpoint(16);
    }



    public void runMotors() {
        double leftVoltage = MathUtil.clamp(mShooterLeftFeedFwd.calculate(mShooterLeftPID.getSetpoint()) + mShooterLeftPID.calculate(mShooterMotorLeft.getEncoder().getVelocity()), -12, 12);
        double rightVoltage = MathUtil.clamp(mShooterRightFeedFwd.calculate(mShooterRightPID.getSetpoint()) + mShooterRightPID.calculate(mShooterMotorRight.getEncoder().getVelocity()), -12, 12);
        double angleVoltage = MathUtil.clamp(mAnglePID.calculate(getAbsoluteDistance(mEncoder)), -10, 10);
        if (mShooterRunning) {
            // mShooterMotorLeft.setVoltage(leftVoltage);
            // mShooterMotorRight.setVoltage(rightVoltage);
            mShooterMotorLeft.setVoltage(leftVoltage);
            mShooterMotorRight.setVoltage(rightVoltage);
        } else {
            stopShoot();
        }

        SmartDashboard.putNumber("Left RPM", mShooterMotorLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Right RPM", mShooterMotorRight.getEncoder().getVelocity());

        // mAngleMotor.setVoltage(angleVoltage);
        // System.out.println("Voltage Constant: " + voltageConstant);
        // System.out.println(voltageConstant * Math.cos(mEncoder.getDistance()));
        // System.out.println(mEncoder.getDistance());
        // System.out.println(voltageConstant);
        // System.out.println("PID: " + mAnglePID.getP() + " " + (angleVoltage +
        // voltageConstant) + " RPM: " + mShooterMotorLeft.getEncoder().getVelocity());
        mAngleMotor.setVoltage(angleVoltage);

    }

    public void tempSetFeed() {
        mShooterMotorLeft.setVoltage(-5.0);
        mShooterMotorRight.setVoltage(-5.0);
    }

    public void tempEndFeed() {
        mShooterMotorLeft.setVoltage(0.0);
        mShooterMotorRight.setVoltage(0.0);
    }

    // public boolean hasNote() {
    //     return mHasNote;
    // }

    // public void changeNoteStatus(boolean status) {
    //     mHasNote = status;
    // }

    public void canStow(boolean canIt) {
        mCanStow = canIt;
        if(canIt) {System.out.println("yuh");};
    }

    public double getAbsoluteDistance(DutyCycleEncoder mEncoder) {
        double distance = -1 * mEncoder.getDistance();
        while (distance < -180) {
            distance += 360;
        }
        while (distance > 180) {
            distance -= 360;
        }
        return distance;
    }

}
