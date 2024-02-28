package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private DutyCycleEncoder mEncoder = new DutyCycleEncoder(ShooterConstants.kEncoderChannel);

    private SimpleMotorFeedforward mShooterFeedFwd = new SimpleMotorFeedforward(ShooterConstants.kKS, ShooterConstants.kKV, 0);
    // private SimpleMotorFeedforward mRightFeedFwd = new SimpleMotorFeedforward(ShooterConstants.kKS, ShooterConstants.kKV, 0);

    private PIDController mAnglePID = new PIDController(ShooterConstants.kDefaultAngleKP, ShooterConstants.kDefaultAngleKI,
            ShooterConstants.kDefaultAngleKI);
    private PIDController mShooterLeftPID = new PIDController(ShooterConstants.kDefaultShooterLeftKP,
            ShooterConstants.kDefaultShooterLeftKI, ShooterConstants.kDefaultShooterLeftKI);
    private PIDController mShooterRightPID = new PIDController(ShooterConstants.kDefaultShooterRightKP,
            ShooterConstants.kDefaultShooterRightKI, ShooterConstants.kDefaultShooterRightKI);

    double mAngleKP, mAngleKI, mAngleKD, mShooterLeftKP, mShooterLeftKI, mShooterLeftKD, mShooterRightKP, mShooterRightKI, mShooterRightKD;
    boolean mShooterRunning = false;

    private CANSparkMax mAngleMotor = new CANSparkMax(ShooterConstants.kAngleMotorID, MotorType.kBrushless);
    private CANSparkMax mShooterMotorLeft = new CANSparkMax(ShooterConstants.kShooterMotorLeftID, MotorType.kBrushless);
    private CANSparkMax mShooterMotorRight = new CANSparkMax(ShooterConstants.kShooterMotorRightID, MotorType.kBrushless);
    private CANSparkMax mShooterTriggerMotor = new CANSparkMax(ShooterConstants.kShooterTriggerMotorID, MotorType.kBrushless);

    private double voltageConstant = 0.0;

    // private Limelight limelight;
    private LinearInterpolator mAngleInterpolator;
    private LinearInterpolator mSpeedInterpolator;

    private SwerveDrivePoseEstimator mPoseEstimator;

    public Shooter(Limelight limelight, SwerveDrivePoseEstimator poseEstimator) {
        mShooterTriggerMotor.setInverted(true);
        
        // I think this needs to be different because of the absolute encoder
        // Yes it did
        //   - jamie :)
        mPoseEstimator = poseEstimator;

        loadPreferences();

        // this.limelight = limelight;
        mAngleInterpolator = new LinearInterpolator(ShooterConstants.SHOOTER_ANGLES);
        mSpeedInterpolator = new LinearInterpolator(ShooterConstants.SHOOTER_SPEEDS);

        mEncoder.setDistancePerRotation(ShooterConstants.kDistancePerRotation);

        // mShooterLeftKP = .0013;
        // mShooterLeftKD = .000082;
        // mShooterRightKP = .0013;
        // mShooterRightKD = .000082;
        // writePIDConstants();

        mAnglePID.setTolerance(1);
        mShooterLeftPID.setTolerance(50);
        mShooterRightPID.setTolerance(50);

        // PID tuning
        SmartDashboard.putData("Burn PID constants", new InstantCommand(() -> writePIDConstants()) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        SmartDashboard.putData("Update PID values", new InstantCommand(() -> updatePIDValues()) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        SmartDashboard.putNumber("Setpoint", 0.0);
        SmartDashboard.putNumber("kP", 0.0);
        SmartDashboard.putNumber("kI", 0.0);
        SmartDashboard.putNumber("kD", 0.0);
        SmartDashboard.putNumber("Voltage constant", 0.0);
        SmartDashboard.putNumber("Angle", mEncoder.getDistance());

    }

    @Override
    public void periodic() {
        runMotors();
        // double x = mLeftFeedFwd.calculate(mShooterLeftPID.getSetpoint()) + mShooterLeftPID.calculate(mShooterMotorLeft.getEncoder().getVelocity());
        // System.out.println(x);
        // mShooterMotorLeft.setVoltage(x);
        // System.out.println(mShooterLeftKP);
        SmartDashboard.putNumber("Angle", mShooterMotorLeft.getEncoder().getVelocity());
        voltageConstant =SmartDashboard.getNumber("Voltage constant", 0.0);
        // updatePIDValues();
    }

    public void updatePIDValues() {
        mAnglePID.setSetpoint(SmartDashboard.getNumber("Setpoint", 0.0));
        mAngleKP = SmartDashboard.getNumber("kP", 0.0);
        mAngleKI = SmartDashboard.getNumber("kI", 0.0);
        mAngleKD = SmartDashboard.getNumber("kD", 0.0);
        mAnglePID.setPID(mAngleKP, mAngleKI, mAngleKD);
    }

    public void writePIDConstants() {
        Preferences.setDouble(ShooterConstants.kShooterLeftKPKey, mShooterLeftKP);
        Preferences.setDouble(ShooterConstants.kShooterLeftKIKey, mShooterLeftKI);
        Preferences.setDouble(ShooterConstants.kShooterLeftKDKey, mShooterLeftKD);

        Preferences.setDouble(ShooterConstants.kShooterRightKPKey, mShooterRightKP);
        Preferences.setDouble(ShooterConstants.kShooterRightKIKey, mShooterRightKI);
        Preferences.setDouble(ShooterConstants.kShooterRightKDKey, mShooterRightKD);

        Preferences.setDouble(ShooterConstants.kAngleKPKey, mAngleKP);
        Preferences.setDouble(ShooterConstants.kAngleKIKey, mAngleKI);
        Preferences.setDouble(ShooterConstants.kAngleKDKey, mAngleKD);
    }

    public void loadPreferences() {
        mAngleKP = Preferences.getDouble(ShooterConstants.kAngleKPKey, ShooterConstants.kDefaultAngleKP);
        mAngleKI = Preferences.getDouble(ShooterConstants.kAngleKIKey, ShooterConstants.kDefaultAngleKI);
        mAngleKD = Preferences.getDouble(ShooterConstants.kAngleKDKey, ShooterConstants.kDefaultAngleKD);

        mShooterLeftKP = Preferences.getDouble(ShooterConstants.kShooterLeftKPKey, ShooterConstants.kDefaultShooterLeftKP);
        mShooterLeftKI = Preferences.getDouble(ShooterConstants.kShooterLeftKIKey, ShooterConstants.kDefaultShooterLeftKI);
        mShooterLeftKD = Preferences.getDouble(ShooterConstants.kShooterLeftKDKey, ShooterConstants.kDefaultShooterLeftKD);

        mShooterRightKP = Preferences.getDouble(ShooterConstants.kShooterRightKPKey, ShooterConstants.kDefaultShooterRightKP);
        mShooterRightKI = Preferences.getDouble(ShooterConstants.kShooterRightKIKey, ShooterConstants.kDefaultShooterRightKI);
        mShooterRightKD = Preferences.getDouble(ShooterConstants.kShooterRightKDKey, ShooterConstants.kDefaultShooterRightKD);
        
        mAnglePID.setPID(mAngleKP, mAngleKI, mAngleKD);
        mShooterLeftPID.setPID(mShooterLeftKP, mShooterLeftKI, mShooterLeftKD);
        mShooterRightPID.setPID(mShooterRightKP, mShooterRightKI, mShooterRightKD);

        mEncoder.setPositionOffset(Preferences.getDouble(ShooterConstants.kEncoderOffsetKey, 0.0));
    }

    // Sets encoder zero point and writes offset to flash.
    // Zero point (degrees) should be level with the ground
    public void resetEncoder() {
        mEncoder.reset();
        // mEncoder.setPositionOffset(mEncoder.getPositionOffset() + (16.0/360.0));
        Preferences.setDouble(ShooterConstants.kEncoderOffsetKey, mEncoder.getPositionOffset());
        logEncoderValues();
    }

    // public void setElbowBase() {
    //     elbowSetpoint = ShooterConstants.ELBOW_BASE_ANGLE;
    // }

    // // lower the shooter to feed to amp scorer
    // public void moveToFeed() {
    //     elbowSetpoint = ShooterConstants.ELBOW_FEED_ANGLE;
    // }

    // lower the shooter to shoot.
    // public void moveToShoot() {
    //     // Pose2d pose = limelight.getTimestampedPose().getPose2d();
    //     Pose2d pose = mPoseEstimator.getEstimatedPosition();
    //     double xCoord = pose.getX() - ShooterConstants.SPEAKER_X_POSITION;
    //     double yCoord = pose.getY() - ShooterConstants.SPEAKER_Y_POSITION;
    //     double shootAngle = mAngleInterpolator
    //             .getInterpolatedValue(Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2)));
    //     elbowSetpoint = shootAngle;
    // }

    // // get shooter angle
    // public double getAngle() {
    //     return potentiometerToDegrees(elbowPot.get());
    // }

    // set shooter angle to a specific angle
    // can be used in conjunction with joystick,
    // by getting shooter angle, adding a certain multiplier times the potentiometer
    // reading
    // to change angle relative to joystick
    // public void setAngle(double angle) {
    //     elbowSetpoint = angle;
    // }

    // move elbow to output
    // public void setElbowOutput(double output) {
    //     output = MathUtil.clamp(output, -ShooterConstants.ELBOW_MAX_SPEED, ShooterConstants.ELBOW_MAX_SPEED);
    //     elbow.set(output);
    // }

    // get left motor, & right motor up to speed - shoot after a second or two
    public void setShooter() {
        Pose2d pose = mPoseEstimator.getEstimatedPosition();

        double xCoord = pose.getX() - ShooterConstants.SPEAKER_X_POSITION;
        double yCoord = pose.getY() - ShooterConstants.SPEAKER_Y_POSITION;
        double shootSpeed = mSpeedInterpolator
                .getInterpolatedValue(Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2)));
        double shootAngle = mAngleInterpolator
                .getInterpolatedValue(Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2)));

        mAnglePID.setSetpoint(shootAngle);
        mShooterLeftPID.setSetpoint(shootSpeed);
        mShooterRightPID.setSetpoint(shootSpeed - ShooterConstants.kRightSpeedOffset);
    }

    public void subwooferShot() {
        mShooterLeftPID.setSetpoint(ShooterConstants.kSubwooferShotSpeed);
        mShooterRightPID.setSetpoint(ShooterConstants.kSubwooferShotSpeed - ShooterConstants.kRightSpeedOffset);
    }

    // stop moving the elbow
    // public void stopAngle() {
    //     elbow.set(0.001);
    // }

    public void stow() {
        stopShoot();
        mAnglePID.setSetpoint(ShooterConstants.kStowedAngle);
    }

    // stop the shooter from rotating
    public void stopShoot() {
        mShooterMotorLeft.set(0.0);
        mShooterMotorRight.set(0.0);
        // stopTrigger();
    }

    // Print encoder values for debugging
    public void logEncoderValues() {
        System.out.println("Elbow Degrees: " + mEncoder.getDistance());
    }

    // turn potentiometer values to degrees
    // private double potentiometerToDegrees(double potValue) {
    //     return (potValue - Math.PI) * Math.PI;
    // }

    // for more autonomous commands & working with limelight

    // get position from limelight, interpolate across field data
    // and find aim according to linear function
    public void autoAim() {

    }

    // Drops the piece at a low speed on the ground
    public void dropPiece() {
        mAnglePID.setSetpoint(ShooterConstants.kDropAngle);
        mShooterRightPID.setSetpoint(ShooterConstants.kFeedSpeed);
        mShooterLeftPID.setSetpoint(ShooterConstants.kFeedSpeed);
    }

    public void runTrigger() {
        mShooterTriggerMotor.setVoltage(ShooterConstants.kShooterTriggerSpeed);
    }

    public void stopTrigger() {
        mShooterTriggerMotor.setVoltage(0.0);
    }

    public void runMotors() {
        double leftVoltage = mShooterFeedFwd.calculate(mShooterLeftPID.getSetpoint()) + mShooterLeftPID.calculate(mShooterMotorLeft.getEncoder().getVelocity());
        double rightVoltage = mShooterFeedFwd.calculate(mShooterRightPID.getSetpoint()) + mShooterLeftPID.calculate(mShooterMotorRight.getEncoder().getVelocity());
        double angleVoltage = mAnglePID.calculate(mEncoder.getDistance());
        if(mShooterRunning) {
            mShooterMotorLeft.setVoltage(leftVoltage);
            mShooterMotorRight.setVoltage(rightVoltage);
        } else {
            stopShoot();
        }

        // mAngleMotor.setVoltage(angleVoltage);
        // System.out.println("Voltage Constant: " + voltageConstant);
        // System.out.println(voltageConstant * Math.cos(mEncoder.getDistance()));
        // System.out.println(mEncoder.getDistance());
        mAngleMotor.setVoltage(voltageConstant);

    }
}
