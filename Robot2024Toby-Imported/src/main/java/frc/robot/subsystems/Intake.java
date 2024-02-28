package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private DutyCycleEncoder mEncoder = new DutyCycleEncoder(IntakeConstants.kEncoderChannel);

    private PIDController mPID = new PIDController(IntakeConstants.kDefaultKP, IntakeConstants.kDefaultKI,
            IntakeConstants.kDefaultKD);
    private double mKP, mKI, mKD;

    private CANSparkMax mIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax mIndexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorID, MotorType.kBrushless);
    private CANSparkMax mAngleMotor = new CANSparkMax(IntakeConstants.kAngleMotorID, MotorType.kBrushless);

    // private double mSetpoint;

    private IntakeConstants.IntakePosition mCurrentPosition;

    public Intake() {
        loadPreferences();
        mEncoder.setDistancePerRotation(IntakeConstants.kDistancePerRotation);
        // mSetpoint = IntakeConstants.kRetratctedAngle;
        mCurrentPosition = IntakeConstants.IntakePosition.RETRACTED;

        // Sets PID controller tolerance for being at setpoint to 2 degrees
        mPID.setTolerance(2);

        // SmartDashboard.putNumber("Voltage Constant", 0.0);
        SmartDashboard.putNumber("Setpoint", 0.0);
        SmartDashboard.putNumber("kP", 0.0);
        SmartDashboard.putNumber("kI", 0.0);
        SmartDashboard.putNumber("kD", 0.0);
        SmartDashboard.putData("Update PID values", new InstantCommand(() -> updatePIDValues()) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        // SmartDashboard.putNumber("Angle")
        // new NetworkButton(intakeEnable).onTrue(new InstantCommand(mAngleController.))
        // SmartDashboard.putData
    }

    public void updatePIDValues() {
        mPID.setSetpoint(SmartDashboard.getNumber("Setpoint", 0.0));
        mKP = SmartDashboard.getNumber("kP", 0.0);
        mKI = SmartDashboard.getNumber("kI", 0.0);
        mKD = SmartDashboard.getNumber("kD", 0.0);
        mPID.setPID(mKP, mKI, mKD);
    }
    
    // Load constants from flash memory
    public void loadPreferences() {
        mKP = Preferences.getDouble(IntakeConstants.kKPKey, IntakeConstants.kDefaultKP);
        mKI = Preferences.getDouble(IntakeConstants.kKIKey, IntakeConstants.kDefaultKI);
        mKD = Preferences.getDouble(IntakeConstants.kKDKey, IntakeConstants.kDefaultKD);
        mPID.setPID(mKP, mKI, mKD);

        mEncoder.setPositionOffset(Preferences.getDouble(IntakeConstants.kEncoderOffsetKey, 0.0));
    }

    // Write current PID constants to RoboRIO flash
    // Should only be used when tuning PID controller
    public void writePIDConstants() {
        Preferences.setDouble(IntakeConstants.kKPKey, mKP);
        Preferences.setDouble(IntakeConstants.kKIKey, mKI);
        Preferences.setDouble(IntakeConstants.kKDKey, mKD);
    }

    @Override
    public void periodic() {

        moveToPosition();
        // System.out.println("bruh");
        SmartDashboard.putNumber("Angle", mEncoder.getDistance());
        SmartDashboard.getNumber("Voltage Constant", 0.0);
        // mAngleMotor.setVoltage(SmartDashboard.getNumber("Voltage Constant", 0.0));
        // System.out.println(mEncoder.getDistance());
    }

    // Sets motor encoder idle modes (should only need to be done once)
    public void setIdleModes() {
        mAngleMotor.setIdleMode(IdleMode.kBrake);
        mIntakeMotor.setIdleMode(IdleMode.kCoast);
        mIndexerMotor.setIdleMode(IdleMode.kBrake);

        mAngleMotor.burnFlash();
        mIntakeMotor.burnFlash();
        mIndexerMotor.burnFlash();
    }

    // Sets encoder zero point and writes offset to flash.
    // Zero point (degrees) should be at the floor intake position, touching the
    // hard stop.
    public void resetEncoder() {
        mEncoder.reset();
        Preferences.setDouble(IntakeConstants.kEncoderOffsetKey, mEncoder.getPositionOffset());
        System.out.println("Intake encoder reset. Offset: " + mEncoder.getPositionOffset());
    }

    // lower the intake to the floor position
    public void moveToFloor() {
        mPID.setSetpoint(IntakeConstants.kFloorAngle);
        mCurrentPosition = IntakeConstants.IntakePosition.FLOOR;
    }

    // lower the intake to the human source position
    public void moveToSource() {
        mPID.setSetpoint(IntakeConstants.kSourceAngle);
        mCurrentPosition = IntakeConstants.IntakePosition.SOURCE;
    }

    // Move the intake to retracted (within chassis perimeter) position
    public void moveToRetracted() {
        mPID.setSetpoint(IntakeConstants.kRetratctedAngle);
        mCurrentPosition = IntakeConstants.IntakePosition.RETRACTED;
    }

    // Run motors depending on current (intended) intake position
    public void runIntake() {
        if (mCurrentPosition == IntakeConstants.IntakePosition.FLOOR) {
            mIntakeMotor.setVoltage(IntakeConstants.kFloorIntakeSpeed);
            mIndexerMotor.setVoltage(IntakeConstants.kFloorIndexerSpeed);
        } else if (mCurrentPosition == IntakeConstants.IntakePosition.SOURCE) {
            mIntakeMotor.setVoltage(IntakeConstants.kSourceIntakeSpeed);
            mIndexerMotor.setVoltage(IntakeConstants.kSourceIndexerSpeed);
        } else if (mCurrentPosition == IntakeConstants.IntakePosition.RETRACTED) {
            mIntakeMotor.setVoltage(0);
        }
    }

    // Stops intake. Leaves indexer alone
    public void stopIntake() {
        mIntakeMotor.setVoltage(0);
    }

    // move to output
    // public void setPinionOutput(double output) {
    // output = MathUtil.clamp(output, -IntakeConstants.PINION_MAX_SPEED,
    // IntakeConstants.PINION_MAX_SPEED);
    // mAngleMotor.setVoltage(output);
    // }

    // // idk what this does but it was in the arm code lmao
    // public void setSetpoint() {
    // pinionSetpoint = pinionPotentiometerToDegrees(pinionPot.get());
    // }

    // Returns true if the intake is within two degrees of the setpoint
    // public boolean reached() {
    // return Math.abs(mEncoder.getDistance() - mSetpoint) < 2;
    // }

    // Print current measured intake angle for debugging
    public void logMeasuredAngle() {
        System.out.println("Intake Angle (degrees):" + mEncoder.getDistance());
    }

    // private double pinionPotentiometerToDegrees(double potValue) {
    // return (potValue - Math.PI) * Math.PI;
    // }

    // Stop the angle motor
    public void stopIntakeAngleMovement() {
        mAngleMotor.set(0);
    }

    // // run motor at the front of the intake to "grab" the piece
    // public void grab() {
    // mIntakeMotor.set(IntakeConstants.GRAB_SPEED);
    // }

    // public void stopGrab() {
    // mIntakeMotor.set(0.0);
    // }

    // // run the motors to transition the piece from
    // public void transition() {
    // mIndexerMotor.set(IntakeConstants.TRANS_SPEED);
    // }

    public void stopIndexer() {
        mIndexerMotor.set(0.0);
    }

    public void moveToPosition() {
        //IntakeConstants.kIntakeMinVoltage * Math.cos(Math.toRadians(mEncoder.getDistance()))
        double x = MathUtil.clamp(mPID.calculate(mEncoder.getDistance()), -12.0, 12.0);
        System.out.println("Angle:" + mEncoder.getDistance() + "Voltage: " + x + " Current: " + mAngleMotor.getOutputCurrent());
        mAngleMotor.setVoltage(x);
    }
}
