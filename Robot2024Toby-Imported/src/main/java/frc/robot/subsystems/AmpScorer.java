package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpScorerConstants;

public class AmpScorer extends SubsystemBase {
    CANSparkMax mFeedMotor = new CANSparkMax(AmpScorerConstants.kMotorID, MotorType.kBrushless);
    CANSparkMax mAngleMotor = new CANSparkMax(AmpScorerConstants.kAngleMotorID, MotorType.kBrushless);
    private double avgMoveSpeed = 0.0;

    public AmpScorer() {
        // Degrees of amp scorer rotation per second (approx.)
        mAngleMotor.setInverted(true);
        mAngleMotor.getEncoder().setVelocityConversionFactor((360.0 * 60.0) / 200.0);
        mAngleMotor.getEncoder().setMeasurementPeriod(64);
    }

    @Override
    public void periodic() {
        avgMoveSpeed = avgMoveSpeed * .7 + mAngleMotor.getEncoder().getVelocity() * .3;
    }

    // rotate the motors fast enough to move the piece and score in the amp
    public void rotate() {
        mFeedMotor.setVoltage(AmpScorerConstants.kRotateSpeed);
    }

    public void stopRotate() {
        mFeedMotor.setVoltage(0.0);
    }

    // To be implemented
    // public boolean moveToReady() {
    //     if(mAngleMotor.getOutputCurrent() > AmpScorerConstants.kMaxAngleMotorCurrent) {
    //         move(0);
    //         return true;
    //     } else {
    //         move(AmpScorerConstants.kAngleSpeed);
    //         return false;
    //     }
    // }

    public void move(double movePowah) {
        avgMoveSpeed = 10.0;
        System.out.println("here");
        mAngleMotor.set(movePowah);
    }

    public boolean reachedStop() {
        System.out.println(avgMoveSpeed);

        return avgMoveSpeed < AmpScorerConstants.kAngularVelocityThreshold;
    }

    // public void move(double moveSpeed) {
    //     mAngleMotor.setVoltage(moveSpeed);
    // }

    // wow great class, does so much
    // thats bc you missed the amp scorer moving functionality :gasp:
}
