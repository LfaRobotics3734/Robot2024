package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.AmpScorerConstants;

public class AmpScorer {
    CANSparkMax mFeedMotor = new CANSparkMax(AmpScorerConstants.kMotorID, MotorType.kBrushless);
    CANSparkMax mAngleMotor = new CANSparkMax(AmpScorerConstants.kAngleMotorID, MotorType.kBrushless);

    public AmpScorer() {
    }

    // rotate the motors fast enough to move the piece and score in the amp
    public void rotate() {
        mFeedMotor.setVoltage(AmpScorerConstants.kRotateSpeed);
    }

    public void stopRotate() {
        mFeedMotor.setVoltage(0.0);
    }

    // To be implemented
    public boolean moveToReady() {
        if(mAngleMotor.getOutputCurrent() > AmpScorerConstants.kMaxAngleMotorCurrent) {
            move(0);
            return true;
        } else {
            move(AmpScorerConstants.kAngleSpeed);
            return false;
        }
    }

    public void move(double moveSpeed) {
        mAngleMotor.setVoltage(moveSpeed);
    }

    // wow great class, does so much
    // thats bc you missed the amp scorer moving functionality :gasp:
}
