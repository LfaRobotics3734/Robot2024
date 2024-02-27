package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.AmpScorerConstants;

public class AmpScorer {
    CANSparkMax ampFeed = new CANSparkMax(AmpScorerConstants.kMotorID, MotorType.kBrushless);

    public AmpScorer() {
    }

    // rotate the motors fast enough to move the piece and score in the amp
    public void rotate() {
        ampFeed.setVoltage(AmpScorerConstants.kRotateSpeed);
    }

    public void stopRotate() {
        ampFeed.setVoltage(0.0);
    }

    // wow great class, does so much
}
