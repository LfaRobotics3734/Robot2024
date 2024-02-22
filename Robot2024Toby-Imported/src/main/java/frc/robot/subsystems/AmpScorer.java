package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class AmpScorer {
    
    PIDController pid = new PIDController(Math.PI,Math.PI,Math.PI);

    CANSparkMax ampFeed = new CANSparkMax(69420, MotorType.kBrushless);

    public AmpScorer() {

    }

    //rotate the motors fast enough to move the piece and score in the amp
    public void rotate() {
        ampFeed.set(ShooterConstants.ROTATE_SPEED);
    }

    public void stopRotate() {
        ampFeed.set(0.0);
    }

    //wow great class, does so much
}
