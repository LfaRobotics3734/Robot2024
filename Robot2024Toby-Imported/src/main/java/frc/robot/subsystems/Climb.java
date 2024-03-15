// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private CANSparkMax mMotor = new CANSparkMax(ClimbConstants.kMotorID, MotorType.kBrushless);
    private double mDistanceTraveled = 0.0;
    /** Creates a new Climb. */
    public Climb() {
        mMotor.setInverted(true);
        mMotor.getEncoder().setPosition(0);
        mMotor.getEncoder().setPositionConversionFactor(ClimbConstants.kPositionConversionFactor);
        // mMotor.getEncoder().setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runClimb(double output) {
        if(mMotor.getEncoder().getPosition() < 0.0) {
            mMotor.set(MathUtil.clamp(output, -.25, 1.0));
        // } else if(mMotor.getEncoder().getPosition() > ClimbConstants.kMaxExtension) {
        //     mMotor.set(MathUtil.clamp(output, -1.0, 0.0));
        } else {
            mMotor.set(output);
        }
        System.out.println(mMotor.getEncoder().getPosition());
    }

    public void stopClimb() {
        mMotor.set(0.0);
    }
}
