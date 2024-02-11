package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
   
    PIDController pid = new PIDController(,,);

    CANSparkMax intakeMotor = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax transitionMotor = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax pinionMotor = new CANSparkMax(, MotorType.kBrushless);


    public Intake(){

    }

    //lower to pinions at the start of the match to the resting position
    public void lower(){
        pinionMotor.setVoltage();
        //stop the arm once it encounters resistance - idk how
    }

    //run motor at the front of the intake to "grab" the piece
    public void grab(){
        intakeMotor.setVoltage();
    }

    public void stopGrab() {
        intakeMotor.setVoltage(0.0);
    }

    //run the motors to transition the piece from 
    public void transition(){
        transitionMotore.setVoltage();
    }

    public void stopTransition() {
        transitionMotore.setVoltage(0.0);
    }
}
