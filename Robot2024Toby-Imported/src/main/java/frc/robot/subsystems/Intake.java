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
   
    AnalogPotentiometer pinionPot = new AnalogPotentiometer(, 100, 0);

    PIDController pid = new PIDController(,,);

    CANSparkMax intakeMotor = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax transitionMotor = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax pinionMotor = new CANSparkMax(, MotorType.kBrushless);

    RelativeEncoder pinionEncoder;

    public double pinionSetpoint = IntakeConstants.ELBOW_BASE_ANGLE;

    public Intake(){
        pinionEncoder = pinionMotor.getEncoder();
        pinionMotor.setIdleMode(IdleMode.kBrake);
        pinionMotor.burnFlash();
        pinionEncoder.setPosition(0);
    }

    public void setPinionBase(){
        pinionSetpoint=IntakeConstants.PINION_BASE_ANGLE;
    }

    //lower the intake to the floor position
    public void moveToFloor(){
        pinionSetpoint = IntakeConstants.PINION_FLOOR_ANGLE;
    }

    //lower the intake to the human source position
    public void moveToSource(){
        pinionSetpoint = IntakeConstants.PINION_SOURCE_ANGLE;
    }

    //move to output
    public void setPinionOutput(double output) {
        output = MathUtil.clamp(output, -IntakeConstants.PINION_MAX_SPEED, IntakeConstants.PINION_MAX_SPEED);
        pinionMotor.set(output);
    }

    //idk what this does but it was in the arm code lmao
    public void setSetpoint() {
        pinionSetpoint = pinionPotentiometerToDegrees(pinionPot.get());
    }

    //if the arm has reached it's desired location
    public boolean reached(){
        return Math.abs(pinionPotentiometerToDegrees(pinionPot.get())-pinionSetpoint)<5&&Math.abs(pinionPotentiometerToDegrees(pinionPot.get())-pinionSetpoint)<6;
    }

    //Print potentiometer values for debugging
    public void logPotentiometerValues() {
        System.out.println("PinionMotor Degrees: " + pinionPotentiometerToDegrees(pinionPot.get()));
    }

    private double pinionPotentiometerToDegrees(double potValue) {
        return (potValue - ) * ;
    }

    //stop moving the pinion motor
    public void stopPinion() {
        pinionMotor.set(0.001);
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
        transitionMotor.setVoltage();
    }

    public void stopTransition() {
        transitionMotor.setVoltage(0.0);
    }
}
