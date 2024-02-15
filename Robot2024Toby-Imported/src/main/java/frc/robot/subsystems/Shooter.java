package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    AnalogPotentiometer elbowPot = new AnalogPotentiometer(, 100, 0);
   
    PIDController pid = new PIDController(,,);

    CANSparkMax elbow = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax leftShooter = new CANSparkMax(, MotorType.kBrushless);
    CANSparkMax rightShooter = new CANSparkMax(, MotorType.kBrushless);

    RelativeEncoder elbowEncoder;

    public double elbowSetpoint = ShooterConstants.ELBOW_BASE_ANGLE;

    public Shooter(){
        //I think this needs to be different because of the absolute encoder
        elbowEncoder = elbow.getEncoder();
        elbow.setIdleMode(IdleMode.kBrake);
        elbow.burnFlash();
        elbowEncoder.setPosition(0);
    }

    public void setElbowBase(){
        elbowSetpoint=ShooterConstants.ELBOW_BASE_ANGLE;
    }

    //lower the shooter to feed to amp scorer
    public void moveToFeed(){
        elbowSetpoint = ShooterConstants.ELBOW_FEED_ANGLE;
    }

    //lower the shooter to shoot.
    public void moveToShoot(){
        elbowSetpoint = ShooterConstants.ELBOW_SHOOT_ANGLE;
    }

    //get shooter angle
    public double getAngle(){
        return potentiometerToDegrees(elbowPot.get());
    }

    //set shooter angle to a specific angle
    //can be used in conjunction with joystick,
    //by getting shooter angle, adding a certain multiplier times the potentiometer reading
    //to change angle relative to joystick
    public void setAngle(double angle){
        elbowSetpoint = angle;
    }

    //move elbow to output
    public void setElbowOutput(double output) {
        output = MathUtil.clamp(output, -ShooterConstants.ELBOW_MAX_SPEED, ShooterConstants.ELBOW_MAX_SPEED);
        elbow.set(output);
    }

    //idk what this does it was in last year's arm code tho
    public void setSetpoints() {
        elbowSetpoint = potentiometerToDegrees(elbowPot.get());
    }

    //get left motor, & right motor up to speed - shoot after a second or two
    public void shoot(double leftMotorSpeed, double rightMotorSpeed){
        leftShooter.set(leftMotorSpeed);
        rightShooter.set(rightMotorSpeed);
    }

    //stop moving the elbow
    public void stopAngle() {
        elbow.set(0.001);
    }

    //stop the shooter from rotating
    public void stopShoot() {
        leftShooter.set(0.001);
        rightShooter.set(0.001);
    }

    //Print potentiometer values for debugging
    public void logPotentiometerValues() {
        System.out.println("Elbow Degrees: " + shoulderPotentiometerToDegrees(shoulderPot.get()));
    }

    //turn potentiometer values to degrees
    private double potentiometerToDegrees(double potValue) {
        return (potValue - ) * ;
    }

    //for more autonomous commands & working with limelight
    
    //get position from limelight, interpolate across field data
    //and find aim according to linear function
    public void autoAim() {
        
    }
}
