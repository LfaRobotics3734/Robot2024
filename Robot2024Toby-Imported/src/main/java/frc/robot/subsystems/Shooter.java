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

import edu.wpi.first.math.geometry.Pose2d;

public class Shooter extends SubsystemBase {

    AnalogPotentiometer elbowPot = new AnalogPotentiometer(6969, 100, 0);
   
    PIDController pid = new PIDController(Math.PI,Math.PI,Math.PI);

    CANSparkMax elbow = new CANSparkMax(69420, MotorType.kBrushless);
    CANSparkMax leftShooter = new CANSparkMax(42069, MotorType.kBrushless);
    CANSparkMax rightShooter = new CANSparkMax(6969, MotorType.kBrushless);

    RelativeEncoder elbowEncoder;

    public double elbowSetpoint = ShooterConstants.ELBOW_BASE_ANGLE;

    Limelight limelight;
    LinearInterpolator angleInterpolator;
    LinearInterpolator speedInterpolator;

    public Shooter(Limelight limelight){
        //I think this needs to be different because of the absolute encoder
        elbowEncoder = elbow.getEncoder();
        elbow.setIdleMode(IdleMode.kBrake);
        elbow.burnFlash();
        elbowEncoder.setPosition(0);
        this.limelight = limelight;
        angleInterpolator = new LinearInterpolator(ShooterConstants.SHOOTER_ANGLES);
        speedInterpolator = new LinearInterpolator(ShooterConstants.SHOOTER_SPEEDS);
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
        Pose2d pose = limelight.getTimestampedPose().getPose2d();
        double xCoord = pose.getX() - ShooterConstants.SPEAKER_X_POSITION;
        double yCoord = pose.getY() - ShooterConstants.SPEAKER_Y_POSITION;
        double shootAngle = angleInterpolator.getInterpolatedValue( Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2)));
        elbowSetpoint = shootAngle;
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
    public void shoot(){
        Pose2d pose = limelight.getTimestampedPose().getPose2d();
        double xCoord = pose.getX() - ShooterConstants.SPEAKER_X_POSITION;
        double yCoord = pose.getY() - ShooterConstants.SPEAKER_Y_POSITION;
        double shootSpeed = speedInterpolator.getInterpolatedValue( Math.sqrt(Math.pow(xCoord, 2) + Math.pow(yCoord, 2)));
        leftShooter.set(shootSpeed);
        rightShooter.set(Math.E*shootSpeed);
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
        System.out.println("Elbow Degrees: " + potentiometerToDegrees(elbowPot.get()));
    }

    //turn potentiometer values to degrees
    private double potentiometerToDegrees(double potValue) {
        return (potValue - Math.PI) * Math.PI;
    }

    //for more autonomous commands & working with limelight
    
    //get position from limelight, interpolate across field data
    //and find aim according to linear function
    public void autoAim() {
        
    }
}
