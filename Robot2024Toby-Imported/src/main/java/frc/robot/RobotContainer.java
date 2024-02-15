// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

/*import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;*/

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IO;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.AmpScorer;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive m_robotDrive;
  private Shooter shooter = new Shooter();
  private AmpScorer ampScorer = new AmpScorer();
  private Intake intake = new Intake();

  /*private List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue" + Constants.Autonomous.autoPath,
      new PathConstraints(3, 2.5)); // 3, 2.5

  SwerveAutoBuilder autoBuilder;

  Limelight limelight = new Limelight(m_robotDrive);
  Autos autonomous;*/

  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick m_driverController = new Joystick(IO.driveController);
  XboxController m_armController = new XboxController(IO.armController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // LL port forwarding
    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    SmartDashboard.putNumber("translation-pid", Constants.Autonomous.TRANSLATION_PID);
    SmartDashboard.putNumber("rotation-pid", Constants.Autonomous.ROTATION_PID);

    // Read initial pose
    // REMINDER: get initial pose
    // Pose2d initialPose = readInitialPose("/");
    m_robotDrive = new SwerveDrive();
    /*autonomous = new Autos(m_robotDrive, limelight, arm, claw, pathGroup);
    autoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose, // Pose2d supplier
        m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants(Constants.Autonomous.TRANSLATION_PID, 0, 0), // between 10.9 and 11
        // PID controllers)
        new PIDConstants(Constants.Autonomous.ROTATION_PID, 0, 0.2), // PID constants to correct for rotation error (used to create the rotation
                                       // controller)
        m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
        autonomous.getEventMap(),
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        m_robotDrive // The drive subsystem. Used to properly set the requirements of path following
                     // commands
    );*/

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), IO.kDriveDeadband), // add cube here
                -MathUtil.applyDeadband(m_driverController.getX(), IO.kDriveDeadband),
                m_driverController.getRawButton(2) ? -MathUtil.applyDeadband(m_driverController.getZ(), 0.4)
                    : ((m_driverController.getPOV() == 45 || m_driverController.getPOV() == 90
                        || m_driverController.getPOV() == 135)
                            ? -0.5
                            : (m_driverController.getPOV() == 225 || m_driverController.getPOV() == 270
                                || m_driverController.getPOV() == 315) ? 0.5 : 0)),
            m_robotDrive));
  }

  // controllers for person with the Xbox
  private void configureBindings() {

    // intake the object when A is pressed
    Trigger leftTrigger = new Trigger(() -> m_armController.getLeftTriggerAxis() >= 0.75);
    Trigger rightTrigger = new Trigger(() -> m_armController.getRightTriggerAxis() >= 0.75);

    //Left trigger starts the intake + transition
    //may want to consider splitting this into two functions
    leftTrigger
        .whileTrue(new RunCommand(() -> {
          intake.grab();
          intake.transition();
        })).onFalse(new InstantCommand(() -> intake.stop(), intake));

     //Right trigger starts the shooter
     rightTrigger
     .whileTrue(new RunCommand(() -> {
       shooter.Shoot();
     })).onFalse(new InstantCommand(() -> shooter.stopShoot(), shooter));

    //Debugging
    /*rightBumper.onTrue(new InstantCommand(()->{
      
    }));*/
      /*new RunCommand(() -> {
      m_robotDrive.autoBalance(true);
    })*/

    //move shooter to feed position
    new JoystickButton(m_armController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          shooter.moveToFeed();
        }));

    //move shooter shoot position?
    //will likely remove later
    new JoystickButton(m_armController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          shooter.moveToShoot();
        }));

    //move intake to source
    (new JoystickButton(m_armController, XboxController.Button.kBack.value)).onTrue(new InstantCommand(() -> {
      intake.moveToSource();
    }));

    //move the intake to floor
    (new JoystickButton(m_armController, XboxController.Button.kStart.value)).onTrue(new InstantCommand(() -> {
      intake.moveToFloor();
    }));

    //move shooter up manually
    (new JoystickButton(m_armController, XboxController.Button.kLeftStick.value)).whileTrue(new RunCommand(() -> {
      shooter.setElbowOutput(-m_armController.getLeftY() * 0.3);
    })).onFalse(new InstantCommand(() -> {
      shooter.setSetpoints();
    }));
    
    //move intake up manually
    (new JoystickButton(m_armController, XboxController.Button.kRightStick.value)).whileTrue(new RunCommand(() -> {
      intake.setPinionOutput(m_armController.getRightY() * 0.3);
    })).onFalse(new InstantCommand(() -> {
      intake.setSetpoint();
    }));

    Trigger dPadDown = new Trigger(
        () -> m_armController.getPOV() == 135 || m_armController.getPOV() == 180 || m_armController.getPOV() == 225);
    Trigger dPadUp = new Trigger(
        () -> m_armController.getPOV() == 315 || m_armController.getPOV() == 0 || m_armController.getPOV() == 45);


    //reset the gyro to reset field orientation
    (new JoystickButton(m_driverController, 1)).onTrue(new InstantCommand(() -> {
      resetGyro();
    }));

    // Change Gears to low gear
    (new JoystickButton(m_driverController, 3)).onTrue(new InstantCommand(() -> {
      m_robotDrive.switchGear(Constants.Drive.lowGear);
    }));

    // Change gears to high gear
    (new JoystickButton(m_driverController, 5)).onTrue(new InstantCommand(() -> {
      m_robotDrive.switchGear(Constants.Drive.highGear);
    }));
  }

  //reset the gyrometer to zero deg
  public void resetGyro() {
    m_robotDrive.zeroHeading();
  }


  //blue 1 and blue 3
  /*public Command getAutonomousCommand() {
    //Changed: new SwerveAutoBuilder here, path at beginning, smart dashboard, undo all
    
    SequentialCommandGroup seq = new SequentialCommandGroup();
    seq.addCommands(
        new InstantCommand(() -> {
          m_robotDrive.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());
        }),
        // Move arm down to read limelight, then move arm up
        new InstantCommand(() -> arm.quickCubeAngle()),
        new WaitUntilCommand(() -> arm.reached()),
        new InstantCommand(() -> claw.releaseObjectAuto()), 
        new WaitCommand(1),
        new InstantCommand(() -> claw.stop()),
        // Go to Initial Point
        new ProxyCommand(() -> new SequentialCommandGroup(
            autonomous.toPosition(new Pose2d(m_robotDrive.getPose().getX() + 0.75, m_robotDrive.getPose().getY() + 0.25,
                Rotation2d.fromDegrees(180)), false),
            new ParallelDeadlineGroup(new SequentialCommandGroup(
                new InstantCommand(() -> arm.moveToFloor()),
                new WaitUntilCommand(() -> arm.reached()),
                returnHome()), new RunCommand(() -> m_robotDrive.addVision())),
            new ProxyCommand(() -> new SequentialCommandGroup(
                autonomous.toPosition(pathGroup.get(0).getInitialPose(), true),
                autoBuilder.fullAuto(pathGroup)
               // returnHomeElbowFirst()
                )))));
    return seq;
  }*/

  //blue 2: middle path with the autobalance path.
  public Command getAutoBalance() {
    SequentialCommandGroup seq = new SequentialCommandGroup();
    seq.addCommands(
        new InstantCommand(() -> m_robotDrive.zeroHeading()),
        // release cube
        /*new InstantCommand(() -> arm.quickCubeAngle()),
        new WaitUntilCommand(() -> arm.reached()),
        new InstantCommand(() -> claw.releaseObject()),
        new WaitCommand(1),
        new InstantCommand(() -> claw.stop()),
        returnHome(),*/
        // move backwards 5 seconds
        new RunCommand(() -> m_robotDrive.drive(-0.3, 0, 0)).until(() -> m_robotDrive.overTheThaang()),
        // move forward until angle BangBang
        new ProxyCommand(()->new SequentialCommandGroup(
            new RunCommand(() -> m_robotDrive.drive(0.3, 0, 0)).until(()->{
              System.out.println("Angle on the way back: " + m_robotDrive.getAngle());
              return Math.abs(m_robotDrive.getAngle()) > 5;
            }),
            // autobalance
            new RunCommand(() -> m_robotDrive.autoBalance(false), m_robotDrive)
        ))
      );
    return seq;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * return new RunCommand(null, null);
   * }
   */
  //supplier for rotation in drive function
  public double getRotation() {
    if (m_driverController.getPOV() == 90) {
      return -0.4;
    } else if (m_driverController.getPOV() == 270) {
      return 0.4;
    }
    return 0.0;
  }
}
