// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;*/

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.IO;
import frc.robot.subsystems.AmpScorer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrive;

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

    private SwerveDrive mRobotDrive;
    private Limelight limelight;
    private Shooter shooter = new Shooter(limelight, mRobotDrive.getPoseEstimator());
    private AmpScorer ampScorer = new AmpScorer();
    private Intake intake = new Intake();

    /*
     * private List<PathPlannerTrajectory> pathGroup =
     * PathPlanner.loadPathGroup("Blue" + Constants.Autonomous.autoPath,
     * new PathConstraints(3, 2.5)); // 3, 2.5
     * 
     * SwerveAutoBuilder autoBuilder;
     * 
     * Limelight limelight = new Limelight(mRobotDrive);
     * Autos autonomous;
     */

    // Replace with CommandPS4Controller or CommandJoystick if needed
    CommandJoystick mDriverController = new CommandJoystick(IO.driveController);
    CommandXboxController mOperatorController = new CommandXboxController(IO.armController);

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
        mRobotDrive = new SwerveDrive();
        /*
         * autonomous = new Autos(mRobotDrive, limelight, arm, claw, pathGroup);
         * autoBuilder = new SwerveAutoBuilder(
         * mRobotDrive::getPose, // Pose2d supplier
         * mRobotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at
         * the beginning of auto
         * Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
         * new PIDConstants(Constants.Autonomous.TRANSLATION_PID, 0, 0), // between 10.9
         * and 11
         * // PID controllers)
         * new PIDConstants(Constants.Autonomous.ROTATION_PID, 0, 0.2), // PID constants
         * to correct for rotation error (used to create the rotation
         * // controller)
         * mRobotDrive::setModuleStates, // Module states consumer used to output to
         * the drive subsystem
         * autonomous.getEventMap(),
         * true, // Should the path be automatically mirrored depending on alliance
         * color.
         * // Optional, defaults to true
         * mRobotDrive // The drive subsystem. Used to properly set the requirements of
         * path following
         * // commands
         * );
         */

        mRobotDrive.setDefaultCommand(
                // Using flight joystick
                new RunCommand(
                        () -> {

                            mRobotDrive.drive(
                                    -MathUtil.applyDeadband(mDriverController.getY(), IO.kDriveDeadband),
                                    -MathUtil.applyDeadband(mDriverController.getX(), IO.kDriveDeadband),
                                    mDriverController.getHID().getRawButton(2)
                                            ? -MathUtil.applyDeadband(mDriverController.getZ(), 0.4)
                                            : ((mDriverController.getHID().getPOV() == 45
                                                    || mDriverController.getHID().getPOV() == 90
                                                    || mDriverController.getHID().getPOV() == 135) ? -0.5
                                                            : (mDriverController.getHID().getPOV() == 225
                                                                    || mDriverController.getHID().getPOV() == 270
                                                                    || mDriverController.getHID().getPOV() == 315) ? 0.5
                                                                            : 0));
                        },
                        mRobotDrive));

        SmartDashboard.putData("Reset Intake Encoder", new InstantCommand(() -> intake.resetEncoder()));
    }

    // controllers for operator
    private void configureBindings() {

        // Left trigger starts the intake + transition
        // may want to consider splitting this into two functions
        // mOperatorController
        // .whileTrue(new InstantCommand(() -> {
        // intake.runIntake();
        // })).onFalse(new InstantCommand(() -> intake.stopIntake(), intake));

        // Right trigger starts the shooter

        mOperatorController.rightTrigger(ControlConstants.kTriggerDeadband)
                .onTrue(new InstantCommand(() -> {
                    shooter.runTrigger();
                    // ampScorer.rotate();
                })).onFalse(new InstantCommand(() -> shooter.stopTrigger(), shooter));

        // Debugging
        /*
         * rightBumper.onTrue(new InstantCommand(()->{
         * 
         * }));
         */
        /*
         * new RunCommand(() -> {
         * mRobotDrive.autoBalance(true);
         * })
         */

        // Panic mode
        // To be implemented
        mOperatorController.leftTrigger(ControlConstants.kTriggerDeadband)
                .onTrue(new InstantCommand(() -> {

                }));

        // move shooter shoot position?
        // will likely remove later

        mOperatorController.leftBumper()
                .onTrue(new InstantCommand(() -> {
                    shooter.subwooferShot();
                }))
                .onFalse(new InstantCommand(() -> {
                    shooter.stow();
                }));


        // Poop game piece
        // To be implemented
        // mOperatorController.rightBumper();
        




    //     new JoystickButton(mOperatorController, XboxController.Button.kY.value)
    //             .onTrue(new InstantCommand(() -> {
    //                 shooter.moveToShoot();
    //             }));

    //     // move intake to source
    //     (new JoystickButton(mOperatorController, XboxController.Button.kBack.value)).onTrue(new InstantCommand(() -> {
    //         intake.moveToSource();
    //     }));

    //     // move the intake to floor
    //     (new JoystickButton(mOperatorController, XboxController.Button.kStart.value)).onTrue(new InstantCommand(() -> {
    //         intake.moveToFloor();
    //     }));

    //     // move shooter up manually
    //     (new JoystickButton(mOperatorController, XboxController.Button.kLeftStick.value))
    //             .whileTrue(new RunCommand(() -> {
    //                 shooter.setElbowOutput(-mOperatorController.getLeftY() * 0.3);
    //             })).onFalse(new InstantCommand(() -> {
    //                 shooter.setSetpoints();
    //             }));

    //     // move intake up manually
    //     (new JoystickButton(mOperatorController, XboxController.Button.kRightStick.value))
    //             .whileTrue(new RunCommand(() -> {
    //                 intake.setPinionOutput(mOperatorController.getRightY() * 0.3);
    //             })).onFalse(new InstantCommand(() -> {
    //                 intake.setSetpoint();
    //             }));

    //     Trigger dPadDown = new Trigger(
    //             () -> mOperatorController.getPOV() == 135 || mOperatorController.getPOV() == 180
    //                     || mOperatorController.getPOV() == 225);
    //     Trigger dPadUp = new Trigger(
    //             () -> mOperatorController.getPOV() == 315 || mOperatorController.getPOV() == 0
    //                     || mOperatorController.getPOV() == 45);

    //     // reset the gyro to reset field orientation
    //     (new JoystickButton(mDriverController, 1)).onTrue(new InstantCommand(() -> {
    //         resetGyro();
    //     }));

    //     // Change Gears to low gear
    //     (new JoystickButton(mDriverController, 3)).onTrue(new InstantCommand(() -> {
    //         mRobotDrive.switchGear(Constants.Drive.lowGear);
    //     }));

    //     // Change gears to high gear
    //     (new JoystickButton(mDriverController, 5)).onTrue(new InstantCommand(() -> {
    //         mRobotDrive.switchGear(Constants.Drive.highGear);
    //     }));
    }

    // reset the gyrometer to zero deg
    public void resetGyro() {
        mRobotDrive.zeroHeading();
    }

    // blue 1 and blue 3
    /*
     * public Command getAutonomousCommand() {
     * //Changed: new SwerveAutoBuilder here, path at beginning, smart dashboard,
     * undo all
     * 
     * SequentialCommandGroup seq = new SequentialCommandGroup();
     * seq.addCommands(
     * new InstantCommand(() -> {
     * mRobotDrive.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());
     * }),
     * // Move arm down to read limelight, then move arm up
     * new InstantCommand(() -> arm.quickCubeAngle()),
     * new WaitUntilCommand(() -> arm.reached()),
     * new InstantCommand(() -> claw.releaseObjectAuto()),
     * new WaitCommand(1),
     * new InstantCommand(() -> claw.stop()),
     * // Go to Initial Point
     * new ProxyCommand(() -> new SequentialCommandGroup(
     * autonomous.toPosition(new Pose2d(mRobotDrive.getPose().getX() + 0.75,
     * mRobotDrive.getPose().getY() + 0.25,
     * Rotation2d.fromDegrees(180)), false),
     * new ParallelDeadlineGroup(new SequentialCommandGroup(
     * new InstantCommand(() -> arm.moveToFloor()),
     * new WaitUntilCommand(() -> arm.reached()),
     * returnHome()), new RunCommand(() -> mRobotDrive.addVision())),
     * new ProxyCommand(() -> new SequentialCommandGroup(
     * autonomous.toPosition(pathGroup.get(0).getInitialPose(), true),
     * autoBuilder.fullAuto(pathGroup)
     * // returnHomeElbowFirst()
     * )))));
     * return seq;
     * }
     */

    // blue 2: middle path with the autobalance path.
    // public Command getAutoBalance() {
    //     SequentialCommandGroup seq = new SequentialCommandGroup();
    //     seq.addCommands(
    //             new InstantCommand(() -> mRobotDrive.zeroHeading()),
    //             // release cube
    //             /*
    //              * new InstantCommand(() -> arm.quickCubeAngle()),
    //              * new WaitUntilCommand(() -> arm.reached()),
    //              * new InstantCommand(() -> claw.releaseObject()),
    //              * new WaitCommand(1),
    //              * new InstantCommand(() -> claw.stop()),
    //              * returnHome(),
    //              */
    //             // move backwards 5 seconds
    //             new RunCommand(() -> mRobotDrive.drive(-0.3, 0, 0)).until(() -> mRobotDrive.overTheThaang()),
    //             // move forward until angle BangBang
    //             new ProxyCommand(() -> new SequentialCommandGroup(
    //                     new RunCommand(() -> mRobotDrive.drive(0.3, 0, 0)).until(() -> {
    //                         System.out.println("Angle on the way back: " + mRobotDrive.getAngle());
    //                         return Math.abs(mRobotDrive.getAngle()) > 5;
    //                     }),
    //                     // autobalance
    //                     new RunCommand(() -> mRobotDrive.autoBalance(false), mRobotDrive))));
    //     return seq;
    // }

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
    // supplier for rotation in drive function
    public double getRotation() {
        if (mDriverController.getHID().getPOV() == 90) {
            return -0.4;
        } else if (mDriverController.getHID().getPOV() == 270) {
            return 0.4;
        }
        return 0.0;
    }
}
