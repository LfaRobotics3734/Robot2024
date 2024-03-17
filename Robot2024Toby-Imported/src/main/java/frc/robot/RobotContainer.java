// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IO;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AmpScorer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrive;
import frc.utils.AllianceFlipUtil;

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
	private Limelight limelight = new Limelight(mRobotDrive);
	private Shooter shooter;
	private AmpScorer ampScorer = new AmpScorer();
	private Intake intake = new Intake();
	private Climb climb = new Climb();

	// private List<PathPlannerTrajectory> pathGroup =
	// PathPlanner.loadPathGroup("Blue" + Constants.Autonomous.autoPath,
	// new PathConstraints(3, 2.5)); // 3, 2.5

	// SwerveAutoBuilder autoBuilder;

	// Limelight limelight = new Limelight(mRobotDrive);
	// Autos autonomous;

	// Replace with CommandPS4Controller or CommandJoystick if needed
	// weird joystick thing
	CommandJoystick mDriverController = new CommandJoystick(IO.kDriverControllerPort);
	// the classic controller
	CommandXboxController mOperatorController = new CommandXboxController(IO.kOperatorControllerPort);

	// private double flipConstant;
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		mRobotDrive = new SwerveDrive(limelight, new Pose2d(0, 0, new Rotation2d()));
		shooter = new Shooter(limelight, mRobotDrive.getPoseEstimator());

		// Configure named commands for autonomous
		registerCommands();

		// Configure the trigger bindings
		configureBindings();
		// forward ports 5800-5807 for limelight
		for (int port = 5800; port <= 5807; port++) {
			PortForwarder.add(port, "limelight.local", port);
		}

		// SmartDashboard.putNumber("translation-pid",
		// Constants.Autonomous.TRANSLATION_PID);
		// SmartDashboard.putNumber("rotation-pid", Constants.Autonomous.ROTATION_PID);

		// Read initial pose
		// REMINDER: get initial pose
		// Pose2d initialPose = readInitialPose("/");
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
		
		
		// Using flight joystick
		mRobotDrive.setDefaultCommand(
				// Using flight joystick
				new RunCommand(
						() -> {
							mRobotDrive.drive(
									-MathUtil.applyDeadband(
											mDriverController
													.getY(),
											IO.kDriveDeadband),
									-MathUtil.applyDeadband(
											mDriverController
													.getX(),
											IO.kDriveDeadband),
									mDriverController.getHID().getRawButton(2)
											? -MathUtil.applyDeadband(
													mDriverController
															.getZ(),
													0.4)
											: ((mDriverController.getHID()
													.getPOV() == 45
													|| mDriverController
															.getHID()
															.getPOV() == 90
													|| mDriverController
															.getHID()
															.getPOV() == 135)
																	? -0.5
																	: (mDriverController
																			.getHID()
																			.getPOV() == 225
																			|| mDriverController
																					.getHID()
																					.getPOV() == 270
																			|| mDriverController
																					.getHID()
																					.getPOV() == 315)
																							? 0.5
																							: 0));
						},
						mRobotDrive));

		// InstantCommand x = new InstantCommand(() -> System.out.println("Grauh"));
		// InstantCommand y = new PrintCommand("bruh2");
		// SmartDashboard.putData("Test2", y);
		// SmartDashboard.putData("Test", new PrintCommand("bruh."));

		AutoBuilder.configureHolonomic(
				mRobotDrive::getPose,
				mRobotDrive::resetOdometry,
				mRobotDrive::getRobotRelativeSpeeds,
				mRobotDrive::driveRobotRelative,
				new HolonomicPathFollowerConfig(
						new PIDConstants(5),
						new PIDConstants(5),
						DriveConstants.maxSpeed,
						Math.sqrt(2 * Math.pow(DriveConstants.baseDimensions / 2, 2)), // 0.4681046891
						new ReplanningConfig()),
				AllianceFlipUtil::shouldFlip,
				mRobotDrive);

				

	}

	public Command getAutonomousCommand() {
		return new ParallelCommandGroup(
			new PathPlannerAuto("2-Piece Top Start")
		, new StartEndCommand(() -> {
				ampScorer.move(.30);
			}, () -> {
				System.out.println("wahoo");
				ampScorer.move(0.0);
			}, ampScorer) {
				@Override
				public boolean isFinished() {
					return ampScorer.reachedStop();
				}
		});
	}

	public Command getTeleopInitCommand() {
		return new InstantCommand(() -> {
			intake.moveToFloor();
			intake.stopIntake();
			intake.stopIndexer();

			shooter.stow();
			shooter.stopShoot();
			shooter.stopTrigger();

			// RED FLIPPING ISSUE
			// if(AllianceFlipUtil.shouldFlip()) {
			// 	mRobotDrive.resetOdometry(mRobotDrive.getPose().rotateBy(Rotation2d.fromDegrees(180)));
			// }
		}, shooter, intake);
	}

	private void registerCommands() {
		// Autotargets, shoots, and stops all running motors
		NamedCommands.registerCommand("takeShot", new SequentialCommandGroup(new RunCommand(() -> {
			shooter.autotarget(mRobotDrive.getPose(), mRobotDrive.getFieldRelativeChassisSpeeds());
			mRobotDrive.autotargetJustRotate();
		}, shooter, mRobotDrive).withTimeout(1.75),
		new InstantCommand(() -> {
			System.out.println("we here");
			shooter.runTrigger();
			intake.runIndexer();
		}, shooter, intake),
		new WaitCommand(1),
		new InstantCommand(() -> {
			shooter.stopShoot();
			shooter.stopTrigger();
			intake.stopIndexer();
		}, shooter, intake)));

		// Takes subwoofer shot with no autoaim
		NamedCommands.registerCommand("takeSubwooferShot", new SequentialCommandGroup(
			new InstantCommand(shooter::subwooferShot, shooter), 
			new WaitCommand(1), 
			new InstantCommand(() -> {
				shooter.runTrigger();
				intake.runIndexer();
			}, shooter, intake),
			new WaitCommand(1),
			new InstantCommand(() -> {
			shooter.stopShoot();
			shooter.stopTrigger();
			intake.stopIndexer();
		}, shooter, intake)));

		// NamedCommands.registerCommand("takeShot", takeShot);

		// ? load ammo while moving
		NamedCommands.registerCommand("movingIntake", new StartEndCommand(() -> {
			intake.runIntake();
			shooter.runTrigger();
			shooter.load();
		}, () -> {
			intake.stopIndexer();
			shooter.stopTrigger();
		}, intake, shooter) {
			@Override
			public boolean isFinished() {
				return shooter.getTripStatus().getAsBoolean();
			}
		});

		NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> {
			intake.stopIntake();
			intake.stopIndexer();
			// shooter.stow();
		}, intake, shooter));
	}

	// controllers for operator
	private void configureBindings() {

		mOperatorController.rightTrigger(ControlConstants.kTriggerDeadband)
				.onTrue(new InstantCommand(() -> {
					shooter.runTrigger();
					intake.runIndexer();
					// shooter.canStow(false);
				}, shooter))
				.onFalse(new InstantCommand(() -> {
					shooter.stopTrigger();
					intake.stopIndexer();
				}, shooter, intake));

		// ///! To be implemented
		// ? Panic mode
		mOperatorController.leftTrigger(ControlConstants.kTriggerDeadband)
				.onTrue(new InstantCommand(() -> {
					shooter.panic();
					intake.panic();
				}, intake, shooter));

		// ? Run intake
		// * Indexer and trigger will be stopped with IR trip sensor at shooter
		mOperatorController.a().onTrue(new InstantCommand(() -> {
			intake.runIntake();
			shooter.runTrigger();
			// shooter.load();
		}, intake, shooter))
				.onFalse(new InstantCommand(() -> {
					intake.stopIntake();
					intake.stopIndexer();
					shooter.stopTrigger();
				}, intake, shooter));

		// ? Stop index
		new Trigger(shooter.getTripStatus()).and(RobotModeTriggers.autonomous().negate())
				.onTrue(new WaitCommand(ShooterConstants.kTripDelay).andThen(new InstantCommand(() -> {
					intake.stopIndexer();
					shooter.stopTrigger();
					// if(!shooter.hasNote()) {
					shooter.stow();
					System.out.println("h");
					// shooter.changeNoteStatus(true);
					// }
				}, intake, shooter)));

		// ? Manual stop index
		mOperatorController.povRight().onTrue(new InstantCommand(() -> {
			intake.stopIndexer();
			shooter.stopTrigger();
		}));

		// ? Prep amp scorer (shoot with the trigger still)
		mOperatorController.x().onTrue(new InstantCommand(() -> {
			shooter.feed();
			ampScorer.rotate();
		}, shooter, ampScorer)).onFalse(new InstantCommand(() -> {
			// shooter.stow()
			shooter.stopThingsButNoStow();
			ampScorer.stopRotate();
		}, shooter, ampScorer));

		// ? Load and stow shooter
		mOperatorController.leftBumper().onTrue(new InstantCommand(shooter::load, shooter))
				.onFalse(new InstantCommand(shooter::stow, shooter));

		// ? Autotarget
		mOperatorController.b().onTrue(new InstantCommand(() -> {
			mRobotDrive.getPoseEstimator().setVisionMeasurementStdDevs(LimelightConstants.kTeleopStDevs);
		})).whileTrue(new RunCommand(() -> {
					shooter.autotarget(mRobotDrive.getPose(), mRobotDrive.getFieldRelativeChassisSpeeds());
					mRobotDrive.autotargetRotate(
							-MathUtil.applyDeadband(
									mDriverController
											.getY(),
									IO.kDriveDeadband),
							-MathUtil.applyDeadband(
									mDriverController
											.getX(),
									IO.kDriveDeadband));
				}, shooter, mRobotDrive))
				.onFalse(new InstantCommand(() -> {
					shooter.stow();
					mRobotDrive.getPoseEstimator().setVisionMeasurementStdDevs(LimelightConstants.kAutonomousStDevs);
				}, shooter));

		// ? Drop game piece
		mOperatorController.rightBumper().onTrue(new InstantCommand(shooter::dropPiece, shooter))
				.onFalse(new InstantCommand(shooter::stow, shooter));

		// ? Manual subwoofer shot
		mOperatorController.y().onTrue(new InstantCommand(shooter::subwooferShot, shooter))
				.onFalse(new InstantCommand(shooter::stow, shooter));

		// ? Move to floor intake position
		// * (on the hard stop)
		mOperatorController.povDown().or(mOperatorController.povDownLeft())
				.onTrue(new InstantCommand(intake::moveToFloor, intake));

		// ? Move to source intake position
		mOperatorController.povLeft().onTrue(new InstantCommand(intake::moveToSource, intake));

		// ? Move to retracted position
		mOperatorController.povUp().or(mOperatorController.povUpLeft())
				.onTrue(new InstantCommand(intake::moveToRetracted, intake));

		// ? Stob climbing
		mOperatorController.leftStick().whileTrue(new RunCommand(() -> {
			climb.runClimb(-1 * mOperatorController.getLeftY());
			System.out.println("here");
		}, climb)).onFalse(new InstantCommand(climb::stopClimb, climb));

		// ? reset the gyro to reset field orientation (hold)
		mDriverController.trigger().onTrue(new InstantCommand(mRobotDrive::setHeadingOffset))
				.onFalse(new InstantCommand(mRobotDrive::resetHeadingOffset));

		// ? Zero the gyro
		// mDriverController.button(11).onTrue(new InstantCommand(() -> {
		// 	mRobotDrive.resetOdometry(new Pose2d(mRobotDrive.getPose().getX(), mRobotDrive.getPose().getY(), 0.0)));
		// }, mRobotDrive);

		// ? Change to low gear
		mDriverController.button(3).onTrue(new InstantCommand(() -> mRobotDrive.switchGear(Constants.Drive.lowGear)));

		// ? Change to high gear
		mDriverController.button(5).onTrue(new InstantCommand(() -> mRobotDrive.switchGear(Constants.Drive.highGear)));
	}

	public SwerveDrive getDrive() {
		return mRobotDrive;
	}
}
