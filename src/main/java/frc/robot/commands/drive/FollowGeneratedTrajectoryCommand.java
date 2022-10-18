package frc.robot.commands.drive;

import java.util.ArrayList;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import frc.robot.subsystems.chassis.DrivetrainConstants;

/**
 * This command generates a trajectory and follows it (as opposed
 * to getting the trajectory from a JSON file).
 */
public class FollowGeneratedTrajectoryCommand extends CommandBase {
	private DrivetrainSubsystem drivetrain;

	public FollowGeneratedTrajectoryCommand(DrivetrainSubsystem drivetrain) {
		this.drivetrain = drivetrain;
		this.addRequirements(this.drivetrain);

		/**
		 * This is an arraylist of Pose2d objects that are points the trajectory
		 * passes through. The start and end poses are mandatory, and you can also
		 * add additional waypoints in between.
		 */
		this.kTrajectoryWayPointsList = new ArrayList<Pose2d>();
		this.timer = new Timer();

		this.kPositionTolerance = new Pose2d(
				DrivetrainConstants.kPositionToleranceMetersX,
				DrivetrainConstants.kPositionToleranceMetersY,
				Rotation2d.fromDegrees(DrivetrainConstants.kPositionToleranceDegrees));
	}

	private Timer timer;
	private Trajectory trajectory;

	private PIDController PIDControllerX, PIDControllerY;
	private ProfiledPIDController profiledPIDControllerAngle;

	private ArrayList<Pose2d> kTrajectoryWayPointsList;

	private HolonomicDriveController driveController;
	private Pose2d currentPose;
	private Trajectory.State currentSetpoint;

	private Pose2d kPositionTolerance;

	@Override
	public void initialize() {
		// The trajectory is generated in initialize() instead of in the constructor
		// because the starting point has to be the robot's current position.

		// Start point
		this.kTrajectoryWayPointsList.add(this.drivetrain.getCurretnPose());
		DriverStation.reportError("starting pose: " + this.drivetrain.getCurretnPose().toString(), false);
		// End point
		this.kTrajectoryWayPointsList.add(new Pose2d(
				DrivetrainConstants.kTrajectoryEndPose_FieldRelativeXMeters,
				DrivetrainConstants.kTrajectoryEndPose_FieldRelativeYMeters,
				Rotation2d.fromDegrees(DrivetrainConstants.kTrajectoryEndPose_FieldRelativeDegrees)));

		this.trajectory = TrajectoryGenerator.generateTrajectory(
				this.kTrajectoryWayPointsList,
				new TrajectoryConfig(DrivetrainConstants.kMaxChassisVelocityMPS,
						DrivetrainConstants.kMaxChassisAccelMPSSquared));
		DriverStation.reportError("trajectory successfully generated!", false);

		// The closed-loop controllers should start from scratch every time the command
		// starts, so they're initialised in initialize() instead of the command's
		// constructor.
		this.PIDControllerX = new PIDController(
				DrivetrainConstants.kXControllerP,
				DrivetrainConstants.kXControllerI,
				DrivetrainConstants.kXControllerD);

		this.PIDControllerY = new PIDController(
				DrivetrainConstants.kYControllerP,
				DrivetrainConstants.kYControllerI,
				DrivetrainConstants.kYControllerD);

		this.profiledPIDControllerAngle = new ProfiledPIDController(
				DrivetrainConstants.kAngleControllerP,
				DrivetrainConstants.kAngleControllerI,
				DrivetrainConstants.kAngleControllerD,
				new TrapezoidProfile.Constraints(
						DrivetrainConstants.kAngleControllerMaxVelocity,
						DrivetrainConstants.kAngleControllerMaxAccel));

		// Angle is measured on a circle, so the minimum and maximum value correspond
		// to the same position in reality. Here the angle is measured in radians, so
		// the min and max values are -PI and PI.
		this.profiledPIDControllerAngle.enableContinuousInput(-Math.PI, Math.PI);

		// HolonomicDriveController accepts 3 constructor parameters: two PID
		// controllers for X and Y, and a profiled PID controller (TrapezoidProfile)
		// for controlling the heading.
		this.driveController = new HolonomicDriveController(
				this.PIDControllerX,
				this.PIDControllerY,
				this.profiledPIDControllerAngle);

		// The position tolerance in X, Y and angle for HolonomicDriveController
		// is represented in one Pose2d object.
		this.driveController.setTolerance(this.kPositionTolerance);

		this.driveController.setEnabled(true);
		this.timer.reset();
		this.timer.start();
	}

	@Override
	public void execute() {
		// The trajectory has a pose, angle, velocity and acceleration for each point
		// in time since start. This is represented in a Trajectory.State object.
		this.currentSetpoint = this.trajectory.sample(this.timer.get());

		// Pose2d represents X in meters, Y in meters, and angle as Rotation2d.
		this.currentPose = this.drivetrain.getCurretnPose();

		// The HolonimicDriveController.calculate() method returns the desired
		// ChassisSpeeds in order to reach the current setpoint. This is then
		// passed to the DrivetrainSubsystem.drive() method.
		this.drivetrain.drive(
				this.driveController.calculate(
						this.currentPose, this.currentSetpoint, this.currentPose.getRotation()),
				true);
	}

	@Override
	public void end(boolean interrupted) {
		this.timer.stop();
		this.timer.reset();
		this.driveController.setEnabled(false);
	}

	@Override
	public boolean isFinished() {
		// Returns true if the time the trajectory takes to drive
		// has passed, and driveController is at it's setpoint or
		// within the position tolerance for it.
		return (this.trajectory.getTotalTimeSeconds() < this.timer.get()
				&& this.driveController.atReference());
	}
}