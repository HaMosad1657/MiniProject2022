package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import frc.robot.subsystems.chassis.DrivetrainConstants;

/**
 * This command generates a trajectory in the code and follows it
 * (as opposed to getting the trajectory from a JSON file).
 */
public class FollowGeneratedTrajectoryCommand extends CommandBase {
	private DrivetrainSubsystem drivetrain;

	public FollowGeneratedTrajectoryCommand(DrivetrainSubsystem drivetrain) {
		this.drivetrain = drivetrain;
		this.addRequirements(this.drivetrain);

		/**
		 * This is an arraylist of PathPoints objects that are points the trajectory
		 * passes through. The start and end poses are mandatory, and you can also
		 * add additional waypoints in between.
		 */
		this.kTrajectoryWaypointsList = new ArrayList<PathPoint>();

		this.timer = new Timer();

		// The position tolerance in X, Y and angle for HolonomicDriveController
		// is represented in one Pose2d object.
		this.kPositionTolerance = new Pose2d(
				DrivetrainConstants.kPositionToleranceMetersX,
				DrivetrainConstants.kPositionToleranceMetersY,
				Rotation2d.fromDegrees(DrivetrainConstants.kPositionToleranceDegrees));
	}

	private Timer timer;
	private PathPlannerTrajectory trajectory1;

	private PIDController PIDControllerX, PIDControllerY, PIDControllerAngle;

	private ArrayList<PathPoint> kTrajectoryWaypointsList;

	private PPHolonomicDriveController driveController;
	private Pose2d currentPose;
	private PathPlannerState currentSetpoint;

	private Pose2d kPositionTolerance;

	@Override
	public void initialize() {
		// The trajectory is generated in initialize() instead of in the constructor
		// because the starting point is the robot's current position.

		PathConstraints trajectoryConstraints = new PathConstraints(
				DrivetrainConstants.kMaxChassisVelocityMPSAuto,
				DrivetrainConstants.kMaxChassisAccelMPSSquared);

		/**
		 * Each point has a:
		 * -- X and Y in meters as a Translation2d.
		 * -- A heading (aka direction of travel) as a Rotation2d.
		 * -- Optionally the orientation of the robot as a Rotation2d.
		 * -- Optionally a velocity override (aka specifying the velocity
		 * manually instead of PathPlanner calculating it) as a double.
		 */

		// Start point
		this.kTrajectoryWaypointsList.add(new PathPoint(
				this.drivetrain.getCurrentPose().getTranslation(),
				// Calculate the angle between the robot's current point and the next
				new Rotation2d(this.getAngleFromPoints(
						this.drivetrain.getCurrentPose().getX(),
						this.drivetrain.getCurrentPose().getY(),
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeXMeters,
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeYMeters)),
				this.drivetrain.getGyroRotation()));

		// Optional: add more points here

		this.kTrajectoryWaypointsList.add(new PathPoint(
				new Translation2d(
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeXMeters,
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeYMeters),
				// Calculate the angle between the robot's previous point and this one
				new Rotation2d(180 - this.getAngleFromPoints(
						this.drivetrain.getCurrentPose().getX(),
						this.drivetrain.getCurrentPose().getY(),
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeXMeters,
						DrivetrainConstants.kTrajectoryEndPose_FieldRelativeYMeters)),
				Rotation2d.fromDegrees(DrivetrainConstants.kTrajectoryEndAngle_FieldRelativeDegrees)));

		// Now create the trajectory...
		this.trajectory1 = PathPlanner.generatePath(
				trajectoryConstraints,
				this.kTrajectoryWaypointsList.get(0),
				this.kTrajectoryWaypointsList.get(1));
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

		this.PIDControllerAngle = new PIDController(
				DrivetrainConstants.kAngleControllerP,
				DrivetrainConstants.kAngleControllerI,
				DrivetrainConstants.kAngleControllerD);

		// Angle is measured on a circle, so the minimum and maximum values are
		// the same position in reality. Here the angle is measured in radians
		// so the min and max values are -PI and PI.
		this.PIDControllerAngle.enableContinuousInput(-Math.PI, Math.PI);

		// HolonomicDriveController accepts 3 constructor parameters: two PID
		// controllers for X and Y, and a profiled PID controller (TrapezoidProfile)
		// for controlling the angle.
		this.driveController = new PPHolonomicDriveController(
				this.PIDControllerX,
				this.PIDControllerY,
				this.PIDControllerAngle);

		this.driveController.setTolerance(this.kPositionTolerance);

		this.driveController.setEnabled(true);
		this.timer.reset();
		this.timer.start();
	}

	@Override
	public void execute() {
		// The WPILib trajectory has a position, direction of travel, linear velocity
		// velocity and linear acceleration for each point in time since start. This is
		// represented in a Trajectory.State object.
		// The PathPlanner trajectory has all the information that the WPILib one has,
		// and also robot orientation, angular velocity and angular acceleration. This
		// is represented in a PathPlannerState object.

		// the sample(time) method returns a Trajectory.State object, but because
		// PathPlannerState extends it, we can cast it to PathPlannerState, which
		// has the right information for a holonomic drivetrain (like our swerve).
		this.currentSetpoint = (PathPlannerState)this.trajectory1.sample(this.timer.get() + 0.02);
		this.currentPose = this.drivetrain.getCurrentPose();

		// The calculate() method returns the desired ChassisSpeeds in order to reach the
		// current setpoint. This is then passed to the DrivetrainSubsystem.drive() method.
		this.drivetrain.drive(
				this.driveController.calculate(
						this.currentPose,
						this.currentSetpoint));
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
		return (this.trajectory1.getTotalTimeSeconds() < this.timer.get()
				&& this.driveController.atReference());
	}

	private double getAngleFromPoints(double x1, double y1, double x2, double y2) {
		return Math.PI / 2 - Math.atan2(y2 - y1, x2 - x1);
	}
}