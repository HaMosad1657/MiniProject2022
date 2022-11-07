

package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import frc.robot.subsystems.chassis.DrivetrainConstants;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

/**
 * This command loads a JSON file representing a trajectory,
 * creates a trajectory from it, and follows it.
 * <p>
 * This command can safely be run more than once!
 */
public class FollowJSONTrajectoryCommand extends CommandBase {
	private DrivetrainSubsystem drivetrain;

	private PathPlannerTrajectory trajectory1;
	private Timer timer;

	public FollowJSONTrajectoryCommand(DrivetrainSubsystem drivetrain) {
		this.drivetrain = drivetrain;
		this.addRequirements(this.drivetrain);

		// Loading the JSON file can take more than 0.2 miliseconds, so
		// it should be done in the constructor and not in initialize().
		// Also, it only needs to be done once.

		try {
			// Resolve the path, read the JSON file and create the trajctory...
			this.trajectory1 = PathPlanner.loadPath("New Path", new PathConstraints(
					DrivetrainConstants.kMaxChassisVelocityMPSAuto,
					DrivetrainConstants.kMaxChassisAccelMPSSquared));
			DriverStation.reportError("Trajectory succesfully created from JSON", false);
		}
		// Catching this Exception isn't intended to fix anything since if the trajectory
		// generation failed, it will crash with a NullPointerException right after that.
		// It just makes the Exception easier to find in the RioLog.
		catch (Exception exception) {
			DriverStation.reportError("Failed to create trajectory from JSON", false);
			DriverStation.reportError(exception.toString(), true);
		}

		this.timer = new Timer();
	}

	private PIDController PIDControllerX, PIDControllerY;
	private ProfiledPIDController profiledPIDControllerAngle;
	private HolonomicDriveController driveController;

  @Override
  public void initialize() {
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
				DrivetrainConstants.kAngleControllerD, new TrapezoidProfile.Constraints(
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

		this.driveController.setTolerance(new Pose2d(
				DrivetrainConstants.kPositionToleranceMetersX,
				DrivetrainConstants.kPositionToleranceMetersY,
				Rotation2d.fromDegrees(DrivetrainConstants.kPositionToleranceDegrees)));

		this.driveController.setEnabled(true);
		this.timer.reset();
		this.timer.start();
  }

  @Override
  public void execute() {
	// The trajectory has a pose, angle, velocity and acceleration for each point
	// in time since start. This is represented in a Trajectory.State object.
	// We don't use this angle, instead we use the one from PathPlanner.
	Trajectory.State currentPositionSetpoint = this.trajectory1.sample(this.timer.get() + 0.02);

	// basically the same thing, but with PathPlanner, so it has information about
	// the angle for a holonomic robot (which the WPILib trajectory doesn't).
	PathPlannerState currentPathPlannerState = (PathPlannerState) this.trajectory1.sample(this.timer.get() + 0.02);
	Rotation2d currentAngleSetpoint = currentPathPlannerState.holonomicRotation;

	Pose2d currentPose = this.drivetrain.getCurrentPose();

	// The HolonimicDriveController.calculate() method returns the desired
	// ChassisSpeeds in order to reach the current setpoints. This is then 
	// passed to the DrivetrainSubsystem.drive() method.
	this.drivetrain.drive(this.driveController.calculate(
					currentPose,
					currentPositionSetpoint,
					currentAngleSetpoint));
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
}
