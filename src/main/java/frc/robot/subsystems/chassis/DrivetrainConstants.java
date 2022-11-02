package frc.robot.subsystems.chassis;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

public class DrivetrainConstants {

	/**
	 * This can be reduced to cap the robot's maximum speed. Typically,
	 * this is useful during initial testing of the robot.
	 */
	protected static final double kMaxVoltage = 11.0;

	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction()
	// * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

	/**
	 * The max velocity of the chassis in meters per second.
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	public static final double kMaxChassisVelocityMPS = 6380.0 / 60.0 *
			SdsModuleConfigurations.MK4_L2.getDriveReduction()
			* SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

	public static final double kMaxChassisVelocityMPSAuto = 6000.0 / 60.0 *
			SdsModuleConfigurations.MK4_L2.getDriveReduction()
			* SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

	/**
	 * The max acceleration of the chassis in meters per second squared.
	 * This is a measure of how much the robot can accelerate when driving
	 * in a straight line.
	 */
	public static final double kMaxChassisAccelMPSSquared = 2.5; // FIXME find max acceleration

	/**
	 * The left-to-right distance between the drivetrain wheels
	 * Should be measured from center to center.
	 */
	public static final double kDrivetrainTrackWidthMeters = 5.903;

	/**
	 * The front-to-back distance between the drivetrain wheels.
	 * Should be measured from center to center.
	 */
	public static final double kDrivetrainWheelbaseMeters = 5.903;

	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * This is a measure of how fast the robot can rotate in place.
	 */
	public static final double kMaxAngularVelocity_RadiansPerSecond = kMaxChassisVelocityMPS /
			Math.hypot(kDrivetrainTrackWidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

	/**
	 * to find the offsets, put them as zero in the code, upload, then straighten
	 * the
	 * modules with a profile (make sure to be in disabled mode) and replace the
	 * zeros
	 * with the absloute encoder angles from the ShuffleBoard.
	 * The wheels should all point forwards, with the bevel gears facing to the
	 * right.
	 */
	// Front left module
	public static final int kFrontLeftDriveMotorID = 20;
	public static final int kFrontLeftAngleMotorID = 21;
	public static final int kFrontLeftAngleEncoderID = 10;
	public static final double kFrontLeftAngleOffset = -Math.toRadians(255.7);

	// Front right module
	public static final int kFrontRightDriveMotorID = 22;
	public static final int kFrontRightAngleMotorID = 23;
	public static final int kFrontRightAngleEncoderID = 11;
	public static final double kFrontRightAngleOffset = -Math.toRadians(178.5);

	// Back left module
	public static final int kBackLeftDriveMotorID = 24;
	public static final int KBackLeftAngleMotorID = 25;
	public static final int kBackLeftAngleEncoderID = 12;
	public static final double kBackLeftAngleOffset = -Math.toRadians(183.6);

	// Back right module
	public static final int kBackRightDriveMotorID = 26;
	public static final int kBackRightAngleMotorID = 27;
	public static final int kBackRightAngleEncoderID = 13;
	public static final double kBackRightAngleOffset = -Math.toRadians(220.5);

	/**
	 * The 2 PID controllers are part of HolonomicDriveController (they're passed as
	 * constructor arguments) and their job is to correct for error in the
	 * field-relative x and y directions, respectively. For example, if the first 2
	 * constructor arguments are PIDController(1, 0, 0) and PIDController(1.2, 0, 0)
	 * respectively, the holonomic drive controller will add an additional meter per
	 * second in the x direction for every meter of error in the x axis, and will
	 * add an additional 1.2 meters per second in the y direction for every meter of
	 * error in the y axis.
	 */
	// FIXME: find correct PID gains
	public static final double kXControllerP = 0.002;
	public static final double kXControllerI = 0.00;
	public static final double kXControllerD = 0.00;

	public static final double kYControllerP = 0.002;
	public static final double kYControllerI = 0.00;
	public static final double kYControllerD = 0.00;

	public static final double kAngleControllerP = 0.50;
	public static final double kAngleControllerI = 0.00;
	public static final double kAngleControllerD = 0.00;

	// The Constraints() constructor accepts 2 doubles, which are the max velocity
	// and acceleration. These are constraints for the profiled PID controller, NOT
	// for the trajectory!!!
	// Units are radians per second and radians per second squared, respectively.
	// (for refrence, there is about 6.28 radians in a circle)
	// FIXME: find correct constraints for angle controller
	public static final double kAngleControllerMaxVelocity = kMaxAngularVelocity_RadiansPerSecond;
	public static final double kAngleControllerMaxAccel = 3.00;

	// These values are used to create PathPoint objects which are the trajectory's
	// waypoints. For now there are only two waypoints, the start and end, but
	// more can be added. The degrees are converted to Rotation2d.
	// There is no start point because it's the robot's current pose.

	public static final double kTrajectoryEndPose_FieldRelativeXMeters = 0.05;
	public static final double kTrajectoryEndPose_FieldRelativeYMeters = 0.05;
	public static final double kTrajectoryEndHeading_FieldRelativeDegrees = 0.00; // This value is calculated instead
	public static final double kTrajectoryEndAngle_FieldRelativeDegrees = 90;

	// This is used in DrivetrainSubsystem to convert the linear acceleration
	// that the navX measures (which is in G) to the units WPILib uses, which
	// is in meters per second squared.
	public static final double kGravityToMPSSquaredConversionFactor = 9.80665;

	// These values are used to create a Pose2d object, which is the position
	// tolerance for driveController in auto. The degrees are converted to
	// Rotation2d.
	// FIXME: find correct position tolerances for auto
	public static final double kPositionToleranceMetersX = 0.25;
	public static final double kPositionToleranceMetersY = 0.25;
	public static final double kPositionToleranceDegrees = 10.000;

	// Module angles for cross locking the wheels
	public static final double kFrontLeftCrossAngleRadians = 0.785398; // 45 degrees
	public static final double kFrontRightCrossAngleRadians = 2.35619; // 135 degrees
	public static final double kBackLeftCrossAngleRadians = 2.35619; // 135
	public static final double kBackRightCrossAngleRadians = 3.92699; // 45
}