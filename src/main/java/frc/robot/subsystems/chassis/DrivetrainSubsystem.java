package frc.robot.subsystems.chassis;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

	private static DrivetrainSubsystem instance;

	public static DrivetrainSubsystem getInstance() {
		if (instance == null)
			instance = new DrivetrainSubsystem();
		return instance;
	}

	/**
	 * An array of SwerveModuleState objects (one for each module),
	 * ordered:
	 * <ul>
	 * <li>front left [0]
	 * <li>front right [1]
	 * <li>back left [2]
	 * <li>back right [3]
	 * </ul>
	 * A SwerveModuleState object represents speeds and angles of
	 * an individual swerve module.
	 */
	private SwerveModuleState[] states;

	/**
	 * Represents the speeds of the chassis (robot relative).
	 * It contains three speeds:
	 * <ul>
	 * <li>The forwards/backwards velocity in meters per second (forwards is +)
	 * <li>The left/right velocity in meters per second (left is +)
	 * <li>The angular velocity in radians per second (counter-clockwise is +)
	 */
	private ChassisSpeeds chassisSpeeds;

	/**
	 * Helps convert between ChassisSpeeds and SwerveModuleState[].
	 * <p>
	 * Inverse kinematics uses the relative locations of the modules with respect
	 * to the center of rotation (which by the way, you can choose, because swerve)
	 * to convert from desired chassis velocities to individual module states.
	 * <p>
	 * Forward kinematics does exactly the opposite: it converts from module states
	 * to chassis velocities. It's also used for odometry.
	 */
	private final SwerveDriveKinematics kinematics;

	/**
	 * SwerveDriveOdometry lets you track the robot's position on the field
	 * using the encoder readings (read: module states) and gyro angle, then
	 * uses forward kinematics to know the robot's speeds, and accumulates
	 * them over time to know it's position.
	 */
	private final SwerveDriveOdometry odometry;

	private final ShuffleboardTab chassisTab, odometryTab, fieldTab, debuggingTab;
	private final NetworkTableEntry ox, oy,
			frontLeftAbsAngleEntry, frontRightAbsAngleEntry, backLeftAbsAngleEntry, backRightAbsAngleEntry,
			frontLeftAngleSetpointEntry, frontLeftRawMeasurmentEntry;
	private final Field2d field;
	private final AHRS navx;
	private final Timer encoderSyncTimer;

	private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

	/*
	 * Initialize + configure TalonFXs and CANCoders,
	 * initialize Shuffleboard tabs and entries,
	 * initialize kinematics and odometry,
	 * Start communication with navX via USB
	 * and wait for it to finish startup calibration.
	 */
	private DrivetrainSubsystem() {

		this.frontLeftModule = new SwerveModule(
				DrivetrainConstants.kFrontLeftDriveMotorID,
				DrivetrainConstants.kFrontLeftSteerMotorID,
				DrivetrainConstants.kFrontLeftCANCoderID,
				DrivetrainConstants.kFrontLeftAngleOffset);

		this.frontRightModule = new SwerveModule(
				DrivetrainConstants.kFrontRightDriveMotorID,
				DrivetrainConstants.kFrontRightAngleMotorID,
				DrivetrainConstants.kFrontRightCANCoderID,
				DrivetrainConstants.kFrontRightAngleOffset);

		this.backLeftModule = new SwerveModule(
				DrivetrainConstants.kBackLeftDriveMotorID,
				DrivetrainConstants.KBackLeftSteerMotorID,
				DrivetrainConstants.kBackLeftCANCoderID,
				DrivetrainConstants.kBackLeftAngleOffset);

		this.backRightModule = new SwerveModule(
				DrivetrainConstants.kBackRightDriveMotorID,
				DrivetrainConstants.kBackRightSteerMotorID,
				DrivetrainConstants.kBackRightCANCoderID,
				DrivetrainConstants.kBackRightAngleOffset);

		this.debuggingTab = Shuffleboard.getTab("Debugging");
		this.frontLeftAngleSetpointEntry = this.debuggingTab.add("FR Setpoint", 0)
				.withWidget(BuiltInWidgets.kGraph).getEntry();
		this.frontLeftRawMeasurmentEntry = this.debuggingTab.add("FR Measurment", 0)
				.withWidget(BuiltInWidgets.kGraph).getEntry();

		this.field = new Field2d();
		this.fieldTab = Shuffleboard.getTab("Field");
		this.fieldTab.add("Field", this.field);

		this.chassisTab = Shuffleboard.getTab("Chassis");
		this.frontLeftAbsAngleEntry = this.chassisTab.add(
				"Front Left Absloute Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
		this.frontRightAbsAngleEntry = this.chassisTab.add(
				"Front Right Absloute Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
		this.backLeftAbsAngleEntry = this.chassisTab.add(
				"Back Left Absloute Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
		this.backRightAbsAngleEntry = this.chassisTab.add(
				"Back Right Absloute Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

		this.odometryTab = Shuffleboard.getTab("Odometry");
		this.ox = this.odometryTab.add("odometry x axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.oy = this.odometryTab.add("odometry y axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

		// chassisSpeeds should start with zero for all speeds in the beginning,
		// because the robot starts the match not moving.
		this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		// Construct a kinematics object with our chassis's size.
		// Four Translation2d objects because our chassis has 4 corners,
		// with a swerve module in each.
		this.kinematics = new SwerveDriveKinematics(
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0));

		// Start communication between the navX and RoboRIO using the
		// outer USB-A port on the RoboRIO, with the default update rate of 60 hz.
		this.navx = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 60);
		this.navx.enableLogging(true);

		while (this.navx.isCalibrating()) {
			// Wait for the navx to finish the startup calibration - loop
			// overrun is okay here, because this all happens in robotInit().
			// Waiting is important because if the robot moves when the navX
			// is calibrating, it can return inaccurate measurments.
		}
		DriverStation.reportError("navX done calibrating", false);
		// Add the navx widget to the shuffleboard
		this.odometryTab.add(this.navx);

		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		// Construct a SwerveDriveOdometry with X=0, Y=0, and current gyro angle
		// (which would be zero here, because it calibrates on startup)
		this.odometry = new SwerveDriveOdometry(this.kinematics, this.getGyroRotation());

		this.encoderSyncTimer = new Timer();
		this.encoderSyncTimer.start();
	}

	@Override
	public void periodic() {
		// The odometry must be updated periodically, in order to accurately track the
		// robot's position.
		this.odometry.update(this.getGyroRotation(), this.states[0], this.states[1], this.states[2], this.states[3]);

		// Update shuffleboard entries...
		this.frontLeftRawMeasurmentEntry.setDouble(this.frontLeftModule.getAbsWheelAngle());
		this.frontLeftAbsAngleEntry.setDouble(this.frontLeftModule.getAbsWheelAngle());
		this.frontRightAbsAngleEntry.setDouble(this.frontRightModule.getAbsWheelAngle());
		this.backLeftAbsAngleEntry.setDouble(this.backLeftModule.getAbsWheelAngle());
		this.backRightAbsAngleEntry.setDouble(this.backRightModule.getAbsWheelAngle());
		this.ox.setDouble(this.getCurrentPose().getX());
		this.oy.setDouble(this.getCurrentPose().getY());
		this.field.setRobotPose(this.odometry.getPoseMeters());

		// If the robot isn't moving for more than a second (five iterations), sync the encoders
		if (this.chassisSpeeds.vxMetersPerSecond == 0 &&
				this.chassisSpeeds.vyMetersPerSecond == 0 &&
				this.chassisSpeeds.omegaRadiansPerSecond == 0 &&
				this.encoderSyncTimer.get() > 1) {
			this.frontLeftModule.syncSteerEncoder();
			this.frontRightModule.syncSteerEncoder();
			this.backLeftModule.syncSteerEncoder();
			this.backRightModule.syncSteerEncoder();
		}
		// if the robot is moving, reset the timer
		else if (this.chassisSpeeds.vxMetersPerSecond != 0 &&
				this.chassisSpeeds.vyMetersPerSecond != 0 &&
				this.chassisSpeeds.omegaRadiansPerSecond != 0) {
			this.encoderSyncTimer.reset();
		}
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		// Updates the desired speeds of the chassis
		this.chassisSpeeds = chassisSpeeds;

		// Preforms inverse kinematics: turns the desired speeds of the entire
		// chassis into speed and angle setpoints for the individual modules.
		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds); // Make temp variable of states

		// Temp variable for debugging
		SwerveModuleState[] optimizedStates = this.states;

		// Optimize the modules to not rotate more then 90 degrees
		optimizedStates[0] = SwerveModuleState.optimize(this.states[0],
				Rotation2d.fromDegrees(this.frontLeftModule.getAbsWheelAngle()));
		optimizedStates[1] = SwerveModuleState.optimize(this.states[1],
				Rotation2d.fromDegrees(this.frontRightModule.getAbsWheelAngle()));
		optimizedStates[2] = SwerveModuleState.optimize(this.states[2],
				Rotation2d.fromDegrees(this.backLeftModule.getAbsWheelAngle()));
		optimizedStates[3] = SwerveModuleState.optimize(this.states[3],
				Rotation2d.fromDegrees(this.backRightModule.getAbsWheelAngle()));

		this.states = optimizedStates;

		// If any of the setpoints are over the max speed, this method lowers
		// all of them (in the same ratio).
		SwerveDriveKinematics.desaturateWheelSpeeds(this.states, DrivetrainConstants.kMaxChassisVelocityMPS);

		// Front left
		this.frontLeftModule.setDriveMotor(this.states[0].speedMetersPerSecond);
		this.frontLeftModule.setSteerMotor(this.states[0].angle.getDegrees());
		// For debugging
		this.frontLeftAngleSetpointEntry.setDouble(this.states[0].angle.getDegrees());

		// Front right
		this.frontRightModule.setDriveMotor(this.states[1].speedMetersPerSecond);
		this.frontRightModule.setSteerMotor(this.states[1].angle.getDegrees());

		// Back left
		this.backLeftModule.setDriveMotor(this.states[2].speedMetersPerSecond);
		this.backLeftModule.setSteerMotor(this.states[2].angle.getDegrees());

		// Back right
		this.backRightModule.setDriveMotor(this.states[3].speedMetersPerSecond);
		this.backRightModule.setSteerMotor(this.states[3].angle.getDegrees());
	}// End drive()

	/**
	 * Sets the yaw angle to zero. This is used to set the direction
	 * the robot is currently facing as the "forward" direction.
	 * <p>
	 * This method does NOT recalibrate the navX, it just adds an offset in the
	 * software to make the angle zero. For more information on navX calibration:
	 * https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/
	 */
	public void resetYaw() {
		this.navx.zeroYaw();
		// The odometry isn't reset, it's just informed of the new angle.
		this.odometry.resetPosition(this.getCurrentPose(), new Rotation2d(0));
	}

	/**
	 * Returns the negated yaw angle (360 - angle) as a Rotation2d.
	 * <p>
	 * When the navX is parallel to the floor and was calibrated parallel to the
	 * floor, "yaw angle" is the angle of the borad-relative Y axis. If the navX is
	 * not mounted parallel to the floor, follow the instructions on this website:
	 * https://pdocs.kauailabs.com/navx-mxp/installation/omnimount/
	 */
	public Rotation2d getGyroRotation() {
		// We have to negate the angle of the NavX so that rotating the
		// robot counter-clockwise (left) makes the angle increase, and
		// rotating clockwise (right) makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - this.navx.getYaw());
	}

	/**
	 * Turns the modules so that they make an x shape, until they are
	 * told to do something else.
	 */
	public void crossLockWheels() {
		this.frontLeftModule.setDriveMotor(0);
		this.frontLeftModule.setSteerMotor(DrivetrainConstants.kFrontLeftCrossAngleDegrees);
		this.frontRightModule.setDriveMotor(0);
		this.frontRightModule.setSteerMotor(DrivetrainConstants.kFrontRightCrossAngleDegrees);
		this.backLeftModule.setDriveMotor(0);
		this.backLeftModule.setSteerMotor(DrivetrainConstants.kBackLeftCrossAngleDegrees);
		this.backRightModule.setDriveMotor(0);
		this.backRightModule.setSteerMotor(DrivetrainConstants.kBackRightCrossAngleDegrees);
	}

	/**
	 * Returns a Pose2d object representing the robot's current position.
	 * X in meters, Y in meters, angle in Rotation2d.
	 */
	public Pose2d getCurrentPose() {
		return this.odometry.getPoseMeters();
	}

	/**
	 * Discards the odometry measurments and sets the values to 0,0,0
	 */
	public void resetOdometry() {
		this.odometry.resetPosition(new Pose2d(), this.getGyroRotation());
	}

	public double getChassisForwardAccelMPSSquared() {
		return this.navx.getWorldLinearAccelY() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}

	public double getChassisLateralAccelMPSSquared() {
		return this.navx.getWorldLinearAccelX() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}
}
