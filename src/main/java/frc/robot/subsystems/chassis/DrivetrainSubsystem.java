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
	private SwerveModuleState[] desiredStates;

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
	private SwerveModuleState[] empiricalStates;

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

	private double[] previousRotations;

	private final ShuffleboardTab chassisTab, odometryTab, fieldTab, debuggingTab;
	private final NetworkTableEntry ox, oy,
			frontLeftAbsAngleEntry, frontRightAbsAngleEntry, backLeftAbsAngleEntry, backRightAbsAngleEntry,
			frontLeftAbsAnglEntry, frontLeftIntegratedSensorEntry;
	private final Field2d field;
	private final AHRS navx;
	private final Timer encoderSyncTimer;

	private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

	/**
	 * Initialize swerve modules, configure PID,
	 * initialize Shuffleboard tabs and entries,
	 * initialize kinematics and odometry,
	 * start communication with navX via USB and
	 * wait for it to finish startup calibration.
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

		// Front left
		this.frontLeftModule.configDrivePID(
				DrivetrainConstants.kDriveP,
				DrivetrainConstants.kDriveI,
				DrivetrainConstants.kDriveD, 0);
		this.frontLeftModule.configSteerPID(
				DrivetrainConstants.kSteerP,
				DrivetrainConstants.kSteerI,
				DrivetrainConstants.kSteerD, 0);
		// Front right
		this.frontRightModule.configDrivePID(
				DrivetrainConstants.kDriveP,
				DrivetrainConstants.kDriveI,
				DrivetrainConstants.kDriveD, 0);
		this.frontRightModule.configSteerPID(
				DrivetrainConstants.kSteerP,
				DrivetrainConstants.kSteerI,
				DrivetrainConstants.kSteerD, 0);
		// Back left
		this.backLeftModule.configDrivePID(
				DrivetrainConstants.kDriveP,
				DrivetrainConstants.kDriveI,
				DrivetrainConstants.kDriveD, 0);
		this.backLeftModule.configSteerPID(
				DrivetrainConstants.kSteerP,
				DrivetrainConstants.kSteerI,
				DrivetrainConstants.kSteerD, 0);
		// Back right
		this.backRightModule.configDrivePID(
				DrivetrainConstants.kDriveP,
				DrivetrainConstants.kDriveI,
				DrivetrainConstants.kDriveD, 0);
		this.backRightModule.configSteerPID(
				DrivetrainConstants.kSteerP,
				DrivetrainConstants.kSteerI,
				DrivetrainConstants.kSteerD, 0);

		this.debuggingTab = Shuffleboard.getTab("Debugging");
		this.frontLeftAbsAnglEntry = this.debuggingTab.add("FR Abs Angle", 0)
				.withWidget(BuiltInWidgets.kTextView).getEntry();
		this.frontLeftIntegratedSensorEntry = this.debuggingTab.add("FR Integrated Sensor", 0)
				.withWidget(BuiltInWidgets.kTextView).getEntry();

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
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthM / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseM / 2.0),
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthM / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseM / 2.0),
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthM / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseM / 2.0),
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthM / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseM / 2.0));

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

		this.desiredStates = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

		this.empiricalStates = new SwerveModuleState[] {
				this.frontLeftModule.getSwerveModuleState(),
				this.frontRightModule.getSwerveModuleState(),
				this.backLeftModule.getSwerveModuleState(),
				this.backRightModule.getSwerveModuleState()
		};

		// Construct a SwerveDriveOdometry with X=0, Y=0, and current gyro angle
		// (which would be zero here, because the navX calibrates on startup)
		this.odometry = new SwerveDriveOdometry(this.kinematics, this.getGyroRotation());

		this.previousRotations = new double[] { 0, 0, 0, 0 };

		this.encoderSyncTimer = new Timer();
		this.encoderSyncTimer.start();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		// Updates the desired speeds of the chassis
		this.chassisSpeeds = chassisSpeeds;

		// Preforms inverse kinematics: turns the desired speeds of the entire
		// chassis into speed and angle setpoints for the individual modules.
		this.desiredStates = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

		// Optimize the modules to not rotate more then 90 degrees.
		// Note that this is our optimization method, not WPILib's,
		// so please do question it if the robot is driving badly
		desiredStates[0] = SwerveModule.optimize(
				this.desiredStates[0],
				this.frontLeftModule.getAbsWheelAngle());
		desiredStates[1] = SwerveModule.optimize(
				this.desiredStates[1],
				this.frontRightModule.getAbsWheelAngle());
		desiredStates[2] = SwerveModule.optimize(
				this.desiredStates[2],
				this.backLeftModule.getAbsWheelAngle());
		desiredStates[3] = SwerveModule.optimize(
				this.desiredStates[3],
				this.backRightModule.getAbsWheelAngle());

		// If any of the setpoints are over the max speed, this method lowers
		// all of them in the same ratio. Note that this method doesn't return
		// anything, but rather changes the SwerveModuleState[] that was passed
		// to it (this.desiredStates)
		SwerveDriveKinematics.desaturateWheelSpeeds(this.desiredStates, DrivetrainConstants.kMaxChassisVelocityMPS);

		// If the robot doesn't need to move
		if (this.chassisSpeeds.vxMetersPerSecond == 0 &&
				this.chassisSpeeds.vyMetersPerSecond == 0 &&
				this.chassisSpeeds.omegaRadiansPerSecond == 0) {
			this.frontLeftModule.setDriveMotor(0);
			this.frontLeftModule.setSteerMotor(this.previousRotations[0]);
			this.frontRightModule.setDriveMotor(0);
			this.frontRightModule.setSteerMotor(this.previousRotations[1]);
			this.backLeftModule.setDriveMotor(0);
			this.backLeftModule.setSteerMotor(this.previousRotations[2]);
			this.backRightModule.setDriveMotor(0);
			this.backRightModule.setSteerMotor(this.previousRotations[3]);
		} else {
			// Front left
			this.frontLeftModule.setDriveMotor(this.desiredStates[0].speedMetersPerSecond);
			this.frontLeftModule.setSteerMotor(this.desiredStates[0].angle.getDegrees());
			this.previousRotations[0] = this.frontLeftModule.getAbsWheelAngle();
			// For debugging
			this.frontLeftAbsAnglEntry.setDouble(this.desiredStates[0].angle.getDegrees());

			// Front right
			this.frontRightModule.setDriveMotor(this.desiredStates[1].speedMetersPerSecond);
			this.frontRightModule.setSteerMotor(this.desiredStates[1].angle.getDegrees());
			this.previousRotations[1] = this.frontRightModule.getAbsWheelAngle();

			// Back left
			this.backLeftModule.setDriveMotor(this.desiredStates[2].speedMetersPerSecond);
			this.backLeftModule.setSteerMotor(this.desiredStates[2].angle.getDegrees());
			this.previousRotations[2] = this.backLeftModule.getAbsWheelAngle();

			// Back right
			this.backRightModule.setDriveMotor(this.desiredStates[3].speedMetersPerSecond);
			this.backRightModule.setSteerMotor(this.desiredStates[3].angle.getDegrees());
			this.previousRotations[3] = this.backRightModule.getAbsWheelAngle();
		}
	}// End drive()

	/**
	 * Sets the yaw angle to zero. This is used to set the direction
	 * the robot is currently facing as the "forward" direction.
	 * <p>
	 * This method does NOT recalibrate the navX, it just adds an offset in the
	 * software to make the angle zero. For more information on navX calibration:
	 * https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/
	 * <p>
	 * When the navX is parallel to the floor and was calibrated parallel to the
	 * floor, "yaw angle" is the angle of the board-relative Y axis. If the navX is
	 * not mounted parallel to the floor, follow the instructions on this website:
	 * https://pdocs.kauailabs.com/navx-mxp/installation/omnimount/
	 */
	public void resetYaw() {
		this.navx.zeroYaw();
		// Reset the odometry to it's current position, but with a different
		// angle (the angle is 0).
		this.odometry.resetPosition(this.getCurrentPose(), new Rotation2d());
	}

	/**
	 * Returns the negated yaw angle (360 - angle) as a Rotation2d.
	 * The angle is negated because the WPILib convention is that the
	 * angle increases as the robot turns clockwise, and the navX does
	 * the opposite of that.
	 * <p>
	 * When the navX is parallel to the floor and was calibrated parallel to the
	 * floor, "yaw angle" is the angle of the board-relative Y axis. If the navX is
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
		this.frontLeftModule.setSteerMotor(DrivetrainConstants.kFrontLeftCrossAngleDeg);
		this.frontRightModule.setDriveMotor(0);
		this.frontRightModule.setSteerMotor(DrivetrainConstants.kFrontRightCrossAngleDeg);
		this.backLeftModule.setDriveMotor(0);
		this.backLeftModule.setSteerMotor(DrivetrainConstants.kBackLeftCrossAngleDeg);
		this.backRightModule.setDriveMotor(0);
		this.backRightModule.setSteerMotor(DrivetrainConstants.kBackRightCrossAngleDeg);
	}

	/**
	 * Returns a Pose2d object representing the robot's current position.
	 * X in meters, Y in meters, angle in Rotation2d.
	 */
	public Pose2d getCurrentPose() {
		return this.odometry.getPoseMeters();
	}

	/**
	 * Discards the odometry measurments and sets the values to X zero,
	 * Y zero, angle stays the same.
	 */
	public void resetOdometry() {
		this.odometry.resetPosition(new Pose2d(), this.getGyroRotation());
	}

	public double getChassisForwardAccelMPSSquared() {
		return this.navx.getWorldLinearAccelY() * DrivetrainConstants.kGravityToMPSSquaredRatio;
	}

	public double getChassisLateralAccelMPSSquared() {
		return this.navx.getWorldLinearAccelX() * DrivetrainConstants.kGravityToMPSSquaredRatio;
	}

	@Override
	public void periodic() {
		// The odometry must be updated periodically, in
		// order to accuratly track the robot's position.
		this.odometry.update(
				this.getGyroRotation(),
				this.empiricalStates[0],
				this.empiricalStates[1],
				this.empiricalStates[2],
				this.empiricalStates[3]);

		// If the robot isn't moving for more than a second
		// (five iterations), sync the encoders.
		if (this.chassisSpeeds.vxMetersPerSecond == 0 &&
				this.chassisSpeeds.vyMetersPerSecond == 0 &&
				this.chassisSpeeds.omegaRadiansPerSecond == 0 &&
				this.encoderSyncTimer.get() >= 1) {
			this.frontLeftModule.syncSteerEncoder();
			this.frontRightModule.syncSteerEncoder();
			this.backLeftModule.syncSteerEncoder();
			this.backRightModule.syncSteerEncoder();
			this.encoderSyncTimer.reset();
		}
		// if the robot is moving, reset the timer
		else if (this.chassisSpeeds.vxMetersPerSecond != 0 &&
				this.chassisSpeeds.vyMetersPerSecond != 0 &&
				this.chassisSpeeds.omegaRadiansPerSecond != 0) {
			this.encoderSyncTimer.reset();
		}

		// Update shuffleboard entries...
		this.frontLeftIntegratedSensorEntry.setDouble(this.frontLeftModule.getAbsWheelAngle());
		this.frontLeftAbsAngleEntry.setDouble(this.frontLeftModule.getAbsWheelAngle());
		this.frontRightAbsAngleEntry.setDouble(this.frontRightModule.getAbsWheelAngle());
		this.backLeftAbsAngleEntry.setDouble(this.backLeftModule.getAbsWheelAngle());
		this.backRightAbsAngleEntry.setDouble(this.backRightModule.getAbsWheelAngle());
		this.ox.setDouble(this.getCurrentPose().getX());
		this.oy.setDouble(this.getCurrentPose().getY());
		this.field.setRobotPose(this.odometry.getPoseMeters());
	}
}
