package frc.robot.subsystems.chassis;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

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

	private CANCoder frontLeftCANCoder;
	private CANCoder frontRightCANCoder;
	private CANCoder backLeftCANCoder;
	private CANCoder backRightCANCoder;

	private TalonFX frontLeftDrive;
	private TalonFX frontRightDrive;
	private TalonFX backLeftDrive;
	private TalonFX backRightDrive;

	private TalonFX frontLeftSteer;
	private TalonFX frontRightSteer;
	private TalonFX backLeftSteer;
	private TalonFX backRightSteer;

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
			frontLeftAbsAngleEntry, frontRightAbsAngleEntry, backLeftAbsAngleEntry, backRightAbsAngleEntry;
	private final Field2d field;
	private final AHRS navx;

	/**
	 * Initialize + configure TalonFXs and CANCoders,
	 * initialize Shuffleboard tabs and entries,
	 * initialize kinematics and odometry,
	 * Start communication with navX via USB
	 * and wait for it to finish startup calibration.
	 */
	private DrivetrainSubsystem() {

		// Construct the CANCoders
		this.frontLeftCANCoder = new CANCoder(DrivetrainConstants.kFrontLeftCANCoderID);
		this.frontRightCANCoder = new CANCoder(DrivetrainConstants.kFrontRightCANCoderID);
		this.backLeftCANCoder = new CANCoder(DrivetrainConstants.kBackLeftCANCoderID);
		this.backRightCANCoder = new CANCoder(DrivetrainConstants.kBackRightCANCoderID);

		// Make the CANCoders boot to their absloute value instead of to 0
		this.frontLeftCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		this.frontRightCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		this.backLeftCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		this.backRightCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

		// Set the CANCoder measurment coefficient to 0.087890625 so that it returns degrees
		// (this is the default)
		this.frontLeftCANCoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
		this.frontRightCANCoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
		this.backLeftCANCoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
		this.backRightCANCoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);

		// Set CANCoder offsets
		this.frontLeftCANCoder.configMagnetOffset(DrivetrainConstants.kFrontLeftAngleOffset);
		this.frontRightCANCoder.configMagnetOffset(DrivetrainConstants.kFrontRightAngleOffset);
		this.backLeftCANCoder.configMagnetOffset(DrivetrainConstants.kBackLeftAngleOffset);
		this.backRightCANCoder.configMagnetOffset(DrivetrainConstants.kBackRightAngleOffset);

		// Make the CANCoders return measurments in 0 to 360.
		// NOTE: the position closed-loop knows that 0 and 360
		// degrees correspond the same point in reality! It is
		// similar to enableContinousInput() method from the
		// WPIlib PIDController class.
		this.frontLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.frontRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.backLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.backRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

		// Constrct the drive motor controllers
		this.frontLeftDrive = new TalonFX(DrivetrainConstants.kFrontLeftDriveMotorID);
		this.frontRightDrive = new TalonFX(DrivetrainConstants.kFrontRightDriveMotorID);
		this.backLeftDrive = new TalonFX(DrivetrainConstants.kBackLeftDriveMotorID);
		this.backRightDrive = new TalonFX(DrivetrainConstants.kBackRightDriveMotorID);

		// Set the drive motors to brake neutral mode
		this.frontLeftDrive.setNeutralMode(NeutralMode.Brake);
		this.frontRightDrive.setNeutralMode(NeutralMode.Brake);
		this.backLeftDrive.setNeutralMode(NeutralMode.Brake);
		this.backRightDrive.setNeutralMode(NeutralMode.Brake);

		// Set the feedback devices for the drive motor controllers as their
		// integrated encoders
		this.frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.backLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.backRightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		// Construct the steer motor controllers
		this.frontLeftSteer = new TalonFX(DrivetrainConstants.kFrontLeftAngleMotorID);
		this.frontRightSteer = new TalonFX(DrivetrainConstants.kFrontRightAngleMotorID);
		this.backLeftSteer = new TalonFX(DrivetrainConstants.KBackLeftAngleMotorID);
		this.backRightSteer = new TalonFX(DrivetrainConstants.kBackRightAngleMotorID);

		// Set the steer motors to brake neutral mode
		this.frontLeftSteer.setNeutralMode(NeutralMode.Brake);
		this.frontRightSteer.setNeutralMode(NeutralMode.Brake);
		this.backLeftSteer.setNeutralMode(NeutralMode.Brake);
		this.backRightSteer.setNeutralMode(NeutralMode.Brake);

		// Set the feedback devices for the steer motor controllers as their respective
		// CANCoders.
		this.frontLeftSteer.configRemoteFeedbackFilter(
				DrivetrainConstants.kFrontLeftCANCoderID,
				RemoteSensorSource.CANCoder, 0);
		this.frontRightSteer.configRemoteFeedbackFilter(
				DrivetrainConstants.kFrontRightCANCoderID,
				RemoteSensorSource.CANCoder, 0);
		this.backLeftSteer.configRemoteFeedbackFilter(
				DrivetrainConstants.kBackLeftCANCoderID,
				RemoteSensorSource.CANCoder, 0);
		this.backRightSteer.configRemoteFeedbackFilter(
				DrivetrainConstants.kBackRightCANCoderID,
				RemoteSensorSource.CANCoder, 0);
		this.frontLeftSteer.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		this.frontRightSteer.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		this.backLeftSteer.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		this.backRightSteer.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

		/*
		 * Every iteration, the Proportional gain is multiplied by the closed-loop
		 * error (the error is in raw sensor units or analog value when controlling
		 * position, or raw sensor units per 100 miliseconds when controlling velocity).
		 * Note that the MAX final output value is 1023 (or -1023 in the reverse
		 * directon), the integrated encoder has 2048 counts per revolution, and
		 * the CANCoder has 4096 counts per revolution.
		 * So if you want max output for an error of one full encoder revolution,
		 * set the proportional gain as 1023 / 2048 if using the integrated encoder
		 * for feedback, or 1023 / 4096 if using the CANCoder for feedback.
		 */

		// Set PID gains for the DRIVE motor controllers on PID slot 0
		this.frontLeftDrive.config_kP(0, 0.00);
		this.frontRightDrive.config_kP(0, 0.00);
		this.backLeftDrive.config_kP(0, 0.00);
		this.backRightDrive.config_kP(0, 0.00);

		this.frontLeftDrive.config_kI(0, 0.0000);
		this.frontRightDrive.config_kI(0, 0.0000);
		this.backLeftDrive.config_kI(0, 0.0000);
		this.backRightDrive.config_kI(0, 0.0000);

		this.frontLeftDrive.config_kD(0, 00);
		this.frontRightDrive.config_kD(0, 00);
		this.backLeftDrive.config_kD(0, 00);
		this.backRightDrive.config_kD(0, 00);

		// Set PID gains for the STEER motor controllers on PID slot 0
		this.frontLeftSteer.config_kP(0, 0.01);
		this.frontRightSteer.config_kP(0, 0.01);
		this.backLeftSteer.config_kP(0, 0.01);
		this.backRightSteer.config_kP(0, 0.01);

		this.frontLeftSteer.config_kI(0, 0.0);
		this.frontRightSteer.config_kI(0, 0.0);
		this.backLeftSteer.config_kI(0, 0.0);
		this.backRightSteer.config_kI(0, 0.0);

		this.frontLeftSteer.config_kD(0, 0.0);
		this.frontRightSteer.config_kD(0, 0.0);
		this.backLeftSteer.config_kD(0, 0.0);
		this.backRightSteer.config_kD(0, 0.0);

		this.debuggingTab = Shuffleboard.getTab("Debugging");

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
	}

	@Override
	public void periodic() {
		// The odometry must be updated periodically, in order to accurately track the
		// robot's position.
		this.odometry.update(this.getGyroRotation(), this.states[0], this.states[1], this.states[2], this.states[3]);

		// Update shuffleboard entries...
		this.frontLeftAbsAngleEntry.setDouble(this.frontLeftCANCoder.getAbsolutePosition());
		this.frontRightAbsAngleEntry.setDouble(this.frontRightCANCoder.getAbsolutePosition());
		this.backLeftAbsAngleEntry.setDouble(this.backLeftCANCoder.getAbsolutePosition());
		this.backRightAbsAngleEntry.setDouble(this.backRightCANCoder.getAbsolutePosition());
		this.ox.setDouble(this.getCurrentPose().getX());
		this.oy.setDouble(this.getCurrentPose().getY());
		this.field.setRobotPose(this.odometry.getPoseMeters());
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
				Rotation2d.fromDegrees(this.frontLeftCANCoder.getAbsolutePosition()));
		optimizedStates[1] = SwerveModuleState.optimize(this.states[1],
				Rotation2d.fromDegrees(this.frontRightCANCoder.getAbsolutePosition()));
		optimizedStates[2] = SwerveModuleState.optimize(this.states[2],
				Rotation2d.fromDegrees(this.backLeftCANCoder.getAbsolutePosition()));
		optimizedStates[3] = SwerveModuleState.optimize(this.states[3],
				Rotation2d.fromDegrees(this.backRightCANCoder.getAbsolutePosition()));

		this.states = optimizedStates;

		// If any of the setpoints are over the max speed, this method lowers
		// all of them (in the same ratio).
		SwerveDriveKinematics.desaturateWheelSpeeds(this.states, DrivetrainConstants.kMaxChassisVelocityMPS);

		// Front left
		this.frontLeftDrive.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(this.states[0].speedMetersPerSecond));
		this.frontLeftSteer.set(ControlMode.Position,
				this.states[0].angle.getDegrees() * DrivetrainConstants.kCANCoderTicksPerDegree);

		// Front right
		this.frontRightDrive.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(this.states[1].speedMetersPerSecond));
		this.frontRightSteer.set(ControlMode.Position,
				this.states[1].angle.getDegrees() * DrivetrainConstants.kCANCoderTicksPerDegree);

		// Back left
		this.backLeftDrive.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(this.states[2].speedMetersPerSecond));
		this.backLeftSteer.set(ControlMode.Position,
				this.states[2].angle.getDegrees() * DrivetrainConstants.kCANCoderTicksPerDegree);

		// Back right
		this.backRightDrive.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(this.states[3].speedMetersPerSecond));
		this.backRightSteer.set(ControlMode.Position,
				this.states[3].angle.getDegrees() * DrivetrainConstants.kCANCoderTicksPerDegree);
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
	 * <p>
	 * Does not work if drive() is called and passed dontRotateInZero false.
	 */
	public void crossLockWheels() {
		this.frontLeftDrive.set(ControlMode.Velocity, 0);
		this.frontLeftSteer.set(ControlMode.Position, DrivetrainConstants.kFrontLeftCrossAngleRadians);
		this.frontRightDrive.set(ControlMode.Velocity, 0);
		this.frontRightSteer.set(ControlMode.Position, DrivetrainConstants.kFrontRightCrossAngleRadians);
		this.backLeftDrive.set(ControlMode.Velocity, 0);
		this.backLeftSteer.set(ControlMode.Position, DrivetrainConstants.kBackLeftCrossAngleRadians);
		this.backRightDrive.set(ControlMode.Velocity, 0);
		this.backRightSteer.set(ControlMode.Position, DrivetrainConstants.kBackRightCrossAngleRadians);
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

	/** Math verified by Noam Geva and Ma'ayan Fucking Bar-El✨ */
	private double MPSToIntegratedEncoderTicksPer100MS(double metersPerSecond) {
		double wheelRotationsPerSec = metersPerSecond / DrivetrainConstants.kWheelCircumferenceCM;
		double motorRotationsPerSec = wheelRotationsPerSec / SdsModuleConfigurations.MK4_L2.getDriveReduction();
		double encoderCountsPerSec = motorRotationsPerSec *
				(DrivetrainConstants.kIntegratedEncoderTicksPerRev);
		double encoderCountsPer100MS = encoderCountsPerSec / 10;
		return encoderCountsPer100MS;
	}
}
