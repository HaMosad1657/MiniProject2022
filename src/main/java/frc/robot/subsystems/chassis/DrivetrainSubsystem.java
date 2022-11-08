package frc.robot.subsystems.chassis;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

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
	 * An array of SwerveModuleStates objects (one for each module),
	 * ordered:
	 * <ul>
	 * <li>front left [0]
	 * <li>front right [1]
	 * <li>back left [2]
	 * <li>back right [3]
	 */
	private SwerveModuleState[] states;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;
	private final ShuffleboardTab chassisTab, odometryTab, fieldTab;
	private final NetworkTableEntry ox, oy,
			frontLeftAbsAngleEntry, frontRightAbsAngleEntry, backLeftAbsAngleEntry, backRightAbsAngleEntry;
	private final Field2d field;
	private final AHRS navx;

	private ChassisSpeeds chassisSpeeds;

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

		// Set CANCoder offsets
		this.frontLeftCANCoder.configMagnetOffset(DrivetrainConstants.kFrontLeftAngleOffset);
		this.frontRightCANCoder.configMagnetOffset(DrivetrainConstants.kFrontRightAngleOffset);
		this.backLeftCANCoder.configMagnetOffset(DrivetrainConstants.kBackLeftAngleOffset);
		this.backRightCANCoder.configMagnetOffset(DrivetrainConstants.kBackRightAngleOffset);

		// Make the CANCoders return measurments in 0 to 360
		this.frontLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.frontRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.backLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.backRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

		// Constrct the drive motor controllers
		this.frontLeftDrive = new TalonFX(DrivetrainConstants.kFrontLeftDriveMotorID);
		this.frontRightDrive = new TalonFX(DrivetrainConstants.kFrontRightDriveMotorID);
		this.backLeftDrive = new TalonFX(DrivetrainConstants.kBackLeftDriveMotorID);
		this.backRightDrive = new TalonFX(DrivetrainConstants.kBackRightDriveMotorID);

		// Set the feedback devices for the drive motor controllers as their
		// integrated encoders
		this.frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.backLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.backRightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		// Set PID gains for the drive motor controllers on PID slot 0
		this.frontLeftDrive.config_kP(0, 0.2);
		this.frontRightDrive.config_kP(0, 0.2);
		this.backLeftDrive.config_kP(0, 0.2);
		this.backRightDrive.config_kP(0, 0.2);
		this.frontLeftDrive.config_kI(0, 0.0002);
		this.frontRightDrive.config_kI(0, 0.0002);
		this.backLeftDrive.config_kI(0, 0.0002);
		this.backRightDrive.config_kI(0, 0.0002);
		this.frontLeftDrive.config_kD(0, 20);
		this.frontRightDrive.config_kD(0, 20);
		this.backLeftDrive.config_kD(0, 20);
		this.backRightDrive.config_kD(0, 20);


		// Construct the steer motor controllers
		this.frontLeftSteer = new TalonFX(DrivetrainConstants.kFrontLeftAngleMotorID);
		this.frontRightSteer = new TalonFX(DrivetrainConstants.kFrontRightAngleMotorID);
		this.backLeftSteer = new TalonFX(DrivetrainConstants.KBackLeftAngleMotorID);
		this.backRightSteer = new TalonFX(DrivetrainConstants.kBackRightAngleMotorID);

		// Set the feedback devices for the steer motor controllers as CANCoders
		this.frontLeftSteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.frontRightSteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.backLeftSteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.backRightSteer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		// Set PID gains for the drive motor controllers on PID slot 0
		this.frontLeftSteer.config_kP(0, 0.2);
		this.frontRightSteer.config_kP(0, 0.2);
		this.backLeftSteer.config_kP(0, 0.2);
		this.backRightSteer.config_kP(0, 0.2);
		this.frontLeftSteer.config_kI(0, 0.0002);
		this.frontRightSteer.config_kI(0, 0.0002);
		this.backLeftSteer.config_kI(0, 0.0002);
		this.backRightSteer.config_kI(0, 0.0002);
		this.frontLeftSteer.config_kD(0, 20);
		this.frontRightSteer.config_kD(0, 20);
		this.backLeftSteer.config_kD(0, 20);
		this.backRightSteer.config_kD(0, 20);

		this.field = new Field2d();
		this.fieldTab = Shuffleboard.getTab("Field");
		this.fieldTab.add("Field", this.field);

		this.chassisTab = Shuffleboard.getTab("Chassis");
		this.frontLeftAbsAngleEntry = this.chassisTab.add(
				"Front Left Absloute Angle", 0.0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
		this.frontRightAbsAngleEntry = this.chassisTab.add(
				"Front Right Absloute Angle", 0.0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
		this.backLeftAbsAngleEntry = this.chassisTab.add(
				"Back Left Absloute Angle", 0.0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
		this.backRightAbsAngleEntry = this.chassisTab.add(
				"Back Right Absloute Angle", 0.0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

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
			// // Wait for the navx to finish the startup calibration - loop
			// overrun is okay here, because this all happens in robotInit().
			// Waiting is important because if the robot moves when the navX
			// is calibrating, it can return inaccurate measurments.
		}
		DriverStation.reportError("navX done calibrating", false);
		// Add the navx widget to the shuffleboard
		this.odometryTab.add(this.navx);

		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		// Construct a SwerveDriveOdometry with X=0, Y=0, rotation=0
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
		this.chassisSpeeds = chassisSpeeds;
		// Turns the desired speed of the entire chassis into speed and angle
		// setpoints for the individual modules, and optimizes them to not rotate
		// more than 90 degrees
		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		this.states[0] = SwerveModuleState.optimize(this.states[0],
				Rotation2d.fromDegrees(this.frontLeftCANCoder.getAbsolutePosition()));
		this.states[1] = SwerveModuleState.optimize(this.states[1],
				Rotation2d.fromDegrees(this.frontRightCANCoder.getAbsolutePosition()));
		this.states[2] = SwerveModuleState.optimize(this.states[2],
				Rotation2d.fromDegrees(this.backLeftCANCoder.getAbsolutePosition()));
		this.states[3] = SwerveModuleState.optimize(this.states[3],
				Rotation2d.fromDegrees(this.backRightCANCoder.getAbsolutePosition()));
		// If any of the setpoints are over the max speed, it lowers all of
		// them (in the same ratio).
		SwerveDriveKinematics.desaturateWheelSpeeds(this.states, DrivetrainConstants.kMaxChassisVelocityMPS);

		this.frontLeftDrive.set(ControlMode.Velocity, this.MPSToEncoderCounts(this.states[0].speedMetersPerSecond));
		this.frontLeftSteer.set(ControlMode.Position, this.degreesToEncoderCounts(this.states[0].angle.getDegrees()));

		this.frontRightDrive.set(ControlMode.Velocity, this.MPSToEncoderCounts(this.states[1].speedMetersPerSecond));
		this.frontRightSteer.set(ControlMode.Position, this.degreesToEncoderCounts(this.states[1].angle.getDegrees()));

		this.backLeftDrive.set(ControlMode.Velocity, this.MPSToEncoderCounts(this.states[2].speedMetersPerSecond));
		this.backLeftSteer.set(ControlMode.Position, this.degreesToEncoderCounts(this.states[2].angle.getDegrees()));

		this.backRightDrive.set(ControlMode.Velocity, this.MPSToEncoderCounts(this.states[3].speedMetersPerSecond));
		this.backRightSteer.set(ControlMode.Position, this.degreesToEncoderCounts(this.states[3].angle.getDegrees()));
	}

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

	private double MPSToEncoderCounts(double metersPerSecond) {
		return mapRange(0, DrivetrainConstants.kMaxChassisVelocityMPS,
				0, DrivetrainConstants.kEncoderTicks, metersPerSecond);
	}

	private double degreesToEncoderCounts(double angleDegrees) {
		return mapRange(0, 360, 0, DrivetrainConstants.kEncoderTicks, angleDegrees);
	}

	private double mapRange(double oldMin, double oldMax, double newMin, double newMax, double value) {
		return ((value - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;
	}
}
