package frc.robot.subsystems.chassis;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

	private static DrivetrainSubsystem instance;

	public static DrivetrainSubsystem getInstance() {
		if (instance == null)
			instance = new DrivetrainSubsystem();
		return instance;
	}

	// MUST. BE. RADIANS!
	private double frontLeftPreviousRotation = 0;
	private double frontRightPreviousRotation = 0;
	private double backLeftPreviousRotation = 0;
	private double backRightPreviousRotation = 0;

	private SwerveModuleState[] states;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;
	private final AHRS navx;
	private final ShuffleboardTab tab, tab2;
	private final NetworkTableEntry ox, oy, gysin;// gyro angle sin

	private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

	private ChassisSpeeds chassisSpeeds;

	private DrivetrainSubsystem() {
		this.tab = Shuffleboard.getTab("Chassis");
		this.tab2 = Shuffleboard.getTab("Calibrating PID");
		this.ox = tab2.add("odometry x axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.oy = tab2.add("odometry y axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.gysin = tab2.add("gyro sin angle", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		this.kinematics = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				// Front right
				new Translation2d(DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				// Back left
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
				// Back right
				new Translation2d(-DrivetrainConstants.kDrivetrainTrackWidthMeters / 2.0,
						-DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0));

		this.frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				this.tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DrivetrainConstants.kFrontLeftDriveMotorID,
				DrivetrainConstants.kFrontLeftAngleMotorID,
				DrivetrainConstants.kFrontLeftAngleEncoderID,
				// This is how much the steer encoder is offset from true zero (zero is facing
				// straight forward)
				DrivetrainConstants.kFrontLeftAngleOffset);

		this.frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
				this.tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DrivetrainConstants.kFrontRightDriveMotorID,
				DrivetrainConstants.kFrontRightAngleMotorID,
				DrivetrainConstants.kFrontRightAngleEncoderID,
				DrivetrainConstants.kFrontRightAngleOffset);

		this.backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				this.tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DrivetrainConstants.kBackLeftDriveMotorID,
				DrivetrainConstants.KBackLeftAngleMotorID,
				DrivetrainConstants.kBackLeftAngleEncoderID,
				DrivetrainConstants.kBackLeftAngleOffset);

		this.backRightModule = Mk4SwerveModuleHelper.createFalcon500(
				this.tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DrivetrainConstants.kBackRightDriveMotorID,
				DrivetrainConstants.kBackRightAngleMotorID,
				DrivetrainConstants.kBackRightAngleEncoderID,
				DrivetrainConstants.kBackRightAngleOffset);

		// Start communication between the navX and RoboRIO using the USB port.
		this.navx = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 60);
		this.navx.enableLogging(true);
		
		if (this.navx.isConnected()) {
			DriverStation.reportError("Connected to navX", false);
		}
		if (this.navx.isCalibrating()) {
			DriverStation.reportError("navX calibrating", false);
		}

		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		this.odometry = new SwerveDriveOdometry(this.kinematics, this.getGyroRotation());
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the "forward" direction.
	 */
	public void resetYaw() {
		this.navx.zeroYaw();
	}

	public Rotation2d getGyroRotation() {
		if (this.navx.isMagnetometerCalibrated()) {
			DriverStation.reportError("navx magnetometer is calibrated", false);
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(this.navx.getFusedHeading());
		}
		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - this.navx.getYaw());
		//return Rotation2d.fromDegrees(360.0 - this.navx.getRawGyroY());

	}

	public double getChassisForwardAccelMPSSquared() {
		return this.navx.getWorldLinearAccelY() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}

	public double getChassisLateralAccelMPSSquared() {
		return this.navx.getWorldLinearAccelX() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}

	public void drive(ChassisSpeeds chassisSpeeds, boolean dontRotateInZero) {
		this.chassisSpeeds = chassisSpeeds;
		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(this.states, DrivetrainConstants.kMaxChassisVelocityMPS);

		if(this.chassisSpeeds.vxMetersPerSecond == 0 && this.chassisSpeeds.vyMetersPerSecond == 0 && this.chassisSpeeds.omegaRadiansPerSecond == 0 && dontRotateInZero) {
			this.frontLeftModule.set(
				this.states[0].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.frontLeftPreviousRotation);

			this.frontRightModule.set(
				this.states[1].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.frontRightPreviousRotation);

			this.backLeftModule.set(
				this.states[2].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.backLeftPreviousRotation);

			this.backRightModule.set(
				this.states[3].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.backRightPreviousRotation);
		}
		else {
			this.frontLeftModule.set(
				this.states[0].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.states[0].angle.getRadians());
			this.frontLeftPreviousRotation = this.states[0].angle.getRadians();

			this.frontRightModule.set(
				this.states[1].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.states[1].angle.getRadians());
			this.frontRightPreviousRotation = this.states[1].angle.getRadians();

			this.backLeftModule.set(
				this.states[2].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.states[2].angle.getRadians());
			this.backLeftPreviousRotation = this.states[2].angle.getRadians();

			this.backRightModule.set(
				this.states[3].speedMetersPerSecond / DrivetrainConstants.kMaxChassisVelocityMPS
						* DrivetrainConstants.kMaxVoltage,
				this.states[3].angle.getRadians());
			this.backRightPreviousRotation = this.states[3].angle.getRadians();
		}
	}

	@Override
	public void periodic() {
    this.odometry.update(this.getGyroRotation(), this.states[0], this.states[1], this.states[2], this.states[3]);
		this.ox.setDouble(getCurretnPose().getX());
		this.oy.setDouble(getCurretnPose().getY());
		this.gysin.setDouble(getGyroRotation().getSin());
	}

	public Pose2d getCurretnPose() {
		return this.odometry.getPoseMeters();
	}
}
