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

	// In radians
	private double frontLeftPreviousRotation = 0;
	private double frontRightPreviousRotation = 0;
	private double backLeftPreviousRotation = 0;
	private double backRightPreviousRotation = 0;

	private SwerveModuleState[] states;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;
	private final ShuffleboardTab tab, tab2;
	private final NetworkTableEntry ox, oy;
	private final AHRS navx;

	private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

	private ChassisSpeeds chassisSpeeds;

	private DrivetrainSubsystem() {
		this.tab = Shuffleboard.getTab("Chassis");
		this.tab2 = Shuffleboard.getTab("Odometry");
		this.ox = this.tab2.add("odometry x axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.oy = this.tab2.add("odometry y axis", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

		this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		this.kinematics = new SwerveDriveKinematics(
				// Front left (problematic)
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
				// This is how much the steer encoder is offset from true zero (zero is
				// the wheels pointing forwards, with the bevel gear facing to the right).
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

		// Start communication between the navX and RoboRIO using the
		// outer USB-A port on the RoboRIO. 
		this.navx = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte)60);
		this.navx.enableLogging(true);
		// Wait for the navx to finish the startup calibration - loop
		// overrun is okay here, because this all happens in robotInit().
		while(this.navx.isCalibrating()) {}
		DriverStation.reportError("navX done calibrating", false);
		// Add the navx widget to the shuffleboard
		this.tab2.add(this.navx);

		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		// Construct a SwerveDriveOdometry with X=0, Y=0, rotation=0
		this.odometry = new SwerveDriveOdometry(this.kinematics, this.getGyroRotation());
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction
	 * the robot is currently facing as the "forward" direction.
	 * <p>
	 * This method does NOT recalibrate the navX, it just adds an offset in the
	 * software to make the angle zero.
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
		// getRawGyroY() returns the same value as getYaw(), but with
		// USB you can use either raw data or processed data, not both.
		// So we use getYaw().
	}

	public double getChassisForwardAccelMPSSquared() {
		return this.navx.getWorldLinearAccelY() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}

	public double getChassisLateralAccelMPSSquared() {
		return this.navx.getWorldLinearAccelX() * DrivetrainConstants.kGravityToMPSSquaredConversionFactor;
	}

	/**
	 * By default, the SDS library sets the modules's angles to zero, which
	 * makes the wheels point forwards) when all the modules are set to 0,0
	 * (aka, the chassis isn't moving).
	 * In order to not do that, pass dontRotateInZero true.
	 * <p>
	 * @param chassisSpeeds
	 * @param dontRotateInZero
	 */
	public void drive(ChassisSpeeds chassisSpeeds, boolean dontRotateInZero) {
		this.chassisSpeeds = chassisSpeeds;
		this.states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(this.states, DrivetrainConstants.kMaxChassisVelocityMPS);

		// If all chassisSpeeds fields are 0, and dontRotateInZero is true,
		// then set the wheel speed to 0 and the angle to the last specified angle.
		// This is a workaround for SDS automatically setting the angle to
		// zero when the chassis isn't moving.
		if (this.chassisSpeeds.vxMetersPerSecond == 0 && this.chassisSpeeds.vyMetersPerSecond == 0
				&& this.chassisSpeeds.omegaRadiansPerSecond == 0 && dontRotateInZero) {

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
			// If one or more of chassisSpeeds fields is nonzero, or if dontRotateInZero is false,
			// then set the wheel speed to the specified speed and the angle to the specified angle.
		} else {
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
	
	/**
	 * Turns the modules so that they make an x shape, untill they are
	 * told to do something else.
	 */
	public void crossLockChassis() {
		this.frontLeftModule.set(0, DrivetrainConstants.kFrontLeftCrossAngleRadians);
		this.frontLeftPreviousRotation = DrivetrainConstants.kFrontLeftCrossAngleRadians;

		this.frontRightModule.set(0, DrivetrainConstants.kFrontRightCrossAngleRadians);
		this.frontRightPreviousRotation = DrivetrainConstants.kFrontRightCrossAngleRadians;

		this.backLeftModule.set(0, DrivetrainConstants.kBackLeftCrossAngleRadians);
		this.backLeftPreviousRotation = DrivetrainConstants.kBackLeftCrossAngleRadians;
		
		this.backRightModule.set(0, DrivetrainConstants.kBackRightCrossAngleRadians);
		this.backRightPreviousRotation = DrivetrainConstants.kBackRightCrossAngleRadians;
	}

	@Override
	public void periodic() {
		// The odometry must be updated periodically, in order to accurately track the robot's position.
		this.odometry.update(this.getGyroRotation(), this.states[0], this.states[1], this.states[2], this.states[3]);

		// Update shuffleboard entries...
		this.ox.setDouble(this.getCurretnPose().getX());
		this.oy.setDouble(this.getCurretnPose().getY());
	}

	/**
	 * Returns a Pose2d object representing the robot's current position.
	 * X in meters, Y in meters, angle in Rotation2d.
	 */
	public Pose2d getCurretnPose() {
		return this.odometry.getPoseMeters();
	}
}
