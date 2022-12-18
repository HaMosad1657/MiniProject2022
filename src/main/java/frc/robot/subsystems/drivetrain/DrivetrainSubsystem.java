package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import com.hamosad1657.lib.sensors.HaNavX;
import com.hamosad1657.lib.swerve.HaSwerveModule;
import com.hamosad1657.lib.swerve.HaSwerveSubsystem;

public class DrivetrainSubsystem {
	private static HaNavX navX = new HaNavX(SerialPort.Port.kUSB1);

	private static HaSwerveModule[] modules = new HaSwerveModule[] {
			new HaSwerveModule(
					DrivetrainConstants.kFrontLeftSteerMotorID,
					DrivetrainConstants.kFrontLeftDriveMotorID,
					DrivetrainConstants.kFrontLeftCANCoderID,
					DrivetrainConstants.kFrontLeftAngleOffset,
					DrivetrainConstants.kWheelRadiusM,
					DrivetrainConstants.kSteerPidGains,
					DrivetrainConstants.kDrivePidGains),
			new HaSwerveModule(
					DrivetrainConstants.kFrontRightSteerMotorID,
					DrivetrainConstants.kFrontRightDriveMotorID,
					DrivetrainConstants.kFrontRightCANCoderID,
					DrivetrainConstants.kFrontRightAngleOffset,
					DrivetrainConstants.kWheelRadiusM,
					DrivetrainConstants.kSteerPidGains,
					DrivetrainConstants.kDrivePidGains),
			new HaSwerveModule(
					DrivetrainConstants.KBackLeftSteerMotorID,
					DrivetrainConstants.kBackLeftDriveMotorID,
					DrivetrainConstants.kBackLeftCANCoderID,
					DrivetrainConstants.kBackLeftAngleOffset,
					DrivetrainConstants.kWheelRadiusM,
					DrivetrainConstants.kSteerPidGains,
					DrivetrainConstants.kDrivePidGains),
			new HaSwerveModule(
					DrivetrainConstants.kBackRightSteerMotorID,
					DrivetrainConstants.kBackRightDriveMotorID,
					DrivetrainConstants.kBackRightCANCoderID,
					DrivetrainConstants.kBackRightAngleOffset,
					DrivetrainConstants.kWheelRadiusM,
					DrivetrainConstants.kSteerPidGains,
					DrivetrainConstants.kDrivePidGains) };


	private static HaSwerveSubsystem swerveSubsystem = new HaSwerveSubsystem(
			new Pose2d(),
			navX,
			modules,
			DrivetrainConstants.kDrivetrainTrackWidthM,
			DrivetrainConstants.kMaxChassisVelocityMPS);

	public static HaNavX getNavX() {
		return navX;
	}

	public static HaSwerveSubsystem getSwerveSubsytem() {
		return swerveSubsystem;
	}
}
