package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.chassis.HaNavX;
import frc.robot.subsystems.chassis.HaSwerveLib.HaSwerveModule;
import frc.robot.subsystems.chassis.HaSwerveLib.HaSwerveSubsystem;

public class HaSwerveSubsystemContainer {
	private static HaNavX navX = HaNavX.getInstance(SerialPort.Port.kUSB1);

	private static HaSwerveModule[] modules = new HaSwerveModule[] {
			new HaSwerveModule(
					DrivetrainConstants.kFrontLeftSteerMotorID,
					DrivetrainConstants.kFrontLeftDriveMotorID,
					DrivetrainConstants.kFrontLeftCANCoderID,
					DrivetrainConstants.kFrontLeftAngleOffset,
					DrivetrainConstants.kWheelDiameterCM),
			new HaSwerveModule(
					DrivetrainConstants.kFrontRightSteerMotorID,
					DrivetrainConstants.kFrontRightDriveMotorID,
					DrivetrainConstants.kFrontRightCANCoderID,
					DrivetrainConstants.kFrontRightAngleOffset,
					DrivetrainConstants.kWheelDiameterCM),
			new HaSwerveModule(
					DrivetrainConstants.KBackLeftSteerMotorID,
					DrivetrainConstants.kBackLeftDriveMotorID,
					DrivetrainConstants.kBackLeftCANCoderID,
					DrivetrainConstants.kBackLeftAngleOffset,
					DrivetrainConstants.kWheelDiameterCM),
			new HaSwerveModule(
					DrivetrainConstants.kBackRightSteerMotorID,
					DrivetrainConstants.kBackRightDriveMotorID,
					DrivetrainConstants.kBackRightCANCoderID,
					DrivetrainConstants.kBackRightAngleOffset,
					DrivetrainConstants.kWheelDiameterCM) };
	
	private static HaSwerveSubsystem swerve = new HaSwerveSubsystem(
			new Pose2d(),
			navX,
			modules,
			DrivetrainConstants.kDrivetrainTrackWidthM,
			DrivetrainConstants.kMaxChassisVelocityMPS);
	
	public static HaNavX getNavX() {
		return navX;
	}

	public static HaSwerveSubsystem getSwerveSubsytem() {
		return swerve;
	}
}
