package frc.robot.subsystems.disks;

public class DisksConstants {
	public static final int kAngleMotorID = 20;
	public static final int kTelescopicMotorID = 19;
	public static final int kGrabberMotorID = 13;
	public static final int kGrabberEncoderID = 14;

	public static final double kGrabberOpenedPosition = 180, kGrabberClosedPosition = 90; // Opened - Disk is grabbed, Closed - Disk is released
	public static final double kGrabberP = 0.0055, kGrabberI = 0.0, kGrabberD = 0.0;
	public static final double kGrabberTolerance = 90.0;
	public static final int kGrabberPIDSlotIndex = 0, kGrabberRemoteSensorIndex = 0;

	public static final double kAngleMotorGearRatio = 125.0;
	// public static final double kInsideAngleLimit = 5.0, kOutsideAngleLimit = 140.0;
	public static final double kInsideAngleLimit = -140.0, kOutsideAngleLimit = 140.0;
	public static final double kAngleMotorSpeed = 0.175;

	public static final double kTelescopicStartPosition = 0.0;
	// public static final double kTelescopicRetractLimit = 0.0, kTelescopicExtendLimit = 130.0;
	public static final double kTelescopicRetractLimit = -130.0, kTelescopicExtendLimit = 130.0;
	public static final double kTelescopicMotorSpeed = 0.25;

	public static final int kPOVUp = 0, kPOVDown = 180;
	public static final int kPOVRight = 90, kPOVLeft = 270;
}