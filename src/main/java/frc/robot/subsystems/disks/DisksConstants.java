package frc.robot.subsystems.disks;

public class DisksConstants {
    public static final int kAngleMotorID = 20; 
    public static final int kTelescopicMotorID = 19;
    public static final int kGrabberMotorID = 13;
    public static final int kGrabberEncoderID = 14;

    public static final double kGrabberOpenedPosition = 0.0, kGrabberClosedPosition = 0.0; // TODO: Энкодер позишн Надо физически покрутить мотор и посмотреть позицию в открытом состоянии
    public static final double kGrabberP = 0.0, kGrabberI = 0.0, kGrabberD = 0.0, kGrabberFF = 0.0;
    public static final int kGrabberPIDSlotIndex = 0, kGrabberRemoteSensorIndex = 0;

    public static final double kAngleMotorStartPosition = 0.0;
    public static final double kAngleMotorGearRatio = 15.0; // это не рандомное число это я уже чекнул
    public static final double kInsideAngleLimit = 0.0, kOutsideAngleLimit = 0.0;
    public static final double kAngleMotorSpeed = 0.0;

    public static final double kTelescopicStartPosition = 0.0;
    public static final double kTelescopicExtendLimit = 0.0, kTelescopicRetractLimit = 0.0;
    public static final double kTelescopicMotorSpeed = 0.0;

    public static final double kCANCoderTicksPerRev = 4096;

    public static final int kPOVUp = 0, kPOVDown = 180;
    public static final int kPOVRight = 90, kPOVLeft = 270;

}