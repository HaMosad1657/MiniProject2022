package frc.robot.subsystems.disks;

public class DisksConstants {
    public static final int kAngleMotorID = 0; // TODO: Надо чекнуть айди мотора а то пока что не ебу.
    public static final int kTelescopicMotorID = 1;
    public static final int kGrabberMotorID = 2;
    public static final int kGrabberEncoderID = 0;

    public static final double kGrabberOpenedPosition = 0.0, kGrabberClosedPosition = 0.0; // TODO: Энкодер позишн Надо физически покрутить мотор и посмотреть позицию в открытом состоянии
    public static final double kGrabberP = 0.0, kGrabberI = 0.0, kGrabberD = 0.0, kGrabberFF = 0.0;

    public static final double kAngleMotorGearRatio = 15.0; // это не рандомное число это я уже чекнул
    public static final double kAngleMotorInsideLimit = 0.0, kAngleMotorOutsideLimit = 0.0;
    public static final double kAngleMotorSpeed = 0.0;

    public static final double kTelescopicExtendLimit = 0.0, kTelescopicRetractLimit = 0.0;
    public static final double kTelescopicMotorSpeed = 0.0;

}