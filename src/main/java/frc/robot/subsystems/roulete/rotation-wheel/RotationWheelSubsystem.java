
package frc.robot.subsystems.rotationWheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armMover.ArmMoverConstants;

public class RotationWheelSubsystem extends SubsystemBase {
  private static RotationWheelSubsystem instance;

  public static RotationWheelSubsystem getInstance() {
    if (instance == null) {
      instance = new RotationWheelSubsystem();
    }
    return instance;
  }

  private TalonSRX motor;

  private RotationWheelSubsystem() {
    this.motor = new TalonSRX(ArmMoverConstants.kMotorID);
  }

  public void setMotor(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }
}
