
package frc.robot.subsystems.armMover;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMoverSubsystem extends SubsystemBase {
  private static ArmMoverSubsystem instance;

  public static ArmMoverSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmMoverSubsystem();
    }
    return instance;
  }

  private TalonSRX motor;

  private ArmMoverSubsystem() {
    this.motor = new TalonSRX(ArmMoverConstants.kMotorID);
  }

  public void setMotor(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }
}
