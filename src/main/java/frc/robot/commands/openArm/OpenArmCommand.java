
package frc.robot.commands.openArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armMover.ArmMoverSubsystem;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;

public class OpenArmCommand extends CommandBase {
  private ArmMoverSubsystem armMover;
  private ColorSensorSubsystem colorSensor;
  private boolean finished;

  public OpenArmCommand(ArmMoverSubsystem armMover, ColorSensorSubsystem colorSensor) {
    this.finished = false;
    this.armMover = armMover;
    this.colorSensor = colorSensor;
    this.addRequirements(this.armMover, this.colorSensor);
  }

  @Override
  public void initialize() {
    armMover.setMotor(OpenArmConstants.kDeafultSpeed);
    // put waiting function here.
  }

  @Override
  public void execute() {
    if (colorSensor.getProximity() == OpenArmConstants.kWantedProximity) {
      this.finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    armMover.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
