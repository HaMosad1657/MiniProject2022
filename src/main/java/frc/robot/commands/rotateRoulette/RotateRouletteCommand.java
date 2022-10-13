
package frc.robot.commands.rotateRoulette;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.rotationWheel.RotationWheelSubsystem;

public class RotateRouletteCommand extends CommandBase {
  private RotationWheelSubsystem rotationWheel;
  private ColorSensorSubsystem colorSensor;

  public RotateRouletteCommand(RotationWheelSubsystem rotationWheel, ColorSensorSubsystem colorSensor) {
    this.rotationWheel = rotationWheel;
    this.colorSensor = colorSensor;
    this.addRequirements(this.rotationWheel, this.colorSensor);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
