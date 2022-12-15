
package frc.robot.commands.rotateRoulette;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.rotationWheel.RotationWheelSubsystem;

public class RotateRouletteCommand extends CommandBase {
  private RotationWheelSubsystem rotationWheel;
  private ColorSensorSubsystem colorSensor;
  private boolean finished;
  private int fullRotationCount;
  private boolean team; // placeholder for the acctual team checker object
  private double wantedRotationCount;
  private boolean whatPrevColor; // true = blue, false = red

  public RotateRouletteCommand(RotationWheelSubsystem rotationWheel, ColorSensorSubsystem colorSensor) {
    this.rotationWheel = rotationWheel;
    this.colorSensor = colorSensor;
    this.finished = false;
    this.fullRotationCount = 0;
    this.team = RotateRouletteConstants.kTeam;
    this.addRequirements(this.rotationWheel, this.colorSensor);
  }


  private int getWantedRotationCountBlue() {
    if (colorSensor.checkForBlue()) {
      this.whatPrevColor = true;
      return RotateRouletteConstants.kMinSemiRotation;
    }
    this.whatPrevColor = false;
    return RotateRouletteConstants.kMaxSemiRotation;
  }

  private int getWantedRotationCountRed() {
    if (colorSensor.checkForBlue()) {
      this.whatPrevColor = true;
      return RotateRouletteConstants.kMaxSemiRotation;
    }
    this.whatPrevColor = false;
    return RotateRouletteConstants.kMinSemiRotation;
  }

  @Override
  public void initialize() {
    this.rotationWheel.setMotor(RotateRouletteConstants.kDeafultSpeed);
    if (this.team) { // blue
      this.wantedRotationCount = getWantedRotationCountBlue();
    } else { // red
      this.wantedRotationCount = getWantedRotationCountRed();
    }
  }

  @Override
  public void execute() {
    if (whatPrevColor != colorSensor.checkForBlue()) {
      this.fullRotationCount++;
      // maybe wait here abit
      whatPrevColor = colorSensor.checkForBlue();
    }
    if (this.fullRotationCount == this.wantedRotationCount) {
      // maybe wait here abit
      this.finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.rotationWheel.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
