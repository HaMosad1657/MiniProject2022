
package frc.robot.commands.roulette;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.roulette.RouletteSubsystem;

public class RotateRouletteCommand extends CommandBase {
  private RouletteSubsystem rouletteSubsystem;
  private boolean finished;
  private int fullRotationCount;
  private Alliance team; 
  private double wantedRotationCount;
  private Alliance prevColor; 

  public RotateRouletteCommand(RouletteSubsystem rouletteSubsystem) {
    this.rouletteSubsystem = rouletteSubsystem;
    this.finished = false;
    this.fullRotationCount = 0;
    this.team = Alliance.Blue;
    this.addRequirements(this.rouletteSubsystem);
  }

  private int getWantedRotationCountBlue() {
    if (rouletteSubsystem.isBlue()) {
      this.prevColor = Alliance.Blue;
      return RotateRouletteConstants.kMinSemiRotation;
    }
    this.prevColor = Alliance.Red;
    return RotateRouletteConstants.kMaxSemiRotation;
  }

  private int getWantedRotationCountRed() {
    if (rouletteSubsystem.isBlue()) {
      this.prevColor = Alliance.Red;
      return RotateRouletteConstants.kMaxSemiRotation;
    }
    this.prevColor = Alliance.Blue;
    return RotateRouletteConstants.kMinSemiRotation;
  }

  @Override
  public void initialize() {
    this.rouletteSubsystem.setRotationMotor(RotateRouletteConstants.kDeafultSpeed);
    if (this.team == Alliance.Blue) { 
      this.wantedRotationCount = getWantedRotationCountBlue();
    } else { 
      this.wantedRotationCount = getWantedRotationCountRed();
    }
  }

  @Override
  public void execute() {
    if (prevColor != rouletteSubsystem.getFullColor()) {
      this.fullRotationCount++;
      // maybe wait here abit
      prevColor = rouletteSubsystem.getFullColor();
    }
    if (this.fullRotationCount == this.wantedRotationCount) {
      // maybe wait here abit
      this.finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.rouletteSubsystem.setRotationMotor(0.0);
  }

  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
