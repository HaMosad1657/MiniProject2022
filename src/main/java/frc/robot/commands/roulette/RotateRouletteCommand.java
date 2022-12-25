
package frc.robot.commands.roulette;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.roulette.RouletteConstants;
import frc.robot.subsystems.roulette.RouletteSubsystem;

public class RotateRouletteCommand extends CommandBase {
  private final RouletteSubsystem rouletteSubsystem;
  private double requiredSemiRotations;

  private int semiRotationsCount;
  private Alliance previousColor;

  public RotateRouletteCommand(RouletteSubsystem rouletteSubsystem) {
    this.semiRotationsCount = 0;

    this.rouletteSubsystem = rouletteSubsystem;
    this.addRequirements(this.rouletteSubsystem);
  }

  @Override
  public void initialize() {
    Alliance rouletteColor = this.rouletteSubsystem.getRouletteColor();
    this.semiRotationsCount = RouletteSubsystem.getRequiredRotations(DriverStation.getAlliance(), rouletteColor);
    this.previousColor = RouletteSubsystem.getOppositeAlliance(rouletteColor);

    this.rouletteSubsystem.setRotationMotor(RouletteConstants.kDeafultSpeed);
  }

  @Override
  public void execute() {
    if (this.previousColor != this.rouletteSubsystem.getRouletteColor()) {
      this.semiRotationsCount++;
      this.previousColor = RouletteSubsystem.getOppositeAlliance(this.previousColor);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.rouletteSubsystem.setRotationMotor(0.0);
  }

  @Override
  public boolean isFinished() {
    return this.semiRotationsCount == this.requiredSemiRotations;
  }
}
