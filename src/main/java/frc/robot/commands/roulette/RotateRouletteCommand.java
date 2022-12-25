
package frc.robot.commands.roulette;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.roulette.RouletteConstants;
import frc.robot.subsystems.roulette.RouletteSubsystem;

public class RotateRouletteCommand extends CommandBase {
	private final RouletteSubsystem rouletteSubsystem;
	private double requiredSemiRotations;

	private int semiRotationsCount;
	private Alliance previousColor;
	private Timer timer;

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
		this.timer.start();
	}

	@Override
	public void execute() {
		if (this.previousColor != this.rouletteSubsystem.getRouletteColor()
				&& timer.get() < RouletteConstants.kWaitingTime) {
			this.semiRotationsCount++;
			this.rouletteSubsystem.upRotationCount();
			this.previousColor = RouletteSubsystem.getOppositeAlliance(this.previousColor);
			this.timer.reset();
			this.timer.start();
		}

	}

	@Override
	public void end(boolean interrupted) {
		this.rouletteSubsystem.setRotationMotor(0.0);
		this.rouletteSubsystem.resetRotCount();
		this.rouletteSubsystem.finish();
		this.timer.stop();
	}

	@Override
	public boolean isFinished() {
		return this.semiRotationsCount == this.requiredSemiRotations;
	}
}
