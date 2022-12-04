
package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climbing.ClimbingConstants;
import frc.robot.subsystems.climbing.ClimbingSubsystem;

public class ClimbingCommand extends CommandBase {
	ClimbingSubsystem climbing;

	public ClimbingCommand(ClimbingSubsystem climbing) {
		this.climbing = climbing;
		this.addRequirements(this.climbing);
	}

	@Override
	public void initialize() {
		this.climbing.resetEncoderLeft();
		this.climbing.resetEncoderRight();
	}

	@Override
	public void execute() {
		if (RobotContainer.controller.getL2Button()
				&& this.climbing.getScissorsEncoderLeft() < ClimbingConstants.kScissorsExtendLimit) {
			this.climbing.setScissorsLeft(ClimbingConstants.kScissorsSpeed);
		} else if (RobotContainer.controller.getL1Button()
				&& this.climbing.getScissorsEncoderLeft() > ClimbingConstants.kScissorsRetractLimit) {
			this.climbing.setScissorsLeft(-ClimbingConstants.kScissorsSpeed);
		} else {
			this.climbing.setScissorsLeft(0);
		}

		if (RobotContainer.controller.getR2Button()
				&& this.climbing.getScissorsEncoderRight() < ClimbingConstants.kScissorsExtendLimit) {
			this.climbing.setScissorsRight(ClimbingConstants.kScissorsSpeed);
		} else if (RobotContainer.controller.getR1Button()
				&& this.climbing.getScissorsEncoderRight() > ClimbingConstants.kScissorsRetractLimit) {
			this.climbing.setScissorsRight(-ClimbingConstants.kScissorsSpeed);
		} else {
			this.climbing.setScissorsRight(0);
		}

		this.climbing.setPulleyLeft(RobotContainer.controller.getLeftY() * ClimbingConstants.kPulleySpeedLimit);
		this.climbing.setPulleyRight(RobotContainer.controller.getRightY() * ClimbingConstants.kPulleySpeedLimit);

	}

	@Override
	public void end(boolean interrupted) {
		this.climbing.setScissorsLeft(0);
		this.climbing.setScissorsRight(0);
		this.climbing.setPulleyLeft(0);
		this.climbing.setPulleyRight(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
