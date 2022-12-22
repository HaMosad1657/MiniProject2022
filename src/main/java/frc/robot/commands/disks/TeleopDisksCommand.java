package frc.robot.commands.disks;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.disks.DisksConstants;
import frc.robot.subsystems.disks.DisksSubsystem;

/**
 * POV up-down extends and retracts arm.
 * <p>
 * POV left-right raises and lowers the arm.
 * <p>
 * Left trigger toggles grabber.
 */
public class TeleopDisksCommand extends CommandBase {
	private DisksSubsystem disks;

	public TeleopDisksCommand(DisksSubsystem disks) {
		this.disks = disks;
		this.addRequirements(this.disks);
	}

	@Override
	public void initialize() {
		this.disks.resetEncoders();
	}

	@Override
	public void execute() {
		// Toggle grabber
		if (RobotContainer.controller.getL1Button()) {
			this.disks.toggleGrabber();
		}

		// Telescopic extend
		if (RobotContainer.controller.getPOV() == DisksConstants.kPOVUp
				&& this.disks.getTelescopicPosition() < DisksConstants.kTelescopicExtendLimit) {
			this.disks.setTelescopicMotor(DisksConstants.kTelescopicMotorSpeed);
		}

		// Telescopic retract
		else if (RobotContainer.controller.getPOV() == DisksConstants.kPOVDown
				&& this.disks.getTelescopicPosition() > DisksConstants.kTelescopicRetractLimit) {
			this.disks.setTelescopicMotor(-DisksConstants.kTelescopicMotorSpeed);
		}

		// Stop telescopic
		else {
			this.disks.setTelescopicMotor(0.0);
		}

		// Telescopic rotate inside
		if (RobotContainer.controller.getPOV() == DisksConstants.kPOVRight
				&& this.disks.getAngle() > DisksConstants.kInsideAngleLimit) {
			this.disks.setAngleMotor(-DisksConstants.kAngleMotorSpeed);
		}

		// Telescopic rotate outside
		else if (RobotContainer.controller.getPOV() == DisksConstants.kPOVLeft
				&& this.disks.getAngle() < DisksConstants.kOutsideAngleLimit) {
			this.disks.setAngleMotor(DisksConstants.kAngleMotorSpeed);
		}

		// Stop angle
		else {
			this.disks.setAngleMotor(0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.disks.setTelescopicMotor(0.0);
		this.disks.setAngleMotor(0.0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
