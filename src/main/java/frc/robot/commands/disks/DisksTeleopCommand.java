package frc.robot.commands.disks;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.disks.DisksConstants;
import frc.robot.subsystems.disks.DisksSubsystem;

/**
 * POV up-down extends and retracts arm.
 * <p>
 * POV left-right rotates the arm inwards and outwards.
 * <p>
 * Left trigger toggles grabber.
 */
public class DisksTeleopCommand extends CommandBase {
	private DisksSubsystem disks;

	public DisksTeleopCommand(DisksSubsystem disks) {
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
		if (RobotContainer.controller.getL2ButtonPressed()) {
			this.disks.toggleGrabber();
		}

		// Telescopic extend
		if (RobotContainer.controller.getPOV() == DisksConstants.kPOVUp
				&& this.disks.getExtendPosition() < DisksConstants.kTelescopicExtendLimit) {
			this.disks.setExtendMotor(-DisksConstants.kTelescopicMotorSpeed);
		}

		// Telescopic retract
		else if (RobotContainer.controller.getPOV() == DisksConstants.kPOVDown
				&& this.disks.getExtendPosition() > DisksConstants.kTelescopicRetractLimit) {
			this.disks.setExtendMotor(DisksConstants.kTelescopicMotorSpeed);
		}

		// Stop telescopic
		else {
			this.disks.setExtendMotor(0.0);
		}

		// Telescopic rotate outside
		if (RobotContainer.controller.getPOV() == DisksConstants.kPOVRight
				&& this.disks.getTelescopicAngle() < DisksConstants.kOutsideAngleLimit) {
			this.disks.setAngleMotor(-DisksConstants.kAngleMotorSpeed);
		}

		// Telescopic rotate inside
		else if (RobotContainer.controller.getPOV() == DisksConstants.kPOVLeft
				&& this.disks.getTelescopicAngle() > DisksConstants.kInsideAngleLimit) {
			this.disks.setAngleMotor(DisksConstants.kAngleMotorSpeed);
		}

		// Stop angle
		else {
			this.disks.setAngleMotor(0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.disks.setExtendMotor(0.0);
		this.disks.setAngleMotor(0.0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
