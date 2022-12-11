package frc.robot.commands.disks;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.disks.DisksConstants;
import frc.robot.subsystems.disks.DisksSubsystem;

public class TeleopDisksCommand extends CommandBase {

	/**
	 * POV up-down extends and retracts arm
	 * POV left-right raises and lowers the arm
	 * left trigger toggles grabber
	 */

	private DisksSubsystem disksSubsystem;

	public TeleopDisksCommand() {
		this.disksSubsystem = DisksSubsystem.GetInstance();
		this.addRequirements(this.disksSubsystem);
	}

	@Override
	public void initialize() {
		this.disksSubsystem.resetEncoders();
	}

	@Override
	public void execute() {
		if (RobotContainer.controller.getL1Button()) {
			this.disksSubsystem.toggleGrabber(); // toggles Grabber
		}
		if (RobotContainer.controller.getPOV() == 0) {
			if (this.disksSubsystem.getTelescopicPosition() < DisksConstants.kTelescopicExtendLimit) {
				this.disksSubsystem.setTelescopicMotor(DisksConstants.kTelescopicMotorSpeed); // telescopic arm extends
			}
		}
		if (RobotContainer.controller.getPOV() == 90) {
			if (this.disksSubsystem.getTelescopicPosition() < DisksConstants.kAngleMotorInsideLimit) {
				this.disksSubsystem.setAngleMotor(DisksConstants.kAngleMotorSpeed); // arm rotates inside
			}
		}
		if (RobotContainer.controller.getPOV() == 180) {
			if (this.disksSubsystem.getTelescopicPosition() < DisksConstants.kTelescopicRetractLimit) {
				this.disksSubsystem.setTelescopicMotor(-(DisksConstants.kTelescopicMotorSpeed)); // telescopic arm retracts

			}
		}
		if (RobotContainer.controller.getPOV() == 270) {
			this.disksSubsystem.setAngleMotor(-(DisksConstants.kAngleMotorSpeed)); // arm rotates outside
			if (this.disksSubsystem.getTelescopicPosition() < DisksConstants.kAngleMotorOutsideLimit) {
				this.disksSubsystem.setAngleMotor(DisksConstants.kAngleMotorSpeed);
			}
		} else {
			this.disksSubsystem.setAngleMotor(0);
			this.disksSubsystem.setTelescopicMotor(0);
		}
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
