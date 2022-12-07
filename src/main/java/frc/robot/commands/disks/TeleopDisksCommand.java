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
    }

    @Override
	public void execute() {
		if (RobotContainer.controller.getPOV() == 0) {
			this.disksSubsystem.setTelescopicMotor(DisksConstants.kTelescopicMotorSpeed);
		} else
			this.disksSubsystem.setTelescopicMotor(0);
	}
	
	

    @Override
    public void end(boolean interrupted) {

    }

    @Override
	public boolean isFinished() {
        return false;
    }
}
