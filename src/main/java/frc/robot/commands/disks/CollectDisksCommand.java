package frc.robot.commands.disks;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.disks.DisksSubsystem;

public class CollectDisksCommand extends CommandBase {

	/**
	 * start with the grabber opened
	 * move the arm to a certain angle and extend the telescopic
	 * according to our distance from the disks stack
	 * close the grabber
	 * lift arm a little bit and maybe retract (check how much is possible)
	 */

	private DisksSubsystem disksSubsystem;

    public CollectDisksCommand() {
		this.disksSubsystem = DisksSubsystem.GetInstance();
    }

    @Override
	public void initialize() {
		if (!this.disksSubsystem.isGrabberOpened()) {
			this.disksSubsystem.toggleGrabber();
		}
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
