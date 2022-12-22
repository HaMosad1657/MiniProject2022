
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.RobotContainer.AutoCommand;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
	private Command autoCommand;
	private CommandScheduler commandScheduler;

	// Note that the "Robot Code" mark in the Driver Station won't show green until robotInit() is done.
	@Override
	public void robotInit() {
		this.robotContainer = new RobotContainer();
		this.commandScheduler = CommandScheduler.getInstance();
	}

	@Override
	public void robotPeriodic() {
		this.commandScheduler.run();
	}

	@Override
	public void autonomousInit() {
		// this.autoCommand = this.robotContainer.getAutoCommand(AutoCommand.kFollowJSONTrajectory);
		// this.commandScheduler.schedule(this.autoCommand); // schedule() only needs to be called once
	}

	@Override
	public void disabledInit() {
		this.robotContainer.crossLockWheels();
		this.commandScheduler.cancelAll();
	}

	public static void print(Object... objects) {
		for (var object : objects)
			DriverStation.reportWarning(object.toString(), false);
	}

	public static void printError(Object... objects) {
		for (var object : objects)
			DriverStation.reportError(object.toString(), true);
	}
}
