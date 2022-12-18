// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.AutoCommand;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
	private Command autoCommand;
	private CommandScheduler commandScheduler;

	// The constructors of all the subsystems and commands are called in
	// RobotContainer(),
	// and RobotContainer() is called here. Note that the "robot code" mark in the
	// Driver
	// Station won't show green until robotInit() is done.
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
