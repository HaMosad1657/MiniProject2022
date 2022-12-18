// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.AutoCommand;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
	private Command autoCommand;
	private CommandScheduler commandScheduler;
	private PowerDistribution pdh;

	@Override
	public void robotInit() {
		this.pdh = new PowerDistribution();
		this.pdh.clearStickyFaults();
		this.robotContainer = new RobotContainer();
		this.commandScheduler = CommandScheduler.getInstance();
		this.pdh.close();
	}

	@Override
	public void robotPeriodic() {
		this.commandScheduler.run();
		this.robotContainer.runGeneralPeriodicRoutines();
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void disabledInit() {
		RobotContainer.drivetrain.crossLockWheels();
	}
}
