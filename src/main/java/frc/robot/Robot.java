// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
	private Command autoCommand;

	@Override
	public void robotInit() {
		this.robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		this.autoCommand = robotContainer.getAutoCommand();
		CommandScheduler.getInstance().schedule(this.autoCommand);
	}

	@Override
	public void autonomousPeriodic() {
	}
}
