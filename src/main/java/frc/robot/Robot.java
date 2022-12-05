// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
	private Command autoCommand;
	private CommandScheduler commandScheduler;
	private Photon photon = new Photon();

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
		double x = photon.robotCurrentX();
		double y = photon.robotCurrentY();
		double z = photon.robotCurrentZ();
		SmartDashboard.putNumber("X", x);
		SmartDashboard.putNumber("Y", y);
		SmartDashboard.putNumber("Z", y);
		this.commandScheduler.run();
		this.robotContainer.runGeneralPeriodicRoutines();
	}

	@Override
	public void autonomousInit() {
		this.commandScheduler.schedule(this.autoCommand); // schedule() only needs to be called once
	}

	@Override
	public void disabledInit() {
		// this.robotContainer.crossLockWheels();
		this.commandScheduler.cancelAll();
	}
}
