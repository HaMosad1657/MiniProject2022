package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.FollowGeneratedTrajectoryCommand;
import frc.robot.commands.drive.FollowJSONTrajectoryCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.chassis.DrivetrainConstants;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;

public class RobotContainer {
	private final PS4Controller controller;
	private final JoystickButton shareButton;
	private final JoystickButton optionsButton;

	private final DrivetrainSubsystem drivetrain;

	private final FollowGeneratedTrajectoryCommand followGeneratedTrajectoryCommand;
	private final FollowJSONTrajectoryCommand followJSONTrajectoryCommand;

	private final ShuffleboardTab joysticksTab;
	private final NetworkTableEntry leftJoystickX, leftJoystickY, rightJoystickX;

	public RobotContainer() {
		this.controller = new PS4Controller(0);
		this.drivetrain = DrivetrainSubsystem.getInstance();
		this.shareButton = new JoystickButton(this.controller, PS4Controller.Button.kShare.value);
		this.optionsButton = new JoystickButton(this.controller, PS4Controller.Button.kOptions.value);

		this.followGeneratedTrajectoryCommand = new
				FollowGeneratedTrajectoryCommand(this.drivetrain);
		this.followJSONTrajectoryCommand = new
				FollowJSONTrajectoryCommand(this.drivetrain);
				
		this.joysticksTab = Shuffleboard.getTab("Joysticks");
		this.leftJoystickX = this.joysticksTab.add("Left joystick X", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.leftJoystickY = this.joysticksTab.add("Left joystick Y", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
		this.rightJoystickX = this.joysticksTab.add("Right joystick X", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

		this.setDefaultCommands();
		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Triangle button zeros the gyroscope
		this.shareButton
				// No requirements because we don't need to interrupt anything
				.whenPressed(this.drivetrain::resetYaw);

		this.optionsButton.whenPressed(new InstantCommand(this.drivetrain::crossLockChassis, this.drivetrain));
	}

	private static double deadBand(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private double modifyAxis(double value) {
		value = deadBand(value, 0.2); // Deadband
		// if (value > 0.0)
		// value += RobotConstants.kMotorSpeedOffset; // Offset
		// value = Math.copySign(value * value, value); // Square the axis
		return value * 0.3; // Ratio
	}

	private void setDefaultCommands() {
		// Set up the default command for the drivetrain.
		// The controls are for field-oriented driving:
		// Left stick Y axis -> forward and backwards movement
		// Left stick X axis -> left and right movement
		// Right stick X axis -> rotation
		this.drivetrain.setDefaultCommand(new TeleopDriveCommand(this.drivetrain,
				() -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.kMaxAngularVelocity_RadiansPerSecond));
	}

	protected enum AutoCommand {
		kFollowPathplannerTrajectory,
		kFollowCodeGeneratedTrajectory;
	}

	protected Command getAutoCommand(AutoCommand autoCommand) {
		if (autoCommand == AutoCommand.kFollowPathplannerTrajectory) {
			return this.followJSONTrajectoryCommand;
		} else
			return this.followGeneratedTrajectoryCommand;
	}
	
	/**
	 * Periodic routines that aren't commands, are not specific to
	 * any subsystem, and should always run no matter the robot mode.
	 */
	protected void runGeneralPeriodicRoutines() {
		this.leftJoystickX.setDouble(deadBand(this.controller.getLeftX(), 0.2));
		this.leftJoystickY.setDouble(deadBand(this.controller.getLeftY(), 0.2));
		this.rightJoystickX.setDouble(deadBand(this.controller.getRightX(), 0.2));
	}
}