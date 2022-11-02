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
	private final JoystickButton crossButton;

	private final DrivetrainSubsystem drivetrain;

	private final FollowGeneratedTrajectoryCommand followGeneratedTrajectoryCommand;
	private final FollowJSONTrajectoryCommand followJSONTrajectoryCommand;

	private final ShuffleboardTab odometryTab;
	private final NetworkTableEntry selectedAutoCommand;

	public RobotContainer() {
		this.controller = new PS4Controller(0);
		this.drivetrain = DrivetrainSubsystem.getInstance();
		this.shareButton = new JoystickButton(this.controller, PS4Controller.Button.kShare.value);
		this.optionsButton = new JoystickButton(this.controller, PS4Controller.Button.kOptions.value);
		this.crossButton = new JoystickButton(this.controller, PS4Controller.Button.kCross.value);

		this.followGeneratedTrajectoryCommand = new FollowGeneratedTrajectoryCommand(this.drivetrain);
		this.followJSONTrajectoryCommand = new FollowJSONTrajectoryCommand(this.drivetrain);

		this.odometryTab = Shuffleboard.getTab("Odometry");
		this.selectedAutoCommand = this.odometryTab.add(
				"Auto Command", "").withWidget(BuiltInWidgets.kTextView).getEntry();

		this.setDefaultCommands();
		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Share button zeros the gyroscope (no requirments)
		this.shareButton.whenPressed(this.drivetrain::resetYaw);
		// Options button resets the odometry (no requirments)
		this.optionsButton.whenPressed(this.drivetrain::resetOdometry);
		// Cross button puts the wheels in cross lock shape until moved again
		// (requires DrivetrainSubsystem)
		this.crossButton.whenPressed(new InstantCommand(this.drivetrain::crossLockWheels, this.drivetrain));
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

	private double modifyAxis(double value, double ratio) {
		value = deadBand(value, 0.2); // Deadband
		return value * ratio;
	}

	private void setDefaultCommands() {
		// Set up the default command for the drivetrain.
		// The controls are for field-oriented driving:
		// Left stick Y axis -> forward and backwards movement
		// Left stick X axis -> left and right movement
		// Right stick X axis -> rotation
		this.drivetrain.setDefaultCommand(new TeleopDriveCommand(this.drivetrain,
				() -> -modifyAxis(controller.getLeftY(), 0.3) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getLeftX(), 0.3) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getRightX(), 0.3)
						* DrivetrainConstants.kMaxAngularVelocity_RadiansPerSecond));
	}

	protected enum AutoCommand {
		kFollowPathplannerTrajectory,
		kFollowCodeGeneratedTrajectory;
	}

	/**
	 * Returns the command to run in autonomous mode,
	 * and writes to the ShuffleBoard which one it is.
	 * <p>
	 * 
	 * @param autoCommand - an enum of the type RobotContainer.AutoCommand
	 *                    <p>
	 * @return an object of the type Command -
	 *         Either FollowJSONTrajectoryCommand, or
	 *         FollowGeneratedTrajectoryCommand.
	 */
	protected Command getAutoCommand(AutoCommand autoCommand) {
		if (autoCommand == AutoCommand.kFollowPathplannerTrajectory) {
			this.selectedAutoCommand.setString("Follow trajectory from JSON");
			return this.followJSONTrajectoryCommand;
		} else {
			this.selectedAutoCommand.setString("follow trajectory generated in code");
			return this.followGeneratedTrajectoryCommand;
		}
	}

	public void crossLockWheels() {
		this.drivetrain.crossLockWheels();
	}

	/**
	 * Periodic routines that aren't commands, are not specific to
	 * any subsystem, and should always run no matter the robot mode.
	 */
	protected void runGeneralPeriodicRoutines() {
	}
}