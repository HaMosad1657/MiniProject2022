package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
	private final PS4Controller controller;
	private final JoystickButton shareButton;
	private final JoystickButton optionsButton;
	private final JoystickButton crossButton;

	private final ShuffleboardTab odometryTab;
	private final NetworkTableEntry selectedAutoCommand;

	public RobotContainer() {
		this.controller = new PS4Controller(0);
		this.shareButton = new JoystickButton(this.controller, PS4Controller.Button.kShare.value);
		this.optionsButton = new JoystickButton(this.controller, PS4Controller.Button.kOptions.value);
		this.crossButton = new JoystickButton(this.controller, PS4Controller.Button.kCross.value);

		this.odometryTab = Shuffleboard.getTab("Odometry");
		this.selectedAutoCommand = this.odometryTab.add(
				"Auto Command", "").withWidget(BuiltInWidgets.kTextView).getEntry();

		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Share button zeros the gyroscope (no requirments)
		// Options button resets the odometry (no requirments)
		// Cross button puts the wheels in cross lock shape until moved again
		// (requires DrivetrainSubsystem)
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

	protected enum AutoCommand {
		kFollowJSONTrajectory,
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

	/**
	 * Periodic routines that aren't commands, are not specific to
	 * any subsystem, and should always run no matter the robot mode.
	 */
	protected void runGeneralPeriodicRoutines() {
	}
}