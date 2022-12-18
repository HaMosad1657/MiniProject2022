package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {
	public static final PS4Controller controller = new PS4Controller(0);;
	private final JoystickButton shareButton;
	private final JoystickButton optionsButton;
	private final JoystickButton triangleButton;

	public static final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

	private final ShuffleboardTab odometryTab;
	private final NetworkTableEntry selectedAutoCommand;

	public RobotContainer() {
		this.shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
		this.optionsButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
		this.triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);

		this.odometryTab = Shuffleboard.getTab("Odometry");
		this.selectedAutoCommand = this.odometryTab.add(
				"Auto Command", "").withWidget(BuiltInWidgets.kTextView).getEntry();

		this.setDefaultCommands();
		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Triangle button zeros the gyroscope
		this.shareButton
				// No requirements because we don't need to interrupt anything
				.whenPressed(drivetrain::resetYaw);

		this.optionsButton.whenPressed(new InstantCommand(drivetrain::crossLockWheels, drivetrain));
		this.triangleButton.whenPressed(new InstantCommand(drivetrain::resetOdometry, drivetrain));
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
		drivetrain.setDefaultCommand(new TeleopDriveCommand(
				() -> -modifyAxis(controller.getLeftY(), 1) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getLeftX(), 1) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(controller.getRightX(), 0.85)
						* DrivetrainConstants.kMaxAngularVelocity_RadiansPerSecond));
	}

	protected enum AutoCommand {
		kFollowPathplannerTrajectory,
		kFollowCodeGeneratedTrajectory;
	}

	/**
	 * Periodic routines that aren't commands, are not specific to
	 * any subsystem, and should always run no matter the robot mode.
	 */
	protected void runGeneralPeriodicRoutines() {
	}
}