package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.disks.DisksTeleopCommand;
import frc.robot.commands.drive.FollowGeneratedTrajectoryCommand;
import frc.robot.commands.drive.FollowJSONTrajectoryCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.chassis.DrivetrainConstants;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import frc.robot.subsystems.disks.DisksSubsystem;

public class RobotContainer {
	public static final PS4Controller controller = new PS4Controller(0);;
	private final JoystickButton shareButton;

	private final DrivetrainSubsystem drivetrain;
	private final DisksSubsystem disks;

	private final FollowGeneratedTrajectoryCommand followGeneratedTrajectoryCommand;
	private final FollowJSONTrajectoryCommand followJSONTrajectoryCommand;

	public RobotContainer() {
		this.shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

		this.drivetrain = DrivetrainSubsystem.getInstance();
		this.disks = DisksSubsystem.getInstance();

		this.followGeneratedTrajectoryCommand = new FollowGeneratedTrajectoryCommand(this.drivetrain);
		this.followJSONTrajectoryCommand = new FollowJSONTrajectoryCommand(this.drivetrain);

		this.configureButtonBindings();
		this.setDefaultCommands();
	}

	private void configureButtonBindings() {
		// Triangle button zeros the gyroscope
		this.shareButton
				// No requirements because we don't need to interrupt anything
				.whenPressed(this.drivetrain::resetYaw);
	}

	private static double deadband(double value, double deadband) {
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
		value = deadband(value, 0.2); // Deadband
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
				() -> -modifyAxis(-controller.getLeftY()) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(-controller.getLeftX()) * DrivetrainConstants.kMaxChassisVelocityMPS,
				() -> -modifyAxis(-controller.getRightX()) * DrivetrainConstants.kMaxAngularVelocity_RadiansPerSecond));

		this.disks.setDefaultCommand(new DisksTeleopCommand(this.disks));
	}

	protected enum AutoCommand {
		kFollowPathplannerTrajectory, kFollowCodeGeneratedTrajectory;
	}

	public Command getAutoCommand(AutoCommand autoCommand) {
		if (autoCommand == AutoCommand.kFollowPathplannerTrajectory) {
			return this.followJSONTrajectoryCommand;
		} else
			return this.followGeneratedTrajectoryCommand;
	}
}