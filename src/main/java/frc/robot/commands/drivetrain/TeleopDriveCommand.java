package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TeleopDriveCommand extends CommandBase {
	private final DrivetrainSubsystem drivetrain;

	// DoubleSupplier is an object, function or a lambda that returns a double.
	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;
	private final DoubleSupplier rotationSupplier;

	public TeleopDriveCommand(
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		this.drivetrain = RobotContainer.drivetrain;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		this.addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {
	}

	// The method fromFieldRelativeSpeeds() retruns a chassisSpeeds
	// object from the joystick inputs and the gyro measurment.
	// true/false for not returning the modules to angle 0 when chassisSpeeds is 0.
	@Override
	public void execute() {
		this.drivetrain.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						this.translationXSupplier.getAsDouble(),
						this.translationYSupplier.getAsDouble(),
						this.rotationSupplier.getAsDouble(),
						this.drivetrain.getGyroRotation()),
				true);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true);
	}
}