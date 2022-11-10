package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
	private final DrivetrainSubsystem drivetrainSubsystem;

	// DoubleSupplier is an object, function or a lambda that returns a double.
	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;
	private final DoubleSupplier rotationSupplier;

	public TeleopDriveCommand(
			DrivetrainSubsystem drivetrainSubsystem,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		this.drivetrainSubsystem = drivetrainSubsystem;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		this.addRequirements(this.drivetrainSubsystem);
	}

	// The method fromFieldRelativeSpeeds() retruns a chassisSpeeds
	// object from the joystick inputs and the gyro measurment.
	// true/false for not returning the modules to angle 0 when chassisSpeeds is 0.
	@Override
	public void execute() {
		//this.drivetrainSubsystem.drive(
		this.drivetrainSubsystem.altDriveDebugging(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						this.translationXSupplier.getAsDouble(),
						this.translationYSupplier.getAsDouble(),
						this.rotationSupplier.getAsDouble(),
						this.drivetrainSubsystem.getGyroRotation()));
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}