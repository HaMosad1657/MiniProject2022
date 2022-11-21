package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	private TalonFX driveMotor;
	private TalonFX steerMotor;
	private CANCoder encoder;

	public SwerveModule(int driveID, int steerID, int CANCoderID, double steerOffsetDegrees) {
		// Construct the CANCoder
		this.encoder = new CANCoder(CANCoderID);

		// Make the CANCoder boot to it's absloute value instead of to 0
		this.encoder.configSensorInitializationStrategy(
				SensorInitializationStrategy.BootToAbsolutePosition, DrivetrainConstants.kCANCoderTimeoutMs);

		// Set the CANCoder measurment coefficient to 0.087890625 so that it returns
		// degrees (this is the default)
		this.encoder.configFeedbackCoefficient(
				0.087890625, "deg", SensorTimeBase.PerSecond, DrivetrainConstants.kCANCoderTimeoutMs);

		// Set CANCoder offset
		this.encoder.configMagnetOffset(
				steerOffsetDegrees, DrivetrainConstants.kCANCoderTimeoutMs);

		// Make the CANCoder return measurments in 0 to 360.
		// NOTE: the position closed-loop treats the sensor measurments as continuous
		// (similar to the enableContinousInput() method from the WPIlib PIDController
		// class).
		this.encoder.configAbsoluteSensorRange(
				AbsoluteSensorRange.Unsigned_0_to_360, DrivetrainConstants.kCANCoderTimeoutMs);

		// Constrct the drive motor controller
		this.driveMotor = new TalonFX(driveID);

		// Set the drive motor to brake neutral mode
		this.driveMotor.setNeutralMode(NeutralMode.Brake);

		// Set the feedback device for the drive motor controller on slot 0
		// as it's integrated encoder.
		this.driveMotor.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor, 0, DrivetrainConstants.kTalonTimeoutMs);

		// Construct the steer motor controller
		this.steerMotor = new TalonFX(steerID);

		// Set the steer motor to brake neutral mode
		this.steerMotor.setNeutralMode(NeutralMode.Brake);

		// Set the feedback device for the steer motor controller on slot 0
		// as it's integrated encoder.
		this.steerMotor.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor, 0, DrivetrainConstants.kTalonTimeoutMs);

		// Sync the integrated encoder with the CANCoder
		this.steerMotor.setSelectedSensorPosition(
				(this.encoder.getAbsolutePosition() * DrivetrainConstants.kIntegratedEncoderTicksPerDegree)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction(),
				0, DrivetrainConstants.kTalonTimeoutMs);
	}

	/**
	 * Every iteration (about 1 ms), the Proportional gain is multiplied by the
	 * closed-loop error (the error is in raw sensor units when controlling
	 * position, or raw sensor units per 100 miliseconds when controlling velocity).
	 * Note that the MAX final output value is 1023 (or -1023 in the reverse
	 * directon), the integrated encoder has 2048 counts per revolution, and
	 * the CANCoder has 4096 counts per revolution.
	 * So if you want max output for an error of one full encoder revolution,
	 * set the proportional gain as 1023 / 2048 if using the integrated encoder
	 * for feedback, or 1023 / 4096 if using the CANCoder for feedback.
	 * <p>
	 * For more information:
	 * https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
	 */
	public void configDrivePID(double Proportional, double Integral, double Derivative, int slot) {
		this.driveMotor.config_kP(slot, Proportional, DrivetrainConstants.kTalonTimeoutMs);
		this.driveMotor.config_kI(slot, Integral, DrivetrainConstants.kTalonTimeoutMs);
		this.driveMotor.config_kD(slot, Derivative, DrivetrainConstants.kTalonTimeoutMs);
	}

	/**
	 * Every iteration (about 1 ms), the Proportional gain is multiplied by the
	 * closed-loop error (the error is in raw sensor units when controlling
	 * position, or raw sensor units per 100 miliseconds when controlling velocity).
	 * Note that the MAX final output value is 1023 (or -1023 in the reverse
	 * directon), the integrated encoder has 2048 counts per revolution, and
	 * the CANCoder has 4096 counts per revolution.
	 * So if you want max output for an error of one full encoder revolution,
	 * set the proportional gain as 1023 / 2048 if using the integrated encoder
	 * for feedback, or 1023 / 4096 if using the CANCoder for feedback.
	 * <p>
	 * For more information:
	 * https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
	 */
	public void configSteerPID(double Proportional, double Integral, double Derivative, int slot) {
		this.steerMotor.config_kP(slot, Proportional, DrivetrainConstants.kTalonTimeoutMs);
		this.steerMotor.config_kI(slot, Integral, DrivetrainConstants.kTalonTimeoutMs);
		this.steerMotor.config_kD(slot, Derivative, DrivetrainConstants.kTalonTimeoutMs);
	}

	/**
	 * @param MPS
	 */
	public void setDriveMotor(double MPS) {
		this.driveMotor.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(MPS)
						/ SdsModuleConfigurations.MK4_L2.getDriveReduction());
	}

	/**
	 * @param degrees
	 */
	public void setSteerMotor(double degrees) {
		this.steerMotor.set(
				ControlMode.Position,
				degrees * DrivetrainConstants.kIntegratedEncoderTicksPerDegree
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	/**
	 * @return degrees 0 to 360
	 */
	public double getAbsWheelAngle() {
		return this.encoder.getAbsolutePosition();
	}

	public double getSteerIntegratedSensorMeasurment() {
		return this.steerMotor.getSelectedSensorPosition();
	}

	/**
	 * synchronises the steer integrated encoder with the CANCoder measurment,
	 * with consideration of units and gear ratio.
	 */
	public void syncSteerEncoder() {
		this.steerMotor.setSelectedSensorPosition(
				(this.encoder.getAbsolutePosition()
						* DrivetrainConstants.kIntegratedEncoderTicksPerDegree)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	/** Math verified by Noam Geva and Ma'ayan Fucking Bar-El✨ */
	private double MPSToIntegratedEncoderTicksPer100MS(double metersPerSecond) {
		double wheelRotationsPerSec = metersPerSecond / DrivetrainConstants.kWheelCircumferenceCM;
		double motorRotationsPerSec = wheelRotationsPerSec / SdsModuleConfigurations.MK4_L2.getDriveReduction();
		double integratedEncoderCountsPerSec = motorRotationsPerSec *
				(DrivetrainConstants.kIntegratedEncoderTicksPerRev);
		double encoderCountsPer100MS = integratedEncoderCountsPerSec / 10;
		return encoderCountsPer100MS;
	}

	public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
		// make the target angle an equivalent of it between 0 and 360
		double targetAngle = placeInZeroTo360Scope(currentAngle, desiredState.angle.getDegrees());
		double targetMPS = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngle;
		if (Math.abs(delta) > 90) { // If you need to turn more than 90 degrees to either direction
			targetMPS = -targetMPS; // Invert the wheel speed
			if (delta > 90) // If you need to turn > positive 90 degrees
				targetAngle -= 180; // make the target angle in the first half oft circle
			else // If you need to turn > negative 90 degrees, 
				targetAngle += 180; // make the taget angle in the last half
		}
		return new SwerveModuleState(targetMPS, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * @param currentAngle
	 * @param targetAngle
	 * @return Closest angle within scope
	 */
	private static double placeInZeroTo360Scope(double currentAngle, double targetAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = currentAngle % 360;
		if (lowerOffset >= 0) {
			lowerBound = currentAngle - lowerOffset;
			upperBound = currentAngle + (360 - lowerOffset);
		} else {
			upperBound = currentAngle - lowerOffset;
			lowerBound = currentAngle - (360 + lowerOffset);
		}
		while (targetAngle < lowerBound) {
			// Increase the angle by 360 until it's between 0 and 360
			targetAngle += 360;
		}
		while (targetAngle > upperBound) {
			// Decrease the angle by 360 until it's between 0 and 360
			targetAngle -= 360;
		}
		if (targetAngle - currentAngle > 180) {
			// if the difference betwe
			targetAngle -= 360;
		} else if (targetAngle - currentAngle < -180) {
			targetAngle += 360;
		}
		return targetAngle;
	}
}
