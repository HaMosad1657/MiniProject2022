package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	private WPI_TalonFX driveMotor;
	private WPI_TalonFX steerMotor;
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

		// Set CANCoder offset in degrees
		this.encoder.configMagnetOffset(
				steerOffsetDegrees, DrivetrainConstants.kCANCoderTimeoutMs);

		// Make the CANCoder return measurments in 0 to 360.
		// NOTE: the position closed-loop treats the sensor measurments as continuous
		// (similar to the enableContinousInput() method from the WPIlib PIDController
		// class). That's according to Chief Delphi, I couldn't find a formal soucrce
		// on this.
		this.encoder.configAbsoluteSensorRange(
				AbsoluteSensorRange.Unsigned_0_to_360, DrivetrainConstants.kCANCoderTimeoutMs);

		// Constrct the drive motor controller
		this.driveMotor = new WPI_TalonFX(driveID);

		// Set the drive motor to brake neutral mode
		this.driveMotor.setNeutralMode(NeutralMode.Brake);

		// Set the feedback device for the drive motor controller on slot 0
		// as it's integrated encoder.
		this.driveMotor.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor, 0, DrivetrainConstants.kTalonTimeoutMs);

		// Construct the steer motor controller
		this.steerMotor = new WPI_TalonFX(steerID);

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
	 * @param metersPerSecond
	 */
	public void setDriveMotor(double metersPerSecond) {
		this.driveMotor.set(ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(metersPerSecond)
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
	 * @return Degrees 0 to 360
	 */
	public double getAbsWheelAngle() {
		return this.encoder.getAbsolutePosition();
	}

	/**
	 * @return The measurment of the steer motor's
	 *         integrated encoder in raw sensor units
	 */
	public double getSteerIntegratedSensorMeasurment() {
		return this.steerMotor.getSelectedSensorPosition();
	}

	/**
	 * Synchronises the steer integrated encoder with the
	 * CANCoder's measurment, considering units and gear ratio.
	 */
	public void syncSteerEncoder() {
		this.steerMotor.setSelectedSensorPosition(
				(this.encoder.getAbsolutePosition()
						* DrivetrainConstants.kIntegratedEncoderTicksPerDegree)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	/**
	 * 
	 * @param desiredState (a SwerveModuleState object)
	 * @param currentAngleDegrees
	 * @return A SwerveModuleState object with an optimized angle and velocity
	 */
	public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngleDegrees) {
		// Make the target angle an equivalent of it between 0 and 360
		double targetAngle = placeInZeroTo360Scope(currentAngleDegrees, desiredState.angle.getDegrees());
		double targetMPS = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngleDegrees;
		if (Math.abs(delta) > 90) { // If you need to turn more than 90 degrees to either direction
			targetMPS = -targetMPS; // Invert the wheel speed
			if (delta > 90) // If you need to turn > positive 90 degrees
				targetAngle -= 180;
			else // If you need to turn > negative 90 degrees,
				targetAngle += 180;
		}
		return new SwerveModuleState(targetMPS, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * @return The speed of the wheel in meters per second, considering
	 *         gear ratio and wheel circumference.
	 */
	public double getWheelMPS() {
		double integratedEncoderCountsPer100MS = this.driveMotor.getSelectedSensorVelocity();
		double integratedEncoderCountsPerSec = integratedEncoderCountsPer100MS * 10;
		double motorRotationsPerSec = integratedEncoderCountsPerSec / DrivetrainConstants.kIntegratedEncoderTicksPerRev;
		double wheelRotationsPerSec = motorRotationsPerSec * SdsModuleConfigurations.MK4_L2.getDriveReduction();
		double metersPerSecond = wheelRotationsPerSec * DrivetrainConstants.kWheelCircumferenceCM;
		return metersPerSecond;
	}

	@Override
	public String toString() {
		return "\n MPS: " + String.valueOf(this.getWheelMPS()) +
				"\n Angle: " + String.valueOf(this.encoder.getAbsolutePosition());
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

	/**
	 * @param currentAngleDegrees
	 * @param targetAngleDegrees
	 * @return Closest angle within scope
	 */
	private static double placeInZeroTo360Scope(double currentAngleDegrees, double targetAngleDegrees) {
		double lowerBound;
		double upperBound;
		double lowerOffset = currentAngleDegrees % 360;
		if (lowerOffset >= 0) {
			lowerBound = currentAngleDegrees - lowerOffset;
			upperBound = currentAngleDegrees + (360 - lowerOffset);
		} else {
			upperBound = currentAngleDegrees - lowerOffset;
			lowerBound = currentAngleDegrees - (360 + lowerOffset);
		}
		while (targetAngleDegrees < lowerBound) {
			// Increase the angle by 360 until it's something between 0 and 360
			targetAngleDegrees += 360;
		}
		while (targetAngleDegrees > upperBound) {
			// Decrease the angle by 360 until it's something between 0 and 360
			targetAngleDegrees -= 360;
		}
		// If the difference between the target and current angle
		// is more than 180, decrease the the target angle by 360
		if (targetAngleDegrees - currentAngleDegrees > 180) {
			targetAngleDegrees -= 360;
		}
		// If the difference between the target and current angle
		// is less than -180, increase the target angle by 360
		else if (targetAngleDegrees - currentAngleDegrees < -180) {
			targetAngleDegrees += 360;
		}
		return targetAngleDegrees;
	}
}
