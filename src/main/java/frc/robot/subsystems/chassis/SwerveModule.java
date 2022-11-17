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

public class SwerveModule {
	private TalonFX driveMotor;
	private TalonFX steerMotor;
	private CANCoder encoder;

	public SwerveModule(int driveID, int steerID, int CANCoderID, double steerOffsetDegrees) {
		// Construct the CANCoder
		this.encoder = new CANCoder(CANCoderID);

		// Make the CANCoder boot to it's absloute value instead of to 0
		this.encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		;

		// Set the CANCoder measurment coefficient to 0.087890625 so that it returns
		// degrees (this is the default)
		this.encoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);

		// Set CANCoder offset
		this.encoder.configMagnetOffset(DrivetrainConstants.kFrontLeftAngleOffset);

		// Make the CANCoder return measurments in 0 to 360.
		// NOTE: the position closed-loop treats the sensor measurments as continuous
		// (similar to the enableContinousInput() method from the WPIlib PIDController
		// class).
		// ADDITIONAL NOTE: as a feedback device, the CANCoder reports to the Talon a
		// value in raw sensor units (0 to 4096) and not in degrees.
		this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

		// Constrct the drive motor controller
		this.driveMotor = new TalonFX(DrivetrainConstants.kFrontLeftDriveMotorID);

		// Set the drive motor to brake neutral mode
		this.driveMotor.setNeutralMode(NeutralMode.Brake);

		// Set the feedback device for the drive motor controller as it's
		// integrated encoder
		this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		// Construct the steer motor controller
		this.steerMotor = new TalonFX(DrivetrainConstants.kFrontLeftAngleMotorID);

		// Set the steer motor to brake neutral mode
		this.steerMotor.setNeutralMode(NeutralMode.Brake);

		// Set the feedback device for the steer motor controllers as it's integrated
		// encoder.
		this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		// Set the integrated encoder to the absloute value (of the motor shaft, not
		// the wheel) using the CANCoder, which is absloute.
		this.steerMotor.setSelectedSensorPosition(
				this.encoder.getAbsolutePosition() *
						(DrivetrainConstants.kIntegratedEncoderTicksPerDegree)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
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
		this.driveMotor.config_kP(slot, Proportional);
		this.driveMotor.config_kI(slot, Integral);
		this.driveMotor.config_kD(slot, Derivative);
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
		this.steerMotor.config_kP(slot, Proportional);
		this.steerMotor.config_kI(slot, Integral);
		this.steerMotor.config_kD(slot, Derivative);
	}

	public void setDriveMotor(double MPS) {
		this.driveMotor.set(ControlMode.Velocity, this.MPSToIntegratedEncoderTicksPer100MS(MPS));
	}

	public void setSteerMotor(double degrees) {
		this.steerMotor.set(
				ControlMode.Position,
				degrees * DrivetrainConstants.kIntegratedEncoderTicksPerDegree
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	public double getAbsWheelAngle() {
		return this.encoder.getAbsolutePosition();
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
}