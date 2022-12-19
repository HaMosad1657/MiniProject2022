package com.hamosad1657.lib.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * @author Shaked - ask me if you have questions🌠
 */
public class HaSwerveModule {

	private final WPI_TalonFX steerMotor, driveMotor;
	private final CANCoder steerEncoder;

	private final double kWheelCircumferenceM;

	/**
	 * Constructs a swerve module with a CANCoder and two Falcons.
	 */
	public HaSwerveModule(
			int steerMotorControllerID, int driveMotorControllerID, int steerCANCoderID,
			double steerOffsetDegrees, double wheelDiameterCM) {

		this.kWheelCircumferenceM = (wheelDiameterCM / 100) * Math.PI;

		this.steerEncoder = new CANCoder(steerCANCoderID);
		this.steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		this.steerEncoder.configMagnetOffset(steerOffsetDegrees);
		this.steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

		this.steerMotor = new WPI_TalonFX(steerMotorControllerID);
		this.steerMotor.setNeutralMode(NeutralMode.Brake);
		this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.syncSteerEncoder();

		this.driveMotor = new WPI_TalonFX(driveMotorControllerID);
		this.driveMotor.setNeutralMode(NeutralMode.Brake);
		this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	}

	/**
	 * Synchronises the integrated steer encoder with the
	 * CANCoder's measurment, considering units and gear ratio.
	 */
	public void syncSteerEncoder() {
		this.steerMotor.setSelectedSensorPosition(
				(this.steerEncoder.getAbsolutePosition()
						* HaSwerveConstants.kTalonFXIntegratedEncoderTicksPerDeg)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	/**
	 * @param P
	 *            - Proportional gain.
	 * @param I
	 *            - Integral gain.
	 * @param D
	 *            - Derivative gain.
	 * @param IZone
	 *            if the absloute closed-loop error is above IZone, the
	 *            Integral accumulator is cleared (making it ineffective).
	 * @throws IllegalArgumentException
	 *             If one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setSteerPID(
			double P, double I, double D, double IZone)
			throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (P < 0 || I < 0 || D < 0)
			throw new IllegalArgumentException("PID gains cannot be negative.");
		// If IZone is negative
		if (IZone < 0)
			throw new IllegalArgumentException("IZone cannot be negative.");

		this.configPID(this.steerMotor, P, I, D, IZone);
	}

	/**
	 * @param P
	 *            - Proportional gain.
	 * @param I
	 *            - Integral gain.
	 * @param D
	 *            - Derivative gain.
	 * @throws IllegalArgumentException
	 *             If one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setSteerPID(
			double P, double I, double D)
			throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (P < 0 || I < 0 || D < 0)
			throw new IllegalArgumentException("PID gains cannot be negative.");

		this.configPID(this.steerMotor, P, I, D, 0.0);
	}

	/**
	 * @param P
	 *            - Proportional gain.
	 * @param I
	 *            - Integral gain.
	 * @param D
	 *            - Derivative gain.
	 * @param IZone
	 *            if the absloute closed-loop error is above IZone, the
	 *            Integral accumulator is cleared (making it ineffective).
	 * @throws IllegalArgumentException
	 *             if one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setDrivePID(double P, double I, double D, double IZone) throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (P < 0 || I < 0 || D < 0)
			throw new IllegalArgumentException("PID gains cannot be negative");
		// If IZone is negative
		if (IZone < 0)
			throw new IllegalArgumentException("IZone cannot be negative");

		this.configPID(this.driveMotor, P, I, D, IZone);
	}

	/**
	 * @param P
	 *            - Proportional gain.
	 * @param I
	 *            - Integral gain.
	 * @param D
	 *            - Derivative gain.
	 * @throws IllegalArgumentException
	 *             if one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setDrivePID(double P, double I, double D) throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (P < 0 || I < 0 || D < 0)
			throw new IllegalArgumentException("PID gains cannot be negative");

		this.configPID(this.driveMotor, P, I, D, 0.0);
	}

	/**
	 * @return The current wheel speed and angle as a {@link SwerveModuleState} object.
	 */
	public SwerveModuleState getSwerveModuleState() {
		return new SwerveModuleState(
				this.getWheelMPS(),
				Rotation2d.fromDegrees(this.getAbsWheelAngleDeg()));
	}

	/**
	 * @return The absloute angle of the wheel in degrees.
	 */
	public double getAbsWheelAngleDeg() {
		return this.steerEncoder.getAbsolutePosition();
	}

	/**
	 * @return The absloute angle of the wheel as a {@link Rotation2d} object.
	 */
	public Rotation2d getAbsWheelAngleRotation2d() {
		return Rotation2d.fromDegrees(this.steerEncoder.getAbsolutePosition());
	}

	/**
	 * @return The speed of the wheel in meters per second.
	 */
	public double getWheelMPS() {
		double integratedEncoderTicksPS = this.driveMotor.getSelectedSensorVelocity() * 10;
		double motorRevPS = integratedEncoderTicksPS /
				HaSwerveConstants.kTalonFXIntegratedEncoderTicksPerRev;
		double wheelRevPS = motorRevPS * SdsModuleConfigurations.MK4_L2.getDriveReduction();
		return wheelRevPS * this.kWheelCircumferenceM;
	}

	/**
	 * Performs velocity and position closed-loop control on the
	 * steer and drive motors, respectively. The control runs on
	 * the motor controllers.
	 */
	public void setSwerveModuleState(SwerveModuleState moduleState) {
		this.setDriveMotor(moduleState.speedMetersPerSecond);
		this.setSteerMotor(moduleState.angle);
	}

	/**
	 * Preforms position closed-loop control on the
	 * steer motor (runs on the motor controller).
	 * 
	 * @param angleDeg
	 *            - The angle of the wheel in degrees.
	 */
	public void setSteerMotor(double angleDeg) {
		this.steerMotor.set(
				ControlMode.Position,
				(angleDeg * HaSwerveConstants.kTalonFXIntegratedEncoderTicksPerDeg)
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction());
	}

	/**
	 * Preforms position closed-loop control on the
	 * steer motor (runs on the motor controller).
	 * 
	 * @param Rotaton2d
	 *            - The angle of the wheel as {@link Rotation2d}.
	 */
	public void setSteerMotor(Rotation2d angle) {
		this.setSteerMotor(angle.getDegrees());
	}

	/**
	 * Preforms velocity closed-loop control on the
	 * drive motor (runs on the motor controller).
	 * 
	 * @param MPS
	 *            THe velocity of the wheel in meters per second.
	 */
	public void setDriveMotor(double MPS) {
		this.driveMotor.set(
				ControlMode.Velocity,
				this.MPSToIntegratedEncoderTicksPer100MS(MPS)
						/ SdsModuleConfigurations.MK4_L2.getDriveReduction());
	}

	private void configPID(WPI_TalonFX talonFX, double P, double I, double D, double IZone) {
		talonFX.config_kP(0, P);
		talonFX.config_kI(0, I);
		talonFX.config_kD(0, D);
		talonFX.config_IntegralZone(0, IZone);
	}

	/**
	 * Converts meters per second to the units the TalonFX uses for position
	 * feedback, which are integrated encoder ticks per 100 miliseconds.
	 * 
	 * @param MPS
	 * @return integrated encoder ticks per 100 ms
	 */
	private double MPSToIntegratedEncoderTicksPer100MS(double MPS) {
		double wheelRevPS = MPS / this.kWheelCircumferenceM;
		double motorRevPS = wheelRevPS
				/ SdsModuleConfigurations.MK4_L2.getDriveReduction();
		double integratedEncoderTicksPS = motorRevPS *
				(HaSwerveConstants.kTalonFXIntegratedEncoderTicksPerRev);
		double encoderTicksPer100MS = integratedEncoderTicksPS / 10;
		return encoderTicksPer100MS;
	}

	@Override
	public String toString() {
		return "\n MPS: " + String.valueOf(this.getWheelMPS()) +
				"\n Angle: " + String.valueOf(this.steerEncoder.getAbsolutePosition());
	}

	/**
	 * Our optimization method! Please do question it's correctness if the swerve doesn't behave as intended.
	 * A replacement for this method is optimizeWithWPI(), which is WPILib's SwerveModuleState.optimize() but wrapped
	 * in this class. you can also use WPILib's method directly.
	 * <p>
	 * Optimizing is minimizing the change in angle that is required to get to the desired heading, by potentially
	 * reversing and calculatinga new spin direction for the wheel. If this is used with a PID controller that has
	 * continuous input for position control, then the maximum rotation will be 90 degrees.
	 * 
	 * @param desiredState
	 *            - The desired {@link SwerveModuleState} for the module.
	 * @param currentAngleDeg
	 *            - The current steer angle of the module (in the degrees).
	 * @return The optimized {@link SwerveModuleState}
	 */
	public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngleDeg) {
		double targetMPS = desiredState.speedMetersPerSecond;
		double targetAngle = placeIn0To360Scope(desiredState.angle.getDegrees(), currentAngleDeg);

		// Check if you need to turn more than 90 degrees to either direction
		double delta = targetAngle - currentAngleDeg;
		if (Math.abs(delta) > 90) {
			targetMPS = -targetMPS; // Invert the wheel speed

			// Add / subtract 180 from the target angle (depending on the direction)
			if (delta > 90)
				targetAngle -= 180;
			else
				targetAngle += 180;
		}

		return new SwerveModuleState(targetMPS, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * WPILib's SwerveModuleState.optimize(), wrapped.
	 * <p>
	 * Optimizing means to minimize the change in heading the
	 * desired swerve module state would require, by potentially
	 * reversing the direction the wheel spins. If this is used
	 * with a PID controller that has continuous input for
	 * position control, then the most the wheel will rotate is
	 * 90 degrees.
	 * 
	 * @return The optimized {@link SwerveModuleState}.
	 */
	public static SwerveModuleState optimizeWithWPI(
			SwerveModuleState desiredState, Rotation2d currentRotation) {
		return SwerveModuleState.optimize(desiredState, currentRotation);
	}

	/**
	 * WPILib's SwerveModuleState.optimize(), wrapped.
	 * <p>
	 * Optimizing means to minimize the change in heading the
	 * desired swerve module state would require, by potentially
	 * reversing the direction the wheel spins. If this is used
	 * with a PID controller that has continuous input for
	 * position control, then the most the wheel will rotate is
	 * 90 degrees.
	 * 
	 * @return The optimized {@link SwerveModuleState}.
	 */
	public static SwerveModuleState optimizeWithWPI(SwerveModuleState desiredState, double currentAngleDegrees) {
		return SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(currentAngleDegrees));
	}


	// TODO: format and add comments (it's taken from 1678)
	private static double placeIn0To360Scope(double currentAngle, double desiredAngle) {
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
		while (desiredAngle < lowerBound) {
			desiredAngle += 360;
		}
		while (desiredAngle > upperBound) {
			desiredAngle -= 360;
		}
		if (desiredAngle - currentAngle > 180) {
			desiredAngle -= 360;
		} else if (desiredAngle - currentAngle < -180) {
			desiredAngle += 360;
		}
		return desiredAngle;
	}
}