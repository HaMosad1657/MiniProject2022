
package com.hamosad1657.lib.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.hamosad1657.lib.HaUnits;
import com.hamosad1657.lib.HaUnitConvertor;
import com.hamosad1657.lib.motors.HaTalonFX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * @author Shaked - ask me if you have questions🌠
 */
public class HaSwerveModule {
	private final double wheelRadiusM;

	private final HaTalonFX steerMotor, driveMotor;
	private final WPI_TalonFX steerTalonFX, driveTalonFX;
	private final CANCoder steerEncoder;

	/**
	 * Constructs a swerve module with a CANCoder and two Falcons.
	 */
	public HaSwerveModule(
			int steerMotorControllerID, int driveMotorControllerID, int steerCANCoderID,
			double steerOffsetDegrees, double wheelRadiusM, HaUnits.PIDGains steerPidGains,
			HaUnits.PIDGains drivePidGains) {
		this.wheelRadiusM = wheelRadiusM;

		this.steerEncoder = new CANCoder(steerCANCoderID);
		this.steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		this.steerEncoder.configMagnetOffset(steerOffsetDegrees);
		this.steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

		// Steer motor
		this.steerTalonFX = new WPI_TalonFX(steerMotorControllerID);
		this.steerMotor = new HaTalonFX(this.steerTalonFX, steerPidGains, this.wheelRadiusM,
				FeedbackDevice.IntegratedSensor);
		this.steerMotor.setIdleMode(IdleMode.kBrake);

		// Drive motor
		this.driveTalonFX = new WPI_TalonFX(driveMotorControllerID);
		this.driveMotor = new HaTalonFX(this.driveTalonFX, drivePidGains, this.wheelRadiusM,
				FeedbackDevice.IntegratedSensor);
		this.driveMotor.setIdleMode(IdleMode.kBrake);

		this.syncSteerEncoder();
	}

	/**
	 * Synchronises the integrated steer encoder with the
	 * CANCoder's measurment, considering units and gear ratio.
	 */
	public void syncSteerEncoder() {
		// The CANCoder returns the angle of the wheel, but we want the motor, so we devide it by the gear ratio.
		this.steerMotor.setEncoderPosition(
				this.steerEncoder.getAbsolutePosition()
						/ SdsModuleConfigurations.MK4_L2.getSteerReduction(),
				HaUnits.Position.kDegrees);
	}

	/**
	 * @param pidGains
	 * @throws IllegalArgumentException
	 *             If one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setSteerPID(HaUnits.PIDGains pidGains)
			throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (pidGains.p < 0 || pidGains.i < 0 || pidGains.d < 0)
			throw new IllegalArgumentException("PID gains cannot be negative.");
		// If IZone is negative
		if (pidGains.iZone < 0)
			throw new IllegalArgumentException("IZone cannot be negative.");

		this.steerMotor.configPID(pidGains);
	}

	/**
	 * @param pidGains
	 * @throws IllegalArgumentException
	 *             If one or more of the PID gains are
	 *             negative, or if IZone is negative.
	 */
	public void setDrivePID(HaUnits.PIDGains pidGains)
			throws IllegalArgumentException {
		// If one of the PID gains are negative
		if (pidGains.p < 0 || pidGains.i < 0 || pidGains.d < 0)
			throw new IllegalArgumentException("PID gains cannot be negative.");
		// If IZone is negative
		if (pidGains.iZone < 0)
			throw new IllegalArgumentException("IZone cannot be negative.");

		this.driveMotor.configPID(pidGains);
	}

	/**
	 * @return The current wheel speed and angle as a {@link SwerveModuleState} object.
	 */
	public SwerveModuleState getSwerveModuleState() {
		return new SwerveModuleState(
				this.getWheelMPS(),
				this.getAbsWheelAngleRotation2d());
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
		return HaUnitConvertor.degPSToMPS(this.steerEncoder.getVelocity(), this.wheelRadiusM);
	}

	/**
	 * Preforms velocity and position closed-loop control on the
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
	 * @param angleDegrees
	 *            of the wheel
	 */
	public void setSteerMotor(double angleDegrees) {
		this.steerMotor.set(
				angleDegrees / SdsModuleConfigurations.MK4_L2.getSteerReduction(),
				HaUnits.Position.kDegrees);
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
	 *            of the wheel
	 */
	public void setDriveMotor(double MPS) {
		this.driveMotor.set(
				MPS / SdsModuleConfigurations.MK4_L2.getDriveReduction(),
				HaUnits.Velocity.kMPS);
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