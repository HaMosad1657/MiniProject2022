// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.disks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisksSubsystem extends SubsystemBase {
	private static DisksSubsystem instance;

	public static DisksSubsystem getInstance() {
		if (instance == null) {
			instance = new DisksSubsystem();
		}
		return instance;
	}

	private final CANSparkMax angleMotor, extendMotor;
	private final RelativeEncoder angleEncoder, extendEncoder;

	private final WPI_TalonSRX grabberMotor;
	private final CANCoder grabberEncoder;
	private final PIDController grabberPIDController;
	private boolean isGrabberOpened;

	private DisksSubsystem() {
		this.angleMotor = new CANSparkMax(DisksConstants.kAngleMotorID, MotorType.kBrushless);
		this.angleMotor.setIdleMode(IdleMode.kBrake);
		this.angleEncoder = this.angleMotor.getEncoder();
		this.angleEncoder.setPositionConversionFactor(360.0 / DisksConstants.kAngleMotorGearRatio);

		this.extendMotor = new CANSparkMax(DisksConstants.kTelescopicMotorID, MotorType.kBrushless);
		this.extendMotor.setIdleMode(IdleMode.kBrake);
		this.extendEncoder = this.extendMotor.getEncoder();

		this.grabberMotor = new WPI_TalonSRX(DisksConstants.kGrabberMotorID);
		this.grabberMotor.setNeutralMode(NeutralMode.Brake);
		this.grabberMotor.setSafetyEnabled(false); // Safety FIRST!
		this.grabberMotor.configSelectedFeedbackSensor(FeedbackDevice.None);

		this.grabberPIDController = new PIDController(
				DisksConstants.kGrabberP,
				DisksConstants.kGrabberI,
				DisksConstants.kGrabberD);
		this.grabberPIDController.setTolerance(DisksConstants.kGrabberTolerance);
		this.grabberPIDController.enableContinuousInput(0.0, 360.0);
		this.grabberPIDController.setSetpoint(DisksConstants.kGrabberClosedPosition);

		this.grabberEncoder = new CANCoder(DisksConstants.kGrabberEncoderID);
		this.isGrabberOpened = false;

		var tab = Shuffleboard.getTab("Disks");
		tab.addNumber("Grabber Position",
				() -> (this.grabberEncoder.getAbsolutePosition()));
		tab.addNumber("Telescopic Angle",
				() -> this.getTelescopicAngle());
		tab.addNumber("Telescopic Extend Position",
				() -> this.getExtendPosition());
		tab.addBoolean("Is Grabber Opened", () -> this.isGrabberOpened);
	}

	/**
	 * Grabber is opened when it holds a disk.
	 */
	public void toggleGrabber() {
		if (this.isGrabberOpened) {
			this.grabberPIDController.setSetpoint(DisksConstants.kGrabberClosedPosition);
		} else {
			this.grabberPIDController.setSetpoint(DisksConstants.kGrabberOpenedPosition);
		}
		this.isGrabberOpened = !this.isGrabberOpened;
	}

	public boolean isGrabberOpened() {
		return this.isGrabberOpened;
	}

	/**
	 * @param speed
	 *            [-1, 1]
	 */
	public void setExtendMotor(double speed) {
		this.extendMotor.set(speed);
	}

	/**
	 * @param speed
	 *            [-1, 1]
	 */
	public void setAngleMotor(double speed) {
		this.angleMotor.set(speed);
	}

	/**
	 * 
	 * @return Number of rotations that the motor did since powerup or since last call to resetEncoders().
	 */
	public double getExtendPosition() {
		return -this.extendEncoder.getPosition();
	}

	/**
	 * @return Angle of the arm. Starts as zero on powerup, and can be reset using resetEncoders().
	 */
	public double getTelescopicAngle() {
		return -this.angleEncoder.getPosition();
	}

	public void resetEncoders() {
		this.angleEncoder.setPosition(0.0);
		this.extendEncoder.setPosition(0.0);
	}

	@Override
	public void periodic() {
		if (grabberPIDController.atSetpoint())
			this.grabberMotor.set(ControlMode.PercentOutput, 0.0);
		else
			this.grabberMotor.set(
					ControlMode.PercentOutput,
					this.grabberPIDController.calculate(this.grabberEncoder.getAbsolutePosition()));

	}
}
