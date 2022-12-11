// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.disks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisksSubsystem extends SubsystemBase {
	private static DisksSubsystem instance;

	public static DisksSubsystem getInstance() {
		if (instance == null) {
			instance = new DisksSubsystem();
		}
		return instance;
	}

	final private CANSparkMax angleMotor;
	final private RelativeEncoder angleEncoder;

	final private CANSparkMax telescopicMotor;
	final private RelativeEncoder telescopicEncoder;

	final private WPI_TalonSRX grabberMotor;
	final private CANCoder grabberEncoder;
	private boolean isGrabberOpened;

	private DisksSubsystem() {
		this.angleMotor = new CANSparkMax(DisksConstants.kAngleMotorID, MotorType.kBrushless);
		this.angleEncoder = this.angleMotor.getEncoder();

		this.telescopicMotor = new CANSparkMax(DisksConstants.kTelescopicMotorID, MotorType.kBrushless);
		this.telescopicEncoder = this.telescopicMotor.getEncoder();

		this.grabberMotor = new WPI_TalonSRX(DisksConstants.kGrabberMotorID);
		this.grabberMotor.config_kP(DisksConstants.kGrabberPIDSlotIndex, DisksConstants.kGrabberP);
		this.grabberMotor.config_kI(DisksConstants.kGrabberPIDSlotIndex, DisksConstants.kGrabberI);
		this.grabberMotor.config_kD(DisksConstants.kGrabberPIDSlotIndex, DisksConstants.kGrabberD);
		this.grabberMotor.config_kF(DisksConstants.kGrabberPIDSlotIndex, DisksConstants.kGrabberFF);
		this.grabberMotor.setSafetyEnabled(false); // Safety FIRST!
		this.grabberMotor.configRemoteFeedbackFilter(
				DisksConstants.kGrabberEncoderID, RemoteSensorSource.CANCoder,
				DisksConstants.kGrabberRemoteSensorIndex);

		this.grabberEncoder = new CANCoder(DisksConstants.kGrabberEncoderID);
		this.isGrabberOpened = false;
	}

	/**
	 * Grabber is closed when it can hold a disk.
	 */
	public void toggleGrabber() {
		if (this.isGrabberOpened) {
			this.grabberMotor.set(ControlMode.Position, DisksConstants.kGrabberClosedPosition);
		} else {
			this.grabberMotor.set(ControlMode.Position, DisksConstants.kGrabberOpenedPosition);
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
	public void setTelescopicMotor(double speed) {
		this.telescopicMotor.set(speed);
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
	 * @return Number of rotations that the motor did since powerup.
	 */
	public double getTelescopicPosition() {
		return this.telescopicEncoder.getPosition();
	}

	/**
	 * @return Angle of the arm.
	 */
	public double getAngle() {
		return (this.angleEncoder.getPosition() / DisksConstants.kAngleMotorGearRatio) * 360;
	}

	public void resetEncoders() {
		this.angleEncoder.setPosition(0.0);
		this.telescopicEncoder.setPosition(0.0);
		this.grabberEncoder.setPosition(0.0);
	}

	@Override
	public void periodic() {}
}

/**
 * a Neo for controlling the angle of the arm
 * a Neo for controlling the opening and closing of the arm
 * a BAG and CANCoder for controlling the grabber
 */
