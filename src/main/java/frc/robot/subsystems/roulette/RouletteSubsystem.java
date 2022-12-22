
package frc.robot.subsystems.roulette;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.hamosad1657.lib.motors.HaTalonSRX;
import com.hamosad1657.lib.sensors.HaColorSensor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RouletteSubsystem extends SubsystemBase {
	private static RouletteSubsystem instance;

	public static RouletteSubsystem getInstance() {
		if (instance == null) {
			instance = new RouletteSubsystem();
		}
		return instance;
	}

	private final ShuffleboardTab tab;
	private final HaColorSensor colorSensor;
	private final HaTalonSRX armMotor;
	private final HaTalonSRX rotationMotor;

	private RouletteSubsystem() {
		this.tab = Shuffleboard.getTab("ColorSensor");
		this.colorSensor = new HaColorSensor(I2C.Port.kMXP, this.tab);
		this.armMotor = new HaTalonSRX(new WPI_TalonSRX(RouletteConstants.kRouletteArmMotor));
		this.rotationMotor = new HaTalonSRX(new WPI_TalonSRX(RouletteConstants.kRotateRouletteMotor));
	}

	@Override
	public void periodic() {

	}

	public double getBlue() {
		return this.colorSensor.getBlue();
	}

	public double getGreen() {
		return this.colorSensor.getGreen();
	}

	public double getRed() {
		return this.colorSensor.getRed();
	}

	private double getProximity() {
		return this.colorSensor.getProximity();
	}

	private void setArmMotor(double speed) {
		this.armMotor.set(speed);
	}

	public void setRotationMotor(double speed) {
		this.rotationMotor.set(speed);
	}

	public Command getOpenArmCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.setArmMotor(0.5)),
				new ParallelRaceGroup(
						new WaitCommand(0.2),
						new WaitUntilCommand(() -> this.getProximity() < RouletteConstants.kWantedProximity)),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}

	public Command getCloseArmCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.setArmMotor(-0.5)),
				new WaitCommand(0.2),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}

	public boolean isBlue() {
		return this.colorSensor.isColorInRange(RouletteConstants.kMinBlue, RouletteConstants.kMaxBlue);
	}

	public Alliance getFullColor() {
		if (isBlue())
			return Alliance.Blue;
		return Alliance.Red;
	}

}

/*
 * abstract
 * To do : fuse all of the seperate commands & subsystems to here
 * command: put the rotate roulette command here
 * delete rest of files
 */