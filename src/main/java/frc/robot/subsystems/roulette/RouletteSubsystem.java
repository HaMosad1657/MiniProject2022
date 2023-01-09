
package frc.robot.subsystems.roulette;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.hamosad1657.lib.HaUnits.PIDGains;
import com.hamosad1657.lib.motors.HaTalonSRX;
import com.hamosad1657.lib.sensors.HaColorSensor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
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

	private int rotationCountval;
	private int isFinishedval;
	GenericEntry rotCount;
	private GenericEntry isFinished;

	private RouletteSubsystem() {
		this.tab = Shuffleboard.getTab("ColorSensor");
		this.colorSensor = new HaColorSensor(I2C.Port.kOnboard, this.tab);
		this.armMotor = new HaTalonSRX(new WPI_TalonSRX(RouletteConstants.kRouletteArmMotorID),
				new PIDGains(0, 0, 0, 0), 0, FeedbackDevice.None);
		this.rotationMotor = new HaTalonSRX(new WPI_TalonSRX(RouletteConstants.kRotateRouletteMotorID),
				new PIDGains(0, 0, 0, 0), 0, FeedbackDevice.None);
		this.rotCount = this.tab.add("Semi Rotation Count", 0).getEntry();
		this.isFinished = this.tab.add("Is Finished", 0).getEntry();
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
				new InstantCommand(() -> this.setArmMotor(RouletteConstants.kDeafultSpeed)),
				new ParallelRaceGroup(
						new WaitCommand(RouletteConstants.kArmOpenCloseWaitDuration),
						new WaitUntilCommand(() -> this.getProximity() < RouletteConstants.kRouletteProximity)),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}

	public Command getCloseArmCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.setArmMotor(-RouletteConstants.kDeafultSpeed)),
				new WaitCommand(RouletteConstants.kArmOpenCloseWaitDuration),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}

	public Alliance getRouletteColor() {
		if (this.colorSensor.isColorInRange(RouletteConstants.kMinBlue, RouletteConstants.kMaxBlue))
			return Alliance.Blue;
		else
			return Alliance.Red;
	}

	public static Alliance getOppositeAlliance(Alliance alliance) {
		if (alliance == Alliance.Blue)
			return Alliance.Red;
		else
			return Alliance.Blue;
	}

	public static int getRequiredRotations(Alliance robotAlliance, Alliance rouletteColor) {
		return (robotAlliance == rouletteColor)
				? RouletteConstants.kMinSemiRotation
				: RouletteConstants.kMaxSemiRotation;
	}

	public void updateRotationCount(int rotationCount) {
		this.rotationCountval = rotationCount;
	}

	public void resetRotCount() {
		this.rotationCountval = 0;
	}

	public void finish() {
		this.isFinishedval++;
	}

	@Override
	public void periodic() {
		this.colorSensor.updateShuffleboardValues();
		this.rotCount.setDouble(this.rotationCountval);
		this.isFinished.setDouble(this.isFinishedval);
	}

}

/*
 * abstract
 * To do : fuse all of the seperate commands & subsystems to here
 * command: put the rotate roulette command here
 * delete rest of files
 */