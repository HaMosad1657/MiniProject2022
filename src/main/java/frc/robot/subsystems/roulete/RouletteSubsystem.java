
package frc.robot.subsystems.roulete;

import com.hamosad1657.lib.sensors.HaColorSensor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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

	private RouletteSubsystem() {
		this.tab = Shuffleboard.getTab("ColorSensor");
		this.colorSensor = new HaColorSensor(I2C.Port.kMXP, this.tab);
	}

	@Override
	public void periodic() {

	}

	private double getProximity() {
		return this.colorSensor.getProximity();
	}

	private void setArmMotor(double speed) {

	}

	private Command getOpenArmCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.setArmMotor(0.5)),
				new ParallelRaceGroup(
						new WaitCommand(0.2),
						new WaitUntilCommand(() -> this.getProximity() < 10)),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}

	private Command getCloseArmCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.setArmMotor(-0.5)),
				new ParallelRaceGroup(
						new WaitCommand(0.2),
),
				new InstantCommand(() -> this.setArmMotor(0.0)));
	}
}

/*
 * abstract
 * To do : fuse all of the seperate commands & subsystems to here
 * command: put the rotate roulette command here
 * delete rest of files
 */