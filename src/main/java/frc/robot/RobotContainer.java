package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.roulette.RotateRouletteCommand;
import frc.robot.subsystems.roulette.RouletteSubsystem;

public class RobotContainer {
	public static PS4Controller controller;
	private RouletteSubsystem roulette;
	private JoystickButton trianglebButton;

	public RobotContainer() {
		controller = new PS4Controller(RobotConstants.kControllerUSBPort);

		this.trianglebButton = new JoystickButton(controller, Button.kTriangle.value);

		this.roulette = RouletteSubsystem.getInstance();
		this.setDeafultCommands();
		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// start roulette spinning
		trianglebButton.whileHeld(new SequentialCommandGroup(this.roulette.getOpenArmCommand(),
				new RotateRouletteCommand(this.roulette)).andThen(this.roulette.getCloseArmCommand()));
	}

	private void setDeafultCommands() {

	}
}
