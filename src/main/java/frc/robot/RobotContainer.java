package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.rotateRoulette.RotateRouletteCommand;
import frc.robot.commands.roulete.closeArm.CloseArmCommand;
import frc.robot.commands.roulete.openArm.OpenArmCommand;
import frc.robot.subsystems.armMover.ArmMoverSubsystem;
import frc.robot.subsystems.colorSensor.ColorSensorSubsystem;
import frc.robot.subsystems.rotationWheel.RotationWheelSubsystem;

public class RobotContainer {
	public static PS4Controller controller;

	private RotationWheelSubsystem rotationWheel;
	private ArmMoverSubsystem armMover;
	private ColorSensorSubsystem colorSensor;
	private JoystickButton trianglebButton;

	public RobotContainer() {
		controller = new PS4Controller(RobotConstants.kControllerUSBPort);

		this.trianglebButton = new JoystickButton(controller, Button.kTriangle.value);

		this.armMover = ArmMoverSubsystem.getInstance();
		this.colorSensor = ColorSensorSubsystem.getInstance();
		this.rotationWheel = RotationWheelSubsystem.getInstance();

		this.setDeafultCommands();
		this.configureButtonBindings();
	}

	private void configureButtonBindings() {
		// start roulette spinning
		trianglebButton.whenPressed(new SequentialCommandGroup(new OpenArmCommand(armMover, colorSensor),
				new RotateRouletteCommand(rotationWheel, colorSensor), new CloseArmCommand(armMover)));
	}

	private void setDeafultCommands() {

	}
}
