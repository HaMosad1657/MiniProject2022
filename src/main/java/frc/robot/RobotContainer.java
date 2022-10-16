package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;

/**
 * BUTTONS BINDINGS:
 * Triangle - 
 * Circle -
 * Ex - 
 * Square -
 */

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  public static PS4Controller controller;

  public RobotContainer() {
    controller = new PS4Controller(RobotConstants.kControllerUSBPort);
    configureButtonBindings();
    setDefaultCommands();
  }

  private void configureButtonBindings() {

  }

  private void setDefaultCommands() {

  }
}
