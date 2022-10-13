package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  Command m_autoCommand;

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {

    return m_autoCommand;
  }
}
