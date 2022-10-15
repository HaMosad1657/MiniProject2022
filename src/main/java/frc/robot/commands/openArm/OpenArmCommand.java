
package frc.robot.commands.openArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armMover.ArmMoverSubsystem;

public class OpenArmCommand extends CommandBase {
  private ArmMoverSubsystem armMover;

  public OpenArmCommand(ArmMoverSubsystem armMover) { // might need to add color sensor here to check if it needs arm
                                                      // needs to stop
    this.armMover = armMover;
    this.addRequirements(this.armMover);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
