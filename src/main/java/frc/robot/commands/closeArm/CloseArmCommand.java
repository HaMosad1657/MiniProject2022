
package frc.robot.commands.closeArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armMover.ArmMoverSubsystem;

public class CloseArmCommand extends CommandBase {
  private ArmMoverSubsystem armMover;

  public CloseArmCommand(ArmMoverSubsystem armMover) {
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
