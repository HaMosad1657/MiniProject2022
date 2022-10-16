
package frc.robot.commands.closeArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armMover.ArmMoverSubsystem;

public class CloseArmCommand extends CommandBase {
  private ArmMoverSubsystem armMover;
  private boolean finished;

  public CloseArmCommand(ArmMoverSubsystem armMover) {
    this.armMover = armMover;
    finished = false;
    this.addRequirements(this.armMover);
  }

  @Override
  public void initialize() {
    armMover.setMotor(CloseArmConstants.kDeafultSpeed);
    // put waiting function here.
    finished = true;
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    armMover.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
