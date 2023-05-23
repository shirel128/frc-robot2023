package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTimerCommand extends CommandBase {
  double _init;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmTimerCommand() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _init = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("ArmTimer",2 - (Timer.getFPGATimestamp() - _init));
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 2 - (Timer.getFPGATimestamp() - _init) == 0;
  }
}
