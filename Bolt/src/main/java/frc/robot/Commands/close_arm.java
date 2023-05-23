// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;

public class close_arm extends CommandBase {
  PistonedTelescopicArm _arm; 
  Intake _Intake;
  /** Creates a new close_arm. */
  public close_arm(PistonedTelescopicArm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm=arm; 
    _Intake = intake;
    addRequirements(_arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!_Intake.get_bottom_limit_Switch())
    {
        _Intake.get_master_collecting_angle().set(-0.35);
    }
    else
      {
        _Intake.get_master_collecting_angle().set(0);
      
        if(_arm.get_arm_motor().getSelectedSensorPosition()<10000)
        {
          _arm.get_arm_motor().set(-0.3);
          if(!_Intake.get_bottom_limit_Switch())
          {
            _arm.close_piston();
          }
        }
      else
      {
        _arm.get_arm_motor().set(-0.65);
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_arm.get_bot_limit())
    {
      return true;
    }
    return false;
  }
}
