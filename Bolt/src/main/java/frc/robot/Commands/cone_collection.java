// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class cone_collection extends CommandBase {
  /** Creates a new open_collection. */
  Intake _Intake;
  DigitalInput beam;
  Gripper _gripper;
  double minAngle;
  double maxAngle;

  public cone_collection(Intake intake, Gripper gripper) {
    _Intake=intake;  
    _gripper = gripper;
     minAngle = 18000;
     maxAngle = 20000;
    addRequirements(_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(!_Intake.get_bottom_limit_Switch())
    {
        _Intake.get_master_collecting_angle().set(-0.4);
    }
    else
    {
      _Intake.get_master_collecting_angle().set(0);
      _Intake.get_collecting().set(-0.75);
      _Intake.get_indexer().set(ControlMode.PercentOutput, 0.6);
    }
    // _Intake.get_indexer().set(ControlMode.PercentOutput,0.4);
    // if (_Intake.get_master_collecting_angle().getSelectedSensorPosition()> minAngle  && _Intake.get_master_collecting_angle().getSelectedSensorPosition()< maxAngle)
    // {
    //   _Intake.get_master_collecting_angle().set(0.055);

    // }
    // else if (_Intake.get_master_collecting_angle().getSelectedSensorPosition()< minAngle)
    // {
    //   _Intake.get_master_collecting_angle().set(0.15);
    // }
    // else if (_Intake.get_master_collecting_angle().getSelectedSensorPosition()> maxAngle)
    // {
    //   _Intake.get_master_collecting_angle().set(-0.15);
    // }
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
