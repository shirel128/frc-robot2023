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

public class cube_collection extends CommandBase {
  /** Creates a new open_collection. */
  Intake _Intake;
  Gripper _gripper;
  double minAngle;
  double maxAngle;
  boolean finished;
  public cube_collection(Intake intake, Gripper gripper) {
    _Intake=intake; 
    _gripper = gripper;
    minAngle = 16000;
    maxAngle = 18000;
    finished = false;
   
    addRequirements(_Intake,gripper);
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
      if(_Intake.get_master_collecting_angle().getSelectedSensorPosition()>1500) 
      {
        _Intake.get_master_collecting_angle().set(ControlMode.PercentOutput,-0.4);
        //System.out.println("giving -0.4");
      }
      else if(_Intake.get_master_collecting_angle().getSelectedSensorPosition()<300)
      {
        _Intake.get_master_collecting_angle().set(ControlMode.PercentOutput,0);
        //System.out.println("giving 0");
      }
      else
      {
        _Intake.get_master_collecting_angle().set(ControlMode.PercentOutput,-0.1);
        //System.out.println("giving -0.1");
      }
    }
    else
    {
      _Intake.get_master_collecting_angle().set(ControlMode.PercentOutput,0);
      if(_Intake.get_beam())
      {
        _Intake.get_indexer().set(ControlMode.PercentOutput,0);

      }
      else
      {
        _Intake.get_indexer().set(ControlMode.PercentOutput,0.6);
      }
      _Intake.get_collecting().set(-0.5);
    }
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
