// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;

public class open_arm_high extends CommandBase {
  PistonedTelescopicArm _arm; 
  Intake _intake;
  Boolean isZero;
  Boolean can_go_down1;

  public open_arm_high(PistonedTelescopicArm arm,Intake intake) {
    _arm=arm; 
    _intake=intake;
    isZero = true;
    can_go_down1=true;
    addRequirements(_arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isZero = true;
    can_go_down1=true;
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (can_go_down1)
    {
      if (!_intake.get_bottom_limit_Switch())
      {
        _intake.get_master_collecting_angle().set(-0.5);
      }
    }
    if ((_intake.get_bottom_limit_Switch()))
    {
      can_go_down1 = false;
    }
    if (!can_go_down1)
    {
      if (isZero)
      {
        _intake.get_master_collecting_angle().set(0);
        isZero = false;
      }
    _arm.open();
    if(_arm.get_pot__position()<2.15)
    {
      
      if(_arm.get_arm_motor().getSelectedSensorPosition()<30000)
      {
        _arm.get_arm_motor().set(0.7);
      }
      else{
        if(!_arm.get_top_limitswith())
        {
          _arm.get_arm_motor().set(0.3);
        }
        else{
          _arm.get_arm_motor().set(0.0);
        }
        
      }
      if(_intake.get_master_collecting_angle().getSelectedSensorPosition()<20000)
        {
          _intake.get_master_collecting_angle().set(ControlMode.PercentOutput,0.3);
        }
        else
        {
          _intake.get_master_collecting_angle().set(ControlMode.PercentOutput,0.06);
        }
    }
  }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    _intake.stop_angle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
