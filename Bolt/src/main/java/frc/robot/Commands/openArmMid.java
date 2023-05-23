// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;

public class openArmMid extends CommandBase {
  PistonedTelescopicArm _arm; 
  Intake _intake;
  Boolean isZero;
  Boolean can_go_down1;
  Boolean already_opened=true; 
  Boolean afterFirstTime;

  /** Creates a new openArmMid. */
  public openArmMid(PistonedTelescopicArm arm,Intake intake) {
    _arm=arm; 
    _intake=intake;
    isZero = true;
    can_go_down1=true;
    addRequirements(_arm,_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isZero = true;
    can_go_down1=true;
    already_opened=true; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (can_go_down1)
    {
      if (!_intake.get_bottom_limit_Switch())
      {
        _intake.get_master_collecting_angle().set(-0.35);
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
      if(already_opened)
      {
        _arm.open();
      }
    if(_arm.get_pot__position()<2.175)
    {
 
        _arm.stop_arm();

      already_opened=false;

      if(_arm.get_arm_motor().getSelectedSensorPosition()<12000)
      {
        _arm.get_arm_motor().set(0.5);
      }
      else
      {
        _arm.get_arm_motor().set(0.0);
      
      }
      if(_intake.get_master_collecting_angle().getSelectedSensorPosition()<20000)
        {
          _intake.get_master_collecting_angle().set(ControlMode.PercentOutput,0.25);

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
  public boolean isFinished() {

    return false;
  }
}
