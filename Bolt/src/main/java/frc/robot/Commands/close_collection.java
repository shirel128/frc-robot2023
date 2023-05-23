// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;

public class close_collection extends CommandBase {
  /** Creates a new open_collection. */
  Intake _intake;
  boolean finished=false;
  PistonedTelescopicArm _arm;
  Boolean isZero;
  Boolean can_go_down1;
  boolean canCloseCollection;
  public close_collection(Intake intake, PistonedTelescopicArm arm) {
    _intake=intake; 
    _arm=arm;
    isZero = true;
    can_go_down1=true;
    canCloseCollection = false;
    addRequirements(_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isZero = true;
    can_go_down1=true;
    _intake.get_indexer().set(ControlMode.PercentOutput,0);
    _intake.get_collecting().set(0);
    canCloseCollection = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
      // if(_intake.get_beam())
      // {
      //   _intake.get_indexer().set(ControlMode.PercentOutput,0);
      // // _gripper.open();
      //   //finished = true;

      // }
      // else
      // {
      //   _intake.get_indexer().set(ControlMode.PercentOutput,0.6);
      // }
        if (!_intake.get_top_limit_Switch() && _intake.get_master_collecting_angle().getSelectedSensorPosition()<62000)
        {
          _intake.get_master_collecting_angle().set(0.5);
    
        }
        else if (!_intake.get_top_limit_Switch() && _intake.get_master_collecting_angle().getSelectedSensorPosition()>62000)  
        {
          _intake.get_master_collecting_angle().set(0.15);

        } 
        else if( _intake.get_master_collecting_angle().getSelectedSensorPosition()>65000)
        {
          _intake.get_master_collecting_angle().set(0);
        }
        else
        {
          _intake.get_master_collecting_angle().set(0);
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

// if (can_go_down1 )
// {
//   if (!_intake.get_bottom_limit_Switch())
//   {
//     _intake.get_master_collecting_angle().set(-0.4);
//   }
// }
// if ((_intake.get_bottom_limit_Switch()))
// {
//   System.out.println("false");
  
//   can_go_down1 = false;
// }
// if (!can_go_down1)
// {
//   if (isZero)
//   {
//     _intake.get_master_collecting_angle().set(0);
//     System.out.println("is sero");
//     isZero = false;
//   }
//   if(!canCloseCollection)
//   {
//     if (_arm.get_arm_motor().getSelectedSensorPosition()<10000)
//     {
//       _arm.get_arm_motor().set(-0.15);
//       System.out.println("trying to go down");
//       if (_arm.get_bot_limit())
//       {
//         System.out.println("trying to go down2");

//         _arm.close_piston();
//         canCloseCollection = true;
//       }
//     }
//     else{
//       _arm.get_arm_motor().set(-0.7);
//     }
//   }


//   if(canCloseCollection)
//   {

    
//       System.out.println("trying to go down3");


//       if(_arm.get_pot__position()>2.225)
//       {
//         //if (_intake.get_master_collecting_angle().getSelectedSensorPosition() < 58000 && !_intake.get_top_limit_Switch())//-------here
//         System.out.println("trying to go down4");

//         if (!_intake.get_top_limit_Switch() && _intake.get_master_collecting_angle().getSelectedSensorPosition()<65000)
//         {
//           _intake.get_master_collecting_angle().set(0.2);
    
//         }
//         else if (!_intake.get_top_limit_Switch() && _intake.get_master_collecting_angle().getSelectedSensorPosition()>65000)  
//         {
//           _intake.get_master_collecting_angle().set(0.1);

//         } 
//         else
//         {
//           _intake.get_master_collecting_angle().set(0);
//         }
//     }
//   }
// }