// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;

public class abortCommand extends CommandBase {
  /** Creates a new abortCommand. */
  PistonedTelescopicArm _arm;
  Intake _intake;
  public abortCommand(PistonedTelescopicArm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm=arm;
    _intake=intake;
    addRequirements(_arm, _intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _intake.get_master_collecting_angle().set(0);
    _intake.get_indexer().set(ControlMode.PercentOutput,0);
    _intake.get_collecting().set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
