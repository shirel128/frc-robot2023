// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class PistonedTelescopicArm extends SubsystemBase {
  DoubleSolenoid _angleSolenoid;
  WPI_TalonSRX _masterArm;
  DigitalInput _toplimitSwitch = new DigitalInput(2);
  DigitalInput _bottomlimitSwitch = new DigitalInput(1);
  Boolean is_in_top =false;
  AnalogInput pot=new AnalogInput(0);


  public PistonedTelescopicArm() {
    
    _angleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.Arm.Ports.KFORWARD,Constants.Arm.Ports.KBACKWARDS);
    _masterArm = new WPI_TalonSRX (Constants.Arm.Ports.MASTERARM);
    _masterArm.setNeutralMode(NeutralMode.Brake);
    _masterArm.setInverted(true);
    _masterArm.config_kF(0, 0.1705);
    _masterArm.configMotionCruiseVelocity(6000);
    _masterArm.configMotionAcceleration(6000);
    }

    public void stop_arm()
    {
      _angleSolenoid.set(Value.kOff);
    }

    public boolean is_open()
    {
      return !_toplimitSwitch.get();
    }

    public void setPos(double pos){
      _masterArm.set(ControlMode.MotionMagic, pos);
    }

    public boolean bottom_true()
    {
      return !_bottomlimitSwitch.get();
    }
    
    public void open(){
      _angleSolenoid.set(Value.kForward);
    
    }

    public void close_piston(){
      _angleSolenoid.set(Value.kReverse);
    
    }

    public void stop_arm(double d)
    {
      _masterArm.set(d);
    }

    public boolean is_closed()
    {
      return !_bottomlimitSwitch.get();
    }

    public boolean can_go_up()
    {
      if(pot.getVoltage()<2.18)
      {
        return true;
      }
      return false;
    }

    public boolean is_down()
    {
      if(pot.getVoltage()>2.237)
      {
        return true;
      }
      return false;
    }

    public boolean get_top_limitswith()
    {
      return !_toplimitSwitch.get();
    }

    public double get_pot__position()
    {
      return pot.getVoltage();
    }

    public double get_encoder()
    {
      return _masterArm.getSelectedSensorPosition();
    }

    public WPI_TalonSRX get_arm_motor()
    {
      return _masterArm;
    }

    public boolean get_bot_limit()
    {
      return !_bottomlimitSwitch.get();
    }

    public void always_up()
    {
      if(!get_bot_limit())
      {
        _masterArm.set(-0.1);
      }

      else
      {
        _masterArm.set(-0.15);
      }
    }

    @Override
    public void periodic() {
      if(!_bottomlimitSwitch.get()){
        SmartDashboard.putBoolean("RESETING",true);
        _masterArm.setSelectedSensorPosition(0);
      }
      if(is_in_top)
      {
        _masterArm.set(0.05);
      }
      SmartDashboard.putBoolean("RESETING",false);
      SmartDashboard.putBoolean("_bottomlimitSwitch",!_bottomlimitSwitch.get());
      SmartDashboard.putBoolean("_toplimitSwitch",_toplimitSwitch.get());
      SmartDashboard.putNumber("arm position",_masterArm.getSelectedSensorPosition());
      SmartDashboard.putNumber("pott",pot.getVoltage());

      super.periodic();
    }


    private boolean canMove() {
      return !_toplimitSwitch.get() || !_bottomlimitSwitch.get();
    }

}
