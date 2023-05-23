package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    WPI_VictorSPX _collecting ;
    TalonSRX _indexer;
    WPI_TalonFX _master_collecting_angle;
    WPI_TalonFX _slave_collecting_angle;
    DigitalInput _bottom_limit_Switch;
    DigitalInput _top_limit_Switch;
    DigitalInput beam;
    Gripper _gripper;
    DigitalInput _lowerLimit;

    /** Creates a new Intake. */
    public Intake(Gripper gripper) 
    {
        _gripper = gripper;
        _indexer = new TalonSRX(Constants.Intake.Ports.INDEXER_PORT);
        _collecting =new WPI_VictorSPX(Constants.Intake.Ports.INTAKE_MOTOR);
        _slave_collecting_angle =new WPI_TalonFX(Constants.Intake.Ports.ANGLE_SLAVE_PORT);
        _master_collecting_angle =new WPI_TalonFX(Constants.Intake.Ports.ANGLE_MASTER_PORT);
        _bottom_limit_Switch = new DigitalInput(6);
        _top_limit_Switch = new DigitalInput(7);
        beam=new DigitalInput(0);
        _slave_collecting_angle.follow(_master_collecting_angle);
        _master_collecting_angle.setInverted(true);

    }

    public boolean is_up()
    {
        if(_master_collecting_angle.getSelectedSensorPosition()>60000)
        {
            return true;
        }
        return false;

    }

    public boolean get_beam()
    {
        return !beam.get();
    }
    
    public void stop_angle()
    {

        _master_collecting_angle.set(0);

    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putBoolean("botton limit switch collection",_bottom_limit_Switch.get());
        SmartDashboard.putBoolean("top limit switch collection",(!_top_limit_Switch.get()));
        SmartDashboard.putBoolean("beam collection",get_beam());
        SmartDashboard.putNumber("encoder switch collection",_master_collecting_angle.getSelectedSensorPosition());

        if(_bottom_limit_Switch.get())
        {
            _master_collecting_angle.setSelectedSensorPosition(0);
        }

        if(get_top_limit_Switch())
        {
            _master_collecting_angle.setSelectedSensorPosition(73000);
        }

        if( _master_collecting_angle.getSelectedSensorPosition()>65000|| get_top_limit_Switch())
        {
            _master_collecting_angle.set(0);
        }
        super.periodic();
    }

    public TalonSRX get_indexer(){
        return _indexer;
    }

    public WPI_VictorSPX get_collecting(){
        return _collecting;
    }

    public WPI_TalonFX get_master_collecting_angle(){
        return _master_collecting_angle;
    }  

    public boolean get_bottom_limit_Switch(){
        return _bottom_limit_Switch.get();
    } 
    
    public boolean get_top_limit_Switch(){
        return !_top_limit_Switch.get();
    }  

    public void stop()
    {
        _indexer.set(ControlMode.PercentOutput,0);
        _collecting.set(0);
        _gripper.open();
    }

    public void collect_game_pice(double collecting_velocity,double indexer_vel)
    {
        
         _collecting.set(collecting_velocity);

         _indexer.set(ControlMode.PercentOutput, indexer_vel);       
    }
}
