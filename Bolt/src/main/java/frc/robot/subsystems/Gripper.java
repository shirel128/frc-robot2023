
package frc.robot.subsystems;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
    private final DoubleSolenoid _leftGripperSolenoid;
    DigitalInput _beam;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    PistonedTelescopicArm _arm;
    Boolean _hasItem;

    public Gripper(PistonedTelescopicArm arm) {
      _hasItem = false;
      _arm = arm;
      _leftGripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.Gripper.Ports.KFORWARD_LEFT,Constants.Gripper.Ports.KBACKWARDS_LEFT);
      m_colorSensor = new ColorSensorV3(I2C.Port.kMXP);
      m_colorMatcher = new ColorMatch();
      m_colorMatcher.addColorMatch(Constants.Gripper.CONE_COLOR);
      m_colorMatcher.addColorMatch(Constants.Gripper.CUBE_COLOR);
    }

    public void close(){
      SmartDashboard.putBoolean("config", false);
      _leftGripperSolenoid.set(Value.kForward);
    }

    public void open(){
      _arm.get_arm_motor().set(0.05); 
      SmartDashboard.putBoolean("config", true);
      _leftGripperSolenoid.set(Value.kReverse);
    }

    public boolean hasItem(){
      ColorMatchResult res = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
      if((res.color == Constants.Gripper.CUBE_COLOR || res.color == Constants.Gripper.CONE_COLOR)){
          _hasItem = true;
      }
      return  _hasItem;
    }

    public void open_defualt()
    {
      _leftGripperSolenoid.set(Value.kReverse);
    }
    
    @Override
    public void periodic() {
      ColorMatchResult res = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
      SmartDashboard.putString("proximity",m_colorSensor.getColor().toString());
      SmartDashboard.putNumber("color",res.confidence);
      SmartDashboard.putBoolean("HAS ITEM",_hasItem);
    }
  
}


