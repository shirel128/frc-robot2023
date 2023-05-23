package frc.robot.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private boolean isAuto;
    ProfiledPIDController thetaController;
    double maxHigh;
    double minHigh;
    boolean forwardFlag;
    int gyro_angle;
    double forword_vel;
    double last;
    double maxSub;

    public TeleopSwerve(Swerve s_Swerve,DoubleSupplier translationSup,DoubleSupplier strafeSup,DoubleSupplier rotationSup,BooleanSupplier robotCentricSup, boolean isauto) 
    {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);
      this.translationSup = translationSup;
      this.strafeSup = strafeSup;
      this.rotationSup = rotationSup;
      this.robotCentricSup = robotCentricSup;
      this.isAuto = isauto;
      maxHigh = 5;
      minHigh = -5;
      forword_vel=-0.09;
      last = 0.1;
      maxSub = 0;
      
    }

    @Override 
    public void initialize()
    {
      forwardFlag = false;
    }

    @Override
    public void execute() {
      gyro_angle= Math.round(s_Swerve.getGyro().getRoll());
      double translationVal =
          MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband) * 1;
      double strafeVal =
          MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband) * 1;
      double rotationVal =
          MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband) * 1.0;

      if (isAuto)
      {      
        if (gyro_angle < minHigh)
        {
          forwardFlag = true;
        }
        if (forwardFlag)
        {

          if ( s_Swerve.getGyro().getRoll() > maxHigh)
          {//to us

            s_Swerve.drive(
              new Translation2d(0.072, 0).times(Constants.Swerve.maxSpeed),
              0 * Constants.Swerve.maxAngularVelocity,
              false,
              true);
              forword_vel=-0.06;
          }
          else if (gyro_angle < minHigh)
          {

            s_Swerve.drive(
              new Translation2d(forword_vel ,0).times(Constants.Swerve.maxSpeed),
              0 * Constants.Swerve.maxAngularVelocity,
              false,
              true);
              maxHigh = 6;
              minHigh = -6;

          }
          else
          {
            s_Swerve.drive(
              new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
              0.0 * Constants.Swerve.maxAngularVelocity,
              false,
              true);
              
          }
        }
        else
        {
          s_Swerve.drive(
            new Translation2d(-0.25, 0).times(Constants.Swerve.maxSpeed),
            0 * Constants.Swerve.maxAngularVelocity,
            false,
            true);
        }
    }
    else
    {
      s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
    } 
    } 

    public void finished()
    {
      s_Swerve.drive(
              new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
              0.0 * Constants.Swerve.maxAngularVelocity,
              false,
              true);
    }
}


