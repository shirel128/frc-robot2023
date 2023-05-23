// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.abortCommand;
import frc.robot.Commands.close_arm;
import frc.robot.Commands.close_collection;
import frc.robot.Commands.openArmLow;
import frc.robot.Commands.openArmMid;
import frc.robot.Commands.open_arm_auto;
import frc.robot.Commands.open_arm_high;
import frc.robot.Commands.cone_collection;
import frc.robot.Commands.cube_collection;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PistonedTelescopicArm;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
   private static final double DRIVE_COEFFICIENT = 1;
   private final CommandPS4Controller _driveController;
   private final GenericHID _operatorController;
   private final Gripper _gripper;
   private final PistonedTelescopicArm _arm;
   private final Intake _intake;
   private final Swerve _swerve;
   private final close_collection _closeCollection;
   private final open_arm_high _openArmHigh;
   private final openArmLow _openArmLow;
   private final openArmMid _openArmMid;
   private final cone_collection _coneCollection;
   private final close_arm _closeArm;
   private final CommandJoystick joistick ;
   private final cube_collection _cubeCollection;
   private final abortCommand _abort_Command;

   JoystickButton button2;
   PathPlannerTrajectory traj_right;
   PathPlannerTrajectory traj_left;
   int auto_selector;
   JoystickButton button3;
   JoystickButton button4;
  
  public RobotContainer() {
    _driveController = new CommandPS4Controller(Constants.DRIVE_CONTROLLER_PORT);
    _operatorController= new GenericHID(1);
    joistick=new CommandJoystick(1);
    _arm = new PistonedTelescopicArm();
    _gripper = new Gripper(_arm);
    _intake = new Intake(_gripper);
    _swerve = new Swerve();
    _closeArm =new close_arm(_arm, _intake);
    _openArmMid = new openArmMid(_arm, _intake);
    _openArmHigh=new open_arm_high(_arm,_intake);
    _openArmLow = new openArmLow(_arm,_intake);
    _coneCollection =new cone_collection(_intake, _gripper);
    _closeCollection =new close_collection(_intake, _arm);
    _cubeCollection = new cube_collection(_intake, _gripper);
    _abort_Command=new abortCommand(_arm, _intake);
    
    
    CommandScheduler.getInstance().registerSubsystem(
      _swerve,
      _gripper
       ,_arm 
       ,_intake     
      );

    _swerve.setDefaultCommand(
      new TeleopSwerve(
          _swerve,
          () -> _driveController.getLeftY()*DRIVE_COEFFICIENT,
          () -> -_driveController.getLeftX()*DRIVE_COEFFICIENT*-1,
          () -> _driveController.getRightX()*-DRIVE_COEFFICIENT,
          () -> false,
          false));

    _gripper.setDefaultCommand(new SequentialCommandGroup( new InstantCommand(_gripper::open_defualt,_gripper)));
    
     _arm.setDefaultCommand(new InstantCommand(
      () -> {
        _arm.always_up();;
     },_arm));

     traj_right = PathPlanner.loadPath("right", 
     new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    configureBindings();

    traj_left = PathPlanner.loadPath("left", 
    new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  }


  public SequentialCommandGroup followPathLeft()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(true){
            _swerve.resetOdometry(traj_left.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        traj_left, 
          _swerve::getPose, // Pose supplier
          Constants.AutoConstants.m_kinematics, // SwerveDriveKinematics
          new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0.1, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0.1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          _swerve::setModuleStates, // Module states consumer
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          _swerve // Requires this drive subsystem
      )
    );
 }

  public SequentialCommandGroup followPathRight()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(true){
            _swerve.resetOdometry(traj_right.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        traj_right, 
          _swerve::getPose, // Pose supplier
          Constants.AutoConstants.m_kinematics, // SwerveDriveKinematics
          new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0.1, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0.1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          _swerve::setModuleStates, // Module states consumer
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          _swerve // Requires this drive subsystem
      )
  );
  }

  public SequentialCommandGroup followRight()
  {
    return new SequentialCommandGroup(
      new InstantCommand(_gripper::close,_gripper),
      new WaitCommand(0.5),
      new open_arm_high
      (_arm,_intake).until(_arm::get_top_limitswith),
      new WaitCommand(1.2),
      new InstantCommand(_gripper::open),
      new WaitCommand(0.5),
      new ParallelCommandGroup(new SequentialCommandGroup( new close_arm(_arm, _intake).until(_arm::get_bot_limit)
      ,new RunCommand(_arm::close_piston).until(_arm::is_down)),followPathRight()
      ));
  }

  public SequentialCommandGroup followLeft()
  {
    return new SequentialCommandGroup(
      new InstantCommand(_gripper::close,_gripper),
      new WaitCommand(0.5),
      new open_arm_high(_arm,_intake).until(_arm::get_top_limitswith),
      new WaitCommand(1.2),
      new InstantCommand(_gripper::open),
      new WaitCommand(0.5),
      new ParallelCommandGroup(new SequentialCommandGroup( new close_arm(_arm, _intake).until(_arm::get_bot_limit)
      ,new RunCommand(_arm::close_piston).until(_arm::is_down), new close_collection(_intake, _arm)),followPathLeft()
      ));
  }

  public Command coneAndChargeStation()
  {
    return new SequentialCommandGroup(
      
    new InstantCommand(_gripper::close,_gripper),
    new WaitCommand(0.2),
    new open_arm_high(_arm,_intake).until(_arm::get_top_limitswith),
    new WaitCommand(0.55),
    new InstantCommand(_gripper::open),
    new WaitCommand(0.2),
    new ParallelCommandGroup(new SequentialCommandGroup( new close_arm(_arm, _intake).until(_arm::get_bot_limit)
    ,new RunCommand(_arm::close_piston).until(_arm::is_down), new close_collection(_intake, _arm)),
    new TeleopSwerve(
      _swerve,
      () -> 0.2,
      () -> -_driveController.getLeftX()*DRIVE_COEFFICIENT*0,
      () -> _driveController.getRightX()*-DRIVE_COEFFICIENT*0,
      () -> false,true)));

  }

  
  public Command getAutonomusCommand()
  {

    int x = 1;


    if (x == 1) //coneAndChargeStation
    {
      return coneAndChargeStation();
    }
    else if (x == 2)//follow left
    {
      return followLeft();
    }
    else if ( x == 3 )//follow right
    {
      return followRight();
    }
    return  new SequentialCommandGroup();
  }

  private void configureBindings() {

    joistick.button(9).onTrue(new SequentialCommandGroup(new InstantCommand(_intake::stop),
        _openArmHigh));

    joistick.button(6).onTrue(
    new SequentialCommandGroup(new InstantCommand(_intake::stop),_openArmMid));

     _driveController.circle().onTrue(
    new SequentialCommandGroup(new InstantCommand(_intake::stop),_openArmLow));

    joistick.button(1).toggleOnTrue(new StartEndCommand(_gripper::close,_gripper::open,_gripper));

    _driveController.R1()
    .toggleOnTrue(new InstantCommand(() -> _gripper.open(),_gripper))
    .toggleOnFalse(new InstantCommand(_gripper::close,_gripper));

    joistick.button(2).onTrue(_coneCollection);

    joistick.button(3).onTrue(new SequentialCommandGroup(_cubeCollection.until(_intake::get_beam),
    new InstantCommand(_intake::stop)));

    joistick.button(12).onTrue(_abort_Command);

    joistick.button(4).onTrue( new SequentialCommandGroup(new close_arm(_arm, _intake).until(_arm::get_bot_limit)
    ,new RunCommand(_arm::close_piston).until(_arm::is_down), new close_collection(_intake, _arm)));

    _driveController.L1().onTrue(new InstantCommand(_swerve::zeroGyro));

  }

  private void resetModuleZeros() {
    _swerve.zeroGyro();
    _swerve.resetModuleZeros();
  }

  public Command high_goal()
  {
    return Commands.sequence(
      new RunCommand(_arm::open, _arm),
      new WaitCommand(3) ,
      new RunCommand(_gripper::close, _gripper)
    );
  }

  public void disabledActions() {
    resetModuleZeros();
  }

  
}
