package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutonUtils {
    
    public static PPSwerveControllerCommand getFollowingPathSwerveCommand(PathPlannerTrajectory path, Swerve _swerve){
        return new PPSwerveControllerCommand(
          path, 
            _swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            _swerve::setModuleStates, 
            true,
            _swerve// Module states consumer
        );
      }

      public static FollowPathWithEvents followPathWithCommandsUntilPathEnds(PathPlannerTrajectory path,Command theCommand ,Swerve _Swerve,Supplier<Object> commandName){
        Map<Object, Command> commandsMap = new HashMap<>();
        commandsMap.put("Open Intake", theCommand);
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("Open Intake", theCommand);
        FollowPathWithEvents pathFollowingCommand = new FollowPathWithEvents(
          getFollowingPathSwerveCommand(path, _Swerve),
        path.getMarkers(),
        eventMap);
        
        return pathFollowingCommand;
      }

      
}
