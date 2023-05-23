package frc.robot.Commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ImageProccesingCommand extends SequentialCommandGroup {
    Swerve _swerve;
    Supplier<Pose2d> _currentPoseSupplier;
    Supplier<Pose2d> _currentTagPoseSupplier;
    
  public ImageProccesingCommand(
    Swerve swerve,
    Supplier<Pose2d> currentPoseSupplier,
    Supplier<Pose2d> currentTagPoseSupplier
    ) {
    this._swerve = swerve;
    this._currentPoseSupplier = currentPoseSupplier;
    this._currentTagPoseSupplier = currentTagPoseSupplier;
    
    
    // An example trajectory to follow.  All units in meters.
    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(1, 1), 
    new PathPoint(new Translation2d(_currentPoseSupplier.get().getX(),_currentPoseSupplier.get().getY()), _currentPoseSupplier.get().getRotation()), // position, heading
    new PathPoint(new Translation2d(_currentTagPoseSupplier.get().getX() + 1,_currentTagPoseSupplier.get().getY()), _currentTagPoseSupplier.get().getRotation()) // position, heading
    );
    
    addCommands(
        new InstantCommand(() -> _swerve.resetOdometry(traj1.getInitialPose())),
        new PPSwerveControllerCommand(
            traj1, 
            this._swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this._swerve::setModuleStates, 
            true,
            this._swerve// Module states consumer
        ));
  }
}
