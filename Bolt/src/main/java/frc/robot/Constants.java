package frc.robot;


import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0;
    public static final String kJoystickPort = null;

    public static class Intake{

        public static class Ports{
            public static final int ANGLE_SLAVE_PORT = 17;
            public static final int INTAKE_MOTOR = 18;

            public static final int ANGLE_MASTER_PORT = 3;
            public static final int INDEXER_PORT = 14;

        }

        public static final double INTAKE_POSITION = 0;

        public static final double INTAKE_IN_POSITION = 0;
        public static final double ANGLE_START_OFFSET = 0;

        public static final double NO_INTAKE_SPEED = 0;
        public static final double LOWER_LIMIT_OFFSET = 0;


        public static final double INTAKE_UPPER_SPEED = 0;
        
        // Indexer
        public static final double INDEXER_INTAKING_SPEED = 0.7;
        public static final double INDEXER_NO_INTAKING_SPEED = 0;


    }

    public static class Arm{

        public static class Ports{
        //public static final int SLAVEARM = 0;
        public static final int MASTERARM = 7;
        public static final int KFORWARD = 6;
        public static final int KBACKWARDS = 7;
        }
        

        public static final int HIGHT_SETPOINT = 0;
        public static final int CLOSED_SETPOINT = 0;
        public static final double ARM_HIGH_THRESHOLD = 0;
        public static final double ARM_LOW_THRESHOLD = 0;
        public static final double ARM_START_OFFSET = 0;

        public static final double TELESCOPIC_KP = 0;
        public static final double TELESCOPIC_CRUISE_VELOCITY = 0;
        public static final double TELESCOPIC_MAX_ACCELERATION = 0;
        public static final double TELESCOPIC_KF = 0;
        public static final double TIMER = 0;

    }

    public static class Gripper{

        public static class Ports{
            public static final int KFORWARD_LEFT = 2;
            public static final int KBACKWARDS_LEFT = 3;

            public static final int KBACKWARDS_RIGHT = 2;
            public static final int KFORWARD_RIGHT = 3;
        }
        

        private static final int KCUBE_RED = 0;
        private static final int KCUBE_GREEN = 0;
        private static final int KCUBE_BLUE = 0;

        private static final int KCONE_BLUE = 0;
        private static final int KCONE_GREEN = 0;
        private static final int KCONE_RED = 0;

        public static final Color CUBE_COLOR = new Color(KCUBE_RED, KCUBE_GREEN,KCUBE_BLUE);
        

        public static final Color CONE_COLOR = new Color(KCONE_RED, KCONE_GREEN,KCONE_BLUE); 
        

    }
    
    
  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    // public static final int pigeonID = 6;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 60;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        ((wheelDiameter * Math.PI) / driveGearRatio);
    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor =
        ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
    public static final double angleConversionFactor = 360.0 / 12.8;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3.6; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        //supposed to be 19 at drive now its 4 
      public static final int driveMotorID =19 ;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 20;
      public static final double angleOffset =41.748+180;
      public static final boolean isDriverEncoderInverted = true;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 21;
      public static final double angleOffset = 280.107+180;
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;

      public static final int canCoderID = 22;
      public static final double angleOffset =137.548+180;
    //   public static final double / =0;
      public static final boolean isDriverEncoderInverted = true;

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 23;
      public static final double angleOffset = 4.744+180;
      public static final boolean isDriverEncoderInverted = false;

      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    
  }

  public static final class AutoConstants {
    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;
    public static final Translation2d m_frontLeftLocation = new Translation2d(0.375, 0.375);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.375, -0.375);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.375, 0.381);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.375, -0.375);
    public static final SwerveDriveKinematics m_kinematics = 
    new SwerveDriveKinematics(
     m_frontLeftLocation,
     m_frontRightLocation, 
     m_backLeftLocation, 
     m_backRightLocation);
    public static final PathConstraints constraints =
        new PathConstraints(maxVelocity, maxAcceleration);
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kMaxSpeedMetersPerSecond = 2;    
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
  }
  public static class VisionConstants{

    public static final String cameraName = "";
    public static final Transform3d robotToCam = new Transform3d();
    }
}
