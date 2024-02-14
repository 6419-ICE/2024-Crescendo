package frc.robot;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TrajectoryPaths {
    private static final HashMap<String,Trajectory> pathWeaverTrajectories = new HashMap<>();
    static {
        registerPathWeaverTrajectories( 
            "Test",
            "FireAndNote",
            "CenterFireAndNote"
        );
    }
    //private DriveSubsystem m_driveSubSystem; 

     // Create config for trajectory
     
  static TrajectoryConfig config = new TrajectoryConfig(
    1,
    1)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);


    static TrajectoryConfig configFast = new TrajectoryConfig(
        3.7,
        3.7)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);


static TrajectoryConfig configSlowForForwardCharge = new TrajectoryConfig(
    0.35,
    0.35)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

static TrajectoryConfig configSlightlyFastForBackCharge = new TrajectoryConfig(
    0.4,
    0.4)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    public TrajectoryPaths()
    {}
    // Start Putting new Autonomus Trajectories aqui(here), use this entry as an example. 
    public static Trajectory trajectoryExample () {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    config);

    return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutOfCommunity () {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0,0)),
        // End 2 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

        return trajectory; 
    }


//Region Drive Out Of Community 
public static Trajectory MoveOutOfCommunityOverWire () {
    configSlowForForwardCharge.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(184), 0, Rotation2d.fromDegrees(0)),
        configSlowForForwardCharge);

        return trajectory; 
    }

//region CommunityPark
public static Trajectory MoveOutOfCommunity () {
    configSlowForForwardCharge.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(164), 0, Rotation2d.fromDegrees(0)),
        configSlowForForwardCharge);

        return trajectory; 
    }


public static Trajectory BackOnCharge () {
    configSlowForForwardCharge.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(88), 0, Rotation2d.fromDegrees(180)),
        configSlowForForwardCharge);
        //93
        return trajectory; 
    }
//
    // These sets of trajectories are being used to Create the Two Cube Autonomus Program
//region TwoBlockAuto
public static Trajectory ForwardToDropArm () {
    config.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(48), 0, Rotation2d.fromDegrees(0)),
        config);

        return trajectory; 
    }


public static Trajectory GoForwardToTurn () {
    config.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(188), 0, Rotation2d.fromDegrees(0)),
        config);

        return trajectory; 
        }

public static Trajectory GoBackwardsTowardsBlock () {
    config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(28), 0, Rotation2d.fromDegrees(180)),
        config);
        
        return trajectory; 
    }

public static Trajectory GoForwardToTurnOnWayBack () {
    config.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(28), 0, Rotation2d.fromDegrees(0)),
        config);

        return trajectory; 
        }

public static Trajectory GoBackwardsToDropBlock () {
    config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(164), 0, Rotation2d.fromDegrees(180)),
        config);
        
        return trajectory; 
    }
//endregion



//region TwoBlockAutoFast

public static Trajectory GoForwardToTurnFast () {
    configFast.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(188), 0, Rotation2d.fromDegrees(0)),
        configFast);

        return trajectory; 
        }

public static Trajectory GoBackwardsTowardsBlockFast () {
    config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(28), 0, Rotation2d.fromDegrees(180)),
        config);
        
        return trajectory; 
    }

public static Trajectory GoForwardToTurnOnWayBackFast () {
    config.setReversed(false);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(28), 0, Rotation2d.fromDegrees(0)),
        config);

        return trajectory; 
        }

public static Trajectory GoBackwardsToDropBlockFast () {
    configFast.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.inchesToMeters(0),0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(164), 0, Rotation2d.fromDegrees(180)),
        configFast);
        
        return trajectory; 
    }
//endregion






    public static Trajectory trajectoryAutoForwardToPutArmDown () {
        config.setReversed(false);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(48), 0, Rotation2d.fromDegrees(0)),
            config);
    
            return trajectory; 
        }

        public static Trajectory trajectoryAutoTurn180 () {
            return new Trajectory(
                List.of(
                    new Trajectory.State(
                        0, 0, 0,
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        0),
                    new Trajectory.State(
                        3, 0, 0,
                        new Pose2d(0, 0, Rotation2d.fromDegrees(179.4)),
                        0
                    )
                )
            );
            // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            //     List.of(
            //         // Start at the origin facing the +X direction
            //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            //         // End 3 meters straight ahead of where we started, facing forward
            //         new Pose2d(0, 0, Rotation2d.fromDegrees(179.4))
            //     ),
            //     config);
        
            // return trajectory;
        }


        public static Trajectory trajectoryAutoForwardTowardsDropoff () {
            config.setReversed(true);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(Units.inchesToMeters(0),0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(Units.inchesToMeters(48), 0, Rotation2d.fromDegrees(0)),
                config);
        
                return trajectory; 
            }

            public static Trajectory TestGoingForward () {
                config.setReversed(false);
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(Units.inchesToMeters(0),0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(Units.inchesToMeters(150), 0, Rotation2d.fromDegrees(0)),
                    config);
            
                    return trajectory; 
                }
        
                public static Trajectory TestComingBack () {
                    config.setReversed(true);
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(Units.inchesToMeters(0),0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(Units.inchesToMeters(48), 0, Rotation2d.fromDegrees(180)),
                        config);
                        
                        return trajectory; 
                    }

    public static Trajectory trajectoryAutoForwardTowardsSecondBlock () {
        config.setReversed(true);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(180)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(0),0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(156), 0, Rotation2d.fromDegrees(180)),
            config);
    
            return trajectory; 
        }

        public static Trajectory trajectoryAutoForwardBackFromSecondBlock () {
            config.setReversed(false);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(Units.inchesToMeters(0),0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(Units.inchesToMeters(180), 0, Rotation2d.fromDegrees(0)),
                config);
                
                return trajectory; 
            }
    

    public static Trajectory trajectoryAutoBackTowardsDropOffOfSecondBlock () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(224), 0, Rotation2d.fromDegrees(179.4)),
            config);
    
            return trajectory; 
        }
    

    // Trajectory to get onto the charging station
    public static Trajectory trajectoryAutoEngageOnChargingStation () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.25,0)),
            // End 100 "meters" straight ahead of where we started, facing forward
            new Pose2d(0.5, 0, new Rotation2d(0)),
            config);
    
            return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutLeft() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0), 
            new Translation2d((Units.feetToMeters(22)),Units.feetToMeters(3.75)), 
            new Translation2d((Units.inchesToMeters(40)),Units.feetToMeters(3.75))
            ),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(43), Units.feetToMeters(6.00), new Rotation2d(Units.degreesToRadians(0))),
            config);
        return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutRight() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0), 
            new Translation2d((Units.feetToMeters(22)),Units.feetToMeters(-6.0)), 
            new Translation2d((Units.inchesToMeters(38)),Units.feetToMeters(-6.0))
            ),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(41), Units.feetToMeters(-6.0), new Rotation2d(Units.degreesToRadians(0))),
            config);
        return trajectory; 
    }
    
    public static Trajectory trajectoryAutoDriveOutCenter() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0)),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(43), Units.feetToMeters(6.00), new Rotation2d(Units.degreesToRadians(0))),
            config);
            return trajectory; 
    }


    public static Trajectory trajectoryMoveToCube() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 
      List.of(
    //new Translation2d(Units.inchesToMeters(100), 0),
    new Translation2d(Units.inchesToMeters(200), 0)
    //new Translation2d(Units.inchesToMeters(300), 0)
    ),
      new Pose2d(Units.inchesToMeters(224),0,new Rotation2d(0)), 
      config
      );
      
      
        return trajectory;
    }
    public static void registerPathWeaverTrajectories(String... names) {
        for (String name : names) {
            Trajectory t = new Trajectory();
            try {
                t = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathweaver/PathWeaver/output/" + name + ".wpilib.json"));
            } catch (IOException e) {
                DriverStation.reportError("Error occured while registering path \"" + name + "\"",false);
            }
            pathWeaverTrajectories.put(name,t);
        }
    }
    public static Trajectory getPathWeaverTrajectory(String name) {
        return pathWeaverTrajectories.get(name);
    }
}
