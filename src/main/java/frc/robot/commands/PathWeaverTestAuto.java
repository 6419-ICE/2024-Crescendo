package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class PathWeaverTestAuto extends SequentialCommandGroup {
    public PathWeaverTestAuto(DriveSubsystem m_DriveSubsystem,LauncherSubsystem m_launcher) {
        Pose2d firePos = new Pose2d(3.5,7.5,new Rotation2d());
        Pose2d intakePos = new Pose2d(3.8,7.4,new Rotation2d());
        addCommands(
            Commands.parallel(
                new TrajectoryCommand(m_DriveSubsystem, TrajectoryPaths.getPathWeaverTrajectory("FireAndNote")),
                Commands.sequence(
                    new WaitUntilCommand(()->Util.isNearPose(firePos, m_DriveSubsystem.getPose(), 0.2,15)),
                    new PrintCommand("fire"),
                    new PrintCommand("intake").withTimeout(2)
                )
            ),
            new PrintCommand("fire")
            );
        
        
    }
    
    
}
