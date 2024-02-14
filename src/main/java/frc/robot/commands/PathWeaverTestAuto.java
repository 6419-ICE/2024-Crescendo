package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        Translation2d firePos = new Translation2d(3.5,7.5);
        addCommands(
            Commands.parallel(
                new TrajectoryCommand(m_DriveSubsystem, TrajectoryPaths.getPathWeaverTrajectory("FireAndNote")), 
                //this sequence fires the note & starts the intake when it hits the correct position (firePose)
                Commands.sequence(
                    new WaitUntilCommand(()->Util.isNearTranslation(firePos, m_DriveSubsystem.getPose().getTranslation(),0.1)),
                    new PrintCommand("fire"),
                    new PrintCommand("intake").withTimeout(2)
                )
            ),
            //fire at the end
            new PrintCommand("fire")
        );
    }
}
