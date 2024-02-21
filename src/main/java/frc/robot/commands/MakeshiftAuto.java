package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class MakeshiftAuto extends SequentialCommandGroup { 
    public MakeshiftAuto(DriveSubsystem m_DriveSubsystem){
        Translation2d targetPos = new Translation2d(1,33);
        addCommands(
            Commands.parallel(
                new TrajectoryCommand(m_DriveSubsystem, TrajectoryPaths.getPathWeaverTrajectory("makeshift")),
                Commands.sequence(
                    new WaitUntilCommand(()->Util.isNearTranslation(targetPos,m_DriveSubsystem.getPose().getTranslation(),0.1),
                        new PrintCommand("makeshift")
                    )
                )
            
        );
    }
}
