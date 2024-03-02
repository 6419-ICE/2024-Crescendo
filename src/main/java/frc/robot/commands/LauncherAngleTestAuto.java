package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.VerticalAimerStateCommand;
import frc.robot.commands.VerticalAimerStateCommand.Position;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;


public class LauncherAngleTestAuto extends SequentialCommandGroup {
    public LauncherAngleTestAuto(VerticalAimerProfiledPIDSubsystem m_aim) {
            Commands.sequence(
            new VerticalAimerStateCommand(m_aim, Position.fire)
        );
    }
    
}
