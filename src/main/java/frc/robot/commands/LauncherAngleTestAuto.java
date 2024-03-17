package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.VerticalAimerStateCommand;
import frc.robot.commands.VerticalAimerStateCommand.Position;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

public class LauncherAngleTestAuto extends SequentialCommandGroup {
    public LauncherAngleTestAuto(VerticalAimerProfiledPIDSubsystem m_aim) {
        addCommands(
            Commands.repeatingSequence(
                 new VerticalAimerStateCommand(m_aim, Position.fireWing).once(),
                new VerticalAimerStateCommand(m_aim, Position.down).once(),
                new VerticalAimerStateCommand(m_aim, Position.fireWing).once(),
                new VerticalAimerStateCommand(m_aim, Position.load).once()
            )
        );
    }
    
}
