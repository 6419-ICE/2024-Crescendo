package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.VerticalAimerStateCommand;
import frc.robot.commands.VerticalAimerStateCommand.Position;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;

public class LauncherAngleTestAuto extends SequentialCommandGroup {
    public LauncherAngleTestAuto(VerticalAimerProfiledPIDSubsystem m_aim) {
        addCommands(
            new VerticalAimerStateCommand(m_aim, Position.fire)
        );
    }
    
}
