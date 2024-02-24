package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.WristProfiledPIDStateCommand.Position;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class ArmTestAuto extends SequentialCommandGroup {
    public ArmTestAuto(WristProfiledPIDSubsystem m_wrist) {
        addCommands(
            new WristProfiledPIDStateCommand(m_wrist,WristProfiledPIDStateCommand.Position.amp)
        );
    }
    
}
