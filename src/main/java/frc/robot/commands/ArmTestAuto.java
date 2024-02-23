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
        WristProfiledPIDStateCommand command = new WristProfiledPIDStateCommand(m_wrist, Position.amp);
        addCommands(
            Commands.parallel(
                command,
                new RunCommand(()->SmartDashboard.putNumber("Target", command.getTarget().getPos())),
                new RunCommand(()->SmartDashboard.putNumber("Position", command.getPosition())),
                new RunCommand(()->SmartDashboard.putBoolean("At Goal", command.atGoal())),
                Commands.sequence(
                    new InstantCommand(()->command.setTarget(Position.intake)),
                    new WaitCommand(5).raceWith(new PrintCommand("waiting").repeatedly()),
                    new InstantCommand(()->command.setTarget(Position.amp))
                )
            )
        );
    }
    
}
