package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.WristProfiledPIDSubsystemThroughBore;

public class EncoderTest extends ParallelCommandGroup {
    public EncoderTest(WristProfiledPIDSubsystemThroughBore m_wrist) {
        addCommands(
            Commands.sequence(
                new InstantCommand(m_wrist::enable),
                new InstantCommand(()->m_wrist.setGoal(WristProfiledPIDSubsystemThroughBore.inside))
            ),
            new RunCommand(()->SmartDashboard.putNumber("pos", m_wrist.getMeasurement())),
            new RunCommand(()->SmartDashboard.putNumber("goal", m_wrist.getGoalPosition())),
            new RunCommand(()->SmartDashboard.putBoolean("atGoal", m_wrist.atGoal()))
        );
    }
    
}
