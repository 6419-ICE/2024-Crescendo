package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.WristProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystemThroughBore;

public class EncoderTest extends SequentialCommandGroup {
    public EncoderTest(WristProfiledPIDSubsystemThroughBore m_wrist,WristProfiledPIDSubsystem m_normWrist) {
        addCommands(
            Commands.parallel(
                Commands.sequence(
                    new InstantCommand(()->m_wrist.setGoal(205)),
                    new InstantCommand(m_wrist::enable)
                ),
                new RunCommand(()->SmartDashboard.putBoolean("At goal", m_wrist.atGoal())),
                new RunCommand(()->SmartDashboard.putBoolean("Controller at goal",m_wrist.getController().atGoal())),
                new RunCommand(()->SmartDashboard.putNumber("Position", m_wrist.getMeasurement())),
                new RunCommand(()->SmartDashboard.putNumber("Goal", m_wrist.getGoal())),
                new RunCommand(()->SmartDashboard.putNumber("Output", m_wrist.getController().calculate(m_wrist.getMeasurement()))),
                new RunCommand(()->SmartDashboard.putBoolean("At goal norm", m_normWrist.atGoal())),
                new RunCommand(()->SmartDashboard.putBoolean("Controller at goal norm",m_normWrist.getController().atGoal())),
                new RunCommand(()->SmartDashboard.putNumber("Position norm", m_normWrist.getMeasurement())),
                new RunCommand(()->SmartDashboard.putNumber("Goal norm", m_normWrist.getGoal())),
                new RunCommand(()->SmartDashboard.putNumber("Output norm", m_normWrist.getController().calculate(m_normWrist.getMeasurement())))
            )
        );
        addRequirements(m_wrist);
    }
    
}
