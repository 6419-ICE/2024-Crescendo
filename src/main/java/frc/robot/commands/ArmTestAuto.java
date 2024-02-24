package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.WristProfiledPIDStateCommand.Position;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class ArmTestAuto extends SequentialCommandGroup {
    public ArmTestAuto(WristProfiledPIDSubsystem m_wrist) {
        addCommands(
            //new MusicCommand(Filesystem.getDeployDirectory().toPath().resolve("music/frozen.chrp").toString(), new TalonFX(Constants.IntakeConstants.wristMotorID)),
            Commands.parallel(
                Commands.sequence(
                    new WristProfiledPIDStateCommand(m_wrist, Position.inside).until(m_wrist::atGoal),
                    new WaitCommand(2),
                    new WristProfiledPIDStateCommand(m_wrist, Position.amp).until(m_wrist::atGoal),
                    new WaitCommand(2),
                    new WristProfiledPIDStateCommand(m_wrist, Position.intake).until(m_wrist::atGoal),
                    new WaitCommand(2),
                    new WristProfiledPIDStateCommand(m_wrist, Position.load).until(m_wrist::atGoal)
                ).repeatedly(),
                new RunCommand(()->SmartDashboard.putNumber("position",m_wrist.getMeasurement())),
                new RunCommand(()->SmartDashboard.putNumber("goal",m_wrist.getGoal())),
                new RunCommand(()->SmartDashboard.putBoolean("atGoal",m_wrist.atGoal()))
            )
        );
    }
    
}
