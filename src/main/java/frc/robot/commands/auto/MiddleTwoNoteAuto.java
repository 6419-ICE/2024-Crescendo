package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.ArmProfiledPIDStateCommand;
import frc.robot.commands.IntakeStateCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class MiddleTwoNoteAuto extends SequentialCommandGroup {
    public MiddleTwoNoteAuto(DriveSubsystem m_drivetrain,WristProfiledPIDSubsystem m_wrist,ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake,VerticalAimerProfiledPIDSubsystem m_aim, LauncherSubsystem m_launch) {
        addCommands(
            //new TrajectoryCommand(m_drivetrain, TrajectoryPaths.moveForward(0.45)),
            RobotContainer.fireLauncher(m_wrist, m_arm, m_intake, m_aim, m_launch),
            Commands.race(
                new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.intake)
               // new TrajectoryCommand(m_drivetrain, TrajectoryPaths.moveForward(1.35))
            ),
            new TrajectoryCommand(m_drivetrain, TrajectoryPaths.moveForward(-1.35)),
            RobotContainer.fireLauncher(m_wrist, m_arm, m_intake, m_aim, m_launch)
            );
    }
    
    
}
