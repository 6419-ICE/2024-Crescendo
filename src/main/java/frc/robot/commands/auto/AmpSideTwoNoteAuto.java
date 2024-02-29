package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPaths;
import frc.robot.Util;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class AmpSideTwoNoteAuto extends SequentialCommandGroup {
    public AmpSideTwoNoteAuto(DriveSubsystem m_drivetrain,WristProfiledPIDSubsystem m_wrist,ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake, VerticalAimerProfiledPIDSubsystem m_aim, LauncherSubsystem m_launch) {
        Translation2d firePos = new Translation2d(3.5,7.5);
        addCommands(
            Commands.parallel(
                new TrajectoryCommand(m_drivetrain, TrajectoryPaths.getPathWeaverTrajectory("FireAndNote")), 
                //this sequence fires the note & starts the intake when it hits the correct position (firePose)
                Commands.sequence(
                    new WaitUntilCommand(()->Util.isNearTranslation(firePos, m_drivetrain.getPose().getTranslation(),0.1)),
                    RobotContainer.fireLauncher(m_wrist, m_arm, m_intake, m_aim, m_launch),
                    new PrintCommand("intake").withTimeout(2)
                )
            ),
            //fire at the end
            new PrintCommand("fire")
        );
    }
}
