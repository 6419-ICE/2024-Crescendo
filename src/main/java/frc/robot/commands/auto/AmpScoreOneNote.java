package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.IntakeStateCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class AmpScoreOneNote extends SequentialCommandGroup {     
    public AmpScoreOneNote(DriveSubsystem m_drive,WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm,IntakeSubsystem m_intake,VerticalAimerProfiledPIDSubsystem m_aim) {
        addCommands(
            new TrajectoryCommand(m_drive,TrajectoryPaths.getPathWeaverTrajectory("AmpScore")),
            RobotContainer.toAmp(m_wrist, m_arm, m_intake, m_aim),
            new IntakeStateCommand(m_intake,false,IntakeStateCommand.State.outtake).withTimeout(3),
            RobotContainer.stowArm(m_wrist, m_arm, m_intake)
        );
    }
}
