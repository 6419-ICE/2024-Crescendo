package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class DriveFortyAndShoot extends SequentialCommandGroup {
    public DriveFortyAndShoot(DriveSubsystem m_drivetrain,WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake, VerticalAimerProfiledPIDSubsystem m_aim,LauncherSubsystem m_launch) {
        addCommands(
            new TrajectoryCommand(m_drivetrain, TrajectoryPaths.moveForward(Units.inchesToMeters(40))),
            RobotContainer.fireLauncher(m_wrist, m_arm, m_intake, m_aim, m_launch),
            new TrajectoryCommand(m_drivetrain, TrajectoryPaths.moveForward(2))
        );
    }
    
}
