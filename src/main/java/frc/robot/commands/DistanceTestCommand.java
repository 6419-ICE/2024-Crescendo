package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.DriveSubsystem;

public class DistanceTestCommand extends SequentialCommandGroup {
    public DistanceTestCommand(DriveSubsystem m_drivetrain) {
        addCommands(
            new TrajectoryCommand(m_drivetrain, TrajectoryPaths.trajectoryAutoDriveOutOfCommunity())
        );
        addRequirements(m_drivetrain);
    }
    
}
