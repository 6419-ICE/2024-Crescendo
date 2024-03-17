package frc.robot.commands.auto.AmpSideFar;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.FireLauncherCommand;
import frc.robot.commands.IntakeStateCommand;
import frc.robot.commands.MoveArmAndWristCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class BlueAmpSideFar extends SequentialCommandGroup {
    public BlueAmpSideFar(DriveSubsystem m_drive, WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm, IntakeSubsystem m_intake, LauncherSubsystem m_launch) {
        addCommands(
            new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.load),
            Commands.race(
                new FireLauncherCommand(m_launch),
                new WaitUntilCommand(()->m_launch.getAverageVelocity() < -35).andThen(new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.outtake).withTimeout(1))
            ),
            Commands.parallel(
                new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("RedAmpSideGoFar")),
                Commands.sequence(
                    new WaitUntilCommand(()->m_drive.getPose().getX() <= 12), //bring out intake after crossing wing line 11.2
                    new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.intake),
                    new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.intake).withTimeout(1.5)
                )
            ),
            new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.load),
            Commands.race(
                Commands.sequence(
                    new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("RedAmpSideToFire1")),
                    new WaitUntilCommand(()->m_launch.getAverageVelocity() <= -50),
                    new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.outtake).withTimeout(1)
                ),
                new FireLauncherCommand(m_launch)
            )//,
            // Commands.deadline( //cancels other commands when trajectory is done and note is fired
            //     Commands.sequence(
            //         new TrajectoryCommand(m_drive, TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(8.9,7.45,Rotation2d.fromDegrees(0)),firePose), TrajectoryPaths.config)),
            //         new WaitUntilCommand(()->m_launch.getAverageVelocity() <= -50),
            //         new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.outtake).withTimeout(1)
            //     ),
            //     new FireLauncherCommand(m_launch),
            //     Commands.sequence(
            //         new IntakeStateCommand(m_intake, false,IntakeStateCommand.State.intake).withTimeout(0.25),
            //         new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.load)
            //     )
            // )
        );
    }
    
}
