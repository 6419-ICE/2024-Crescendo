package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.VerticalAimerProfiledPIDStateCommand.Position;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.VerticalAimerProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;
import frc.robot.commands.VerticalAimerStateCommand;
import frc.robot.commands.IntakeStateCommand.State;

public class AutoTwoNoteSpeakerRed1 extends SequentialCommandGroup {
    public AutoTwoNoteSpeakerRed1(VerticalAimerProfiledPIDSubsystem m_aimer, DriveSubsystem m_drive, IntakeSubsystem m_intake, WristProfiledPIDSubsystem m_wrist, ArmProfiledPIDSubsystem m_arm, LauncherSubsystem m_launcher) {
        addCommands(
            Commands.sequence(
                Commands.parallel(
                    new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("Auto2NS1Red")),
                    //Goes forward to the shooting position
                    new VerticalAimerProfiledPIDStateCommand(m_aimer, Position.fire),
                    //new IntakeStateCommand(m_intake, false)
                    new FireLauncherCommand(m_launcher)
                ),

                new IntakeStateCommand(m_intake, false, State.outtake),


                new MoveArmAndWristCommand(m_arm, m_wrist, frc.robot.commands.MoveArmAndWristCommand.Position.intake),
                //Moves arm to intake position

                new IntakeStateCommand(m_intake, false, State.intake),

                new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("Auto2NS2Red"))
                

                //Commands.race(
                    //starts running intake
                  //  new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("Auto2NS2"))
                    //Intakes while robot moves to first note
                //),

                //new MoveArmAndWristCommand(m_arm, m_wrist, frc.robot.commands.MoveArmAndWristCommand.Position.inside),
                
                //new TrajectoryCommand(m_drive, TrajectoryPaths.getPathWeaverTrajectory("Auto2NS3")),
                //Moves back to originial shooting position
                //new VerticalAimerProfiledPIDStateCommand(m_aimer, Position.fire)
                //Fires second time at the shooting position
            )
        );
    } 
}
