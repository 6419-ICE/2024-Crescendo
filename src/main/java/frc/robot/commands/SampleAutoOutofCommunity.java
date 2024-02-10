package frc.robot.commands;

import org.opencv.video.TrackerDaSiamRPN_Params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.IntakeStateCommand.State;

public class SampleAutoOutofCommunity extends SequentialCommandGroup{

    public SampleAutoOutofCommunity(DriveSubsystem driveSubsystem, LauncherSubsystem m_launcher, IntakeSubsystem m_intake) {
  
        addCommands(
          Commands.sequence(

          Commands.parallel(
            new TrajectoryCommand(driveSubsystem, null),
            Commands.sequence(
            new WaitCommand(50).until(()->driveSubsystem.getPose().equals(new Pose2d())),
            new FireLauncherCommand(m_launcher).withTimeout(2)
              )
            ),

            Commands.sequence(
              new IntakeStateCommand(m_intake, State.intake).withTimeout(2),
              new TrajectoryCommand(driveSubsystem, null),
              new FireLauncherCommand(m_launcher).withTimeout(2)
            )

          )
        );
        
        /* 
          Commands.sequence(
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.turnToSpeaker()),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.straightToNote()),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.backToSpeaker()),
            new WaitCommand(5)
           // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardTowardsSecondBlock()),
           // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardBackFromSecondBlock()),
           // new TurnToAngleProfiled(0, driveSubsystem).withTimeout(3)  
            )

  


        );
        */
  }
}





