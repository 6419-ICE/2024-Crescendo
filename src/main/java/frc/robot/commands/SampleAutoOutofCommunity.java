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
        //*****************************************************************************NOT CURRENTLY IN THE ROBOT CONTAINER AUTO CHOOSER************************************************************************************************************************* 
        addCommands(
          Commands.sequence(
          //This parallel command drives to the ring, and shoots at a certain pose(currently null)
          Commands.parallel(
            new TrajectoryCommand(driveSubsystem, null),
            Commands.sequence(
            new WaitCommand(50).until(()->driveSubsystem.getPose().equals(new Pose2d())),//The wait is basically just an either/or - either it reaches 50 seconds, or the pose is at the required pose
            new FireLauncherCommand(m_launcher).withTimeout(2)//Fire for two seconds
              )
            ),

            Commands.sequence(//Sequential command: Intkae, then go to the first launching position, and then launching using the fire launcher command
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





