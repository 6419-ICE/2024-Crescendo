package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;

public class SampleAutoOutofCommunity extends SequentialCommandGroup{

    public SampleAutoOutofCommunity(DriveSubsystem driveSubsystem) {
  
        addCommands(
          
          Commands.sequence(
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TrialOne()),
            new WaitCommand(2),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TrialTwo())

           // new TurnToAngleProfiled(-179.999, driveSubsystem).withTimeout(3), 
           // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardTowardsSecondBlock()),
           // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardBackFromSecondBlock()),
           // new TurnToAngleProfiled(0, driveSubsystem).withTimeout(3)
            
            )
        );
     
}
}
