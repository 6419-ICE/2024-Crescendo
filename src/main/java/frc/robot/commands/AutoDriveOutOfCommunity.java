// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveOutOfCommunity extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoDriveOutOfCommunity(DriveSubsystem driveSubsystem) {
    // driveSubsystem.zeroHeading();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.sequence(
            // new WaitCommand(.5),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TestGoingForward()),
            new WaitCommand(2),
            new TurnToAngleProfiled(-179.999, driveSubsystem).withTimeout(3),
            new WaitCommand(2),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TestComingBack()),
            new WaitCommand(2),
            new TurnToAngleProfiled(-179.999, driveSubsystem).withTimeout(3),
            new WaitCommand(2),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TestGoingForward()),
            new WaitCommand(2),
            new TurnToAngleProfiled(-179.999, driveSubsystem).withTimeout(3),
            new WaitCommand(2),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.TestComingBack())));

  }
}
