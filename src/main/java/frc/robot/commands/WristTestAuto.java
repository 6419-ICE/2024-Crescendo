package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.WristStateCommand.Position;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class WristTestAuto extends SequentialCommandGroup {
    public WristTestAuto(WristProfiledPIDSubsystem m_wrist) {
        addCommands(
            //new MusicCommand(Filesystem.getDeployDirectory().toPath().resolve("music/frozen.chrp").toString(), new TalonFX(Constants.IntakeConstants.wristMotorID)),
            Commands.parallel(
                Commands.sequence(
                    new WristStateCommand(m_wrist, WristStateCommand.Position.intake).withTimeout(2),
                    new WaitCommand(2),
                    //new WristStateCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.inside),
                    new WristStateCommand(m_wrist, WristStateCommand.Position.inside).withTimeout(2),
                    new WaitCommand(2),
                    //new WristStateCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.inside),
                    new WristStateCommand(m_wrist, WristStateCommand.Position.load).withTimeout(2)

                ),
                new RunCommand(()->SmartDashboard.putNumber("position",m_wrist.getMeasurement())),
                new RunCommand(()->SmartDashboard.putNumber("goal",m_wrist.getGoal())),
                new RunCommand(()->SmartDashboard.putNumber("tolerance", m_wrist.getController().getPositionTolerance())),
                new RunCommand(()->SmartDashboard.putBoolean("atGoal",m_wrist.atGoal()))
            )

        );
        addRequirements(m_wrist);
    }
    
}
