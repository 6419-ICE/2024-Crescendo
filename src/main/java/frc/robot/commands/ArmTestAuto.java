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
import frc.robot.commands.MoveArmAndWristCommand.Position;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

public class ArmTestAuto extends SequentialCommandGroup {
    public ArmTestAuto(WristProfiledPIDSubsystem m_wrist,ArmProfiledPIDSubsystem m_arm) {
        // addCommands(
        //     //new MusicCommand(Filesystem.getDeployDirectory().toPath().resolve("music/frozen.chrp").toString(), new TalonFX(Constants.IntakeConstants.wristMotorID)),
        //     Commands.parallel(
        //         Commands.sequence(
        //             new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.amp),
        //             new WaitCommand(2),
        //             new MoveArmAndWristCommand(m_arm, m_wrist, MoveArmAndWristCommand.Position.inside),
        //             new MoveArmAndWristCommand(m_arm,m_wrist, MoveArmAndWristCommand.Position.load)
        //         ),
        //         new RunCommand(()->SmartDashboard.putNumber("position",m_arm.getMeasurement())),
        //         new RunCommand(()->SmartDashboard.putNumber("goal",m_arm.getGoal())),
        //         new RunCommand(()->SmartDashboard.putNumber("tolerance", m_arm.getController().getPositionTolerance())),
        //         new RunCommand(()->SmartDashboard.putBoolean("atGoal",m_arm.atGoal()))
        //     )

        // );
        addCommands(new MoveArmAndWristCommand(m_arm, m_wrist, Position.ampLow));
        addRequirements(m_wrist,m_arm);
    }
    
}
