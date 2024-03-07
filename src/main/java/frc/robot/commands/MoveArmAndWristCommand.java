package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

/**
 * command for controlling arm & wrist in auto using ArmStateCommand & WristStateCommand
 */
public class MoveArmAndWristCommand extends SequentialCommandGroup {
    public enum Position {
        intake(WristStateCommand.Position.intake,ArmStateCommand.Position.intake),
        load(WristStateCommand.Position.load,ArmStateCommand.Position.load),
        amp(WristStateCommand.Position.amp,ArmStateCommand.Position.amp),
        balance(WristStateCommand.Position.balance,ArmStateCommand.Position.balance),
        ampLow(WristStateCommand.Position.ampLow,ArmStateCommand.Position.ampLow),
        inside(WristStateCommand.Position.inside,ArmStateCommand.Position.inside);
        private WristStateCommand.Position wristPos;
        private ArmStateCommand.Position armPos;
        Position(WristStateCommand.Position wristPos, ArmStateCommand.Position armPos) {
            this.wristPos = wristPos;
            this.armPos = armPos;
        }
        public WristStateCommand.Position getWristPos() {
            return wristPos;
        }
        public ArmStateCommand.Position getArmPos() {
            return armPos;
        }
    }
    public MoveArmAndWristCommand(ArmProfiledPIDSubsystem m_arm, WristProfiledPIDSubsystem m_wrist, Position position) {
        addCommands(
            Commands.parallel(
                new WristStateCommand(m_wrist, WristStateCommand.Position.inside).once().onlyIf(()->m_arm.getGoal() != position.armPos.getPos()), //.until(m_wrist::atGoal) -> .once()
                new WaitCommand(0.1).andThen(new ArmStateCommand(m_arm, position.getArmPos()).once()) // .until(m_arm::atGoal) -> .once()
            ),
            new WristStateCommand(m_wrist, position.getWristPos()).once() // .until(m_wrist::atGoal) -> .once()
           // new InstantCommand(m_wrist::disable).onlyIf(()->position == Position.intake)
        );
        addRequirements(m_arm,m_wrist);
    }
}
