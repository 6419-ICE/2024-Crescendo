package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

/**
 * command for controlling arm & wrist in auto using ArmStateCommand & WristStateCommand
 */
public class MoveArmAndWristCommand extends ParallelCommandGroup {
    public enum Position {
        intake(WristStateCommand.Position.intake,ArmStateCommand.Position.intake),
        load(WristStateCommand.Position.load,ArmStateCommand.Position.load),
        amp(WristStateCommand.Position.amp,ArmStateCommand.Position.amp),
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
            new ArmStateCommand(m_arm, position.getArmPos()).until(m_arm::atGoal),
            new WristStateCommand(m_wrist, position.getWristPos()).until(m_wrist::atGoal)
        );
        addRequirements(m_arm,m_wrist);
    }
}
