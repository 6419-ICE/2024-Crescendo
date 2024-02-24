package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;
import frc.robot.subsystems.WristProfiledPIDSubsystem;

/**
 * command for controlling arm & wrist in auto using ArmProfiledPIDStateCommand & WristProfiledPIDStateCommand
 */
public class MoveArmAndWristCommand extends ParallelCommandGroup {
    public enum Position {
        intake(WristProfiledPIDStateCommand.Position.intake,ArmProfiledPIDStateCommand.Position.intake),
        load(WristProfiledPIDStateCommand.Position.load,ArmProfiledPIDStateCommand.Position.load),
        amp(WristProfiledPIDStateCommand.Position.amp,ArmProfiledPIDStateCommand.Position.amp),
        inside(WristProfiledPIDStateCommand.Position.inside,ArmProfiledPIDStateCommand.Position.inside);
        private WristProfiledPIDStateCommand.Position wristPos;
        private ArmProfiledPIDStateCommand.Position armPos;
        Position(WristProfiledPIDStateCommand.Position wristPos, ArmProfiledPIDStateCommand.Position armPos) {
            this.wristPos = wristPos;
            this.armPos = armPos;
        }
        public WristProfiledPIDStateCommand.Position getWristPos() {
            return wristPos;
        }
        public ArmProfiledPIDStateCommand.Position getArmPos() {
            return armPos;
        }
    }
    public MoveArmAndWristCommand(ArmProfiledPIDSubsystem m_arm, WristProfiledPIDSubsystem m_wrist, Position position) {
        addCommands(
            
            new ArmProfiledPIDStateCommand(m_arm, position.getArmPos()).until(m_arm::atGoal),
            new WristProfiledPIDStateCommand(m_wrist, position.getWristPos()).until(m_wrist::atGoal)
        );
        addRequirements(m_arm,m_wrist);
    }
}
