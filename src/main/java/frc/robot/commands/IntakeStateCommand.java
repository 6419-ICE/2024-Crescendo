package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * State based command for controlling the intake
 */
public class IntakeStateCommand extends Command {
    public enum State {
        intake,
        outtake,
        idle
    }
    private IntakeSubsystem m_intake;
    private State currentState;
    public IntakeStateCommand(IntakeSubsystem m_intake) {
        this(m_intake,State.idle);
    }
    public IntakeStateCommand(IntakeSubsystem m_intake,State initialState) {
        this.m_intake = m_intake;
        addRequirements(this.m_intake);
        currentState = initialState;
    }
    @Override
    public void execute() {
        if (RobotContainer.getIntakeButton()) setState(State.intake);
        else if (RobotContainer.getOuttakeButton()) setState(State.outtake);
        else setState(State.idle);
        switch(currentState) {
            case idle -> m_intake.setSpeed(0); 
            case intake -> m_intake.setSpeed(Constants.IntakeConstants.intakeSpeed); 
            case outtake -> m_intake.setSpeed(Constants.IntakeConstants.outtakeSpeed);
        }
    }
    public void setState(State state) {
        currentState = state;
    }
}
