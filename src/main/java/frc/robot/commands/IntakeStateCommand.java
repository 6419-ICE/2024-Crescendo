package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * State based command for controlling the intake. Mostly for teleop
 */
public class IntakeStateCommand extends Command {
    public enum State {
        /**
         * Sets the motor power so the intake is pulling in 
         */
        intake,
        /**
         * Sets the motor power so the intake is pushing out
         */
        outtake,
        /**
         * Sets the motor power to 0
         */
        idle,
        /**
         * Sets the motor power to 0 as well as stopping all user input
         */
        frozen
    }
    int highAmpCount = 0; //keeps track of how many loops the currentoutput has been irregularly high in a row
    private IntakeSubsystem m_intake;
    private State currentState; //stores the current State of the intake
    private BooleanSupplier intake,outtake; //intake/outtake buttons
    private boolean canFreeze;
    public IntakeStateCommand(IntakeSubsystem m_intake, boolean canFreeze) {
        this(m_intake,canFreeze,State.idle);
    }
    public IntakeStateCommand(IntakeSubsystem m_intake,boolean canFreeze,State initialState) {
        this.m_intake = m_intake;
        addRequirements(this.m_intake);
        currentState = initialState;
        this.canFreeze = canFreeze;
    }
    @Override
    public void execute() {
        if (m_intake.getMotor().getOutputCurrent() > 40) { //checks if the intake is pulling in a note by checking if the output current is higher than usual.
            if (highAmpCount < 3) highAmpCount++; //this counter prevents the motor's startup current from triggering a freeze
            else {
                m_intake.setHasNote(true);
            } //todo make this retract arm too (Idealy the intake will be frozen when inside the robot, for now I manually freeze it here)
        }  else highAmpCount = 0; //reset counter when current returns to normal
        setState(m_intake.hasNote() && canFreeze ? State.frozen : getState());
        if (intake != null && outtake != null && getState() != State.frozen) { //check if buttons have been fully binded before checking for input. Also checks if the intake is "frozen"
            //set states based on input. If multiple buttons are pressed, intake takes priority
            if (intake.getAsBoolean()) setState(State.intake); 
            else if (outtake.getAsBoolean()) setState(State.outtake);
            else setState(State.idle);
        } 
        switch(currentState) { //update motor based on the current state
            case idle -> m_intake.setSpeed(0); 
            case intake -> m_intake.setSpeed(Constants.IntakeConstants.intakeSpeed); 
            case outtake -> m_intake.setSpeed(Constants.IntakeConstants.outtakeSpeed);
            case frozen -> m_intake.setSpeed(0);
        }
    }
    /**
     * sets the current {@link State} that this Command uses.
     * @param state the state
     */
    public void setState(State state) {
        currentState = state;
    }
    public State getState() {
        return currentState;
    }
    /**
     * sets the buttons for intaking & outtaking. You can give this method a {@link JoystickButton}, as it extends BooleanSupplier.
     * If you are mot using a JoystickButton or other subclass of {@link BooleanSupplier}, I recommend you use a <a href="https://www.w3schools.com/java/java_lambda.asp">lambda statement</a> 
     * @param intake
     * @param outtake
     */
    public void setButtons(BooleanSupplier intake,BooleanSupplier outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }
}
