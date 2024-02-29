package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.HangerSubsystem;

public class HangerStateCommand extends Command{
    public enum HangPosition {
        //up,
        //Hanger moves up set speed
        down,
        //Hanger moves dwn set speed
        idle
        //Hanger stays put
    }



    
    private HangerSubsystem m_hanger;
    private HangPosition state;
    
    
    public HangerStateCommand(HangerSubsystem m_hanger, HangPosition state) {
        this.m_hanger = m_hanger;
        addRequirements(m_hanger);
        this.state = state;
    }

    @Override 
    public void execute() {
        switch (state) {
            /*   case up:
             *   m_hanger.setPower(Constants.HangerConstants.hangerUpPower);
             */

            case down:
            m_hanger.setPower(Constants.HangerConstants.hangerDownPower);

            case idle:
            m_hanger.setPower(0);

        }
    }

    public void setState(HangPosition wantedState) {
        this.state = wantedState;
    }

    public HangPosition getState(HangPosition state) {
        return state;
    }



}
