package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class MusicCommand extends Command {
    TalonFX[] motors;
    Orchestra orchestra;
    
    public MusicCommand(String filePath,TalonFX... motors) {
        this.motors = motors;
        orchestra = new Orchestra();
        orchestra.loadMusic(filePath);
    }
    @Override
    public void initialize() {
        for (TalonFX motor : motors) orchestra.addInstrument(motor);
        orchestra.play();
    }
    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
        orchestra.clearInstruments();
    }
    @Override
    public boolean isFinished() {
        return !orchestra.isPlaying();
    }
    
}
