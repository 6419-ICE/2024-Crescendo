package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Combines all the Launcher subsystems 
 * ({@link LauncherSubsystem}, {@link LauncherStagingSubsystem}, and {@link LauncherVerticalAimerSubsystem}) 
 * into one Object for easier usage
 */
public class CombinedLauncherSubsystem extends SubsystemBase {
    private LauncherSubsystem m_launcher;
    private LauncherStagingSubsystem m_staging;
    private LauncherVerticalAimerSubsystem m_verticalAimer;

    public CombinedLauncherSubsystem() {
        m_launcher = new LauncherSubsystem();
        m_staging = new LauncherStagingSubsystem();
        m_verticalAimer = new LauncherVerticalAimerSubsystem();
    }
    @Override
    public void periodic() {
        //runs verticalAimer periodic to keep PID updated, maybe use addChild()???? (addChild() MIGHT do this for us, I couldnt figure out what it did for sure though)
        m_verticalAimer.periodic();
    }
    public void aimTo(double degrees) {
        m_verticalAimer.start(degrees);
    }
    public void setStaging(double power) {
        m_staging.setPower(power);
    }
    public void setLauncher(double power) {
        m_launcher.setPower(power);
    }
}
