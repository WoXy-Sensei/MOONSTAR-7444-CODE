package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp.AmpSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class AmpAfterFeedCommand extends Command {

  private final AmpSubsystem m_amp;
  private final IntakeSubsystem m_intake;

  public AmpAfterFeedCommand(AmpSubsystem amp,IntakeSubsystem intake) {
   
    m_amp = amp;
    m_intake = intake;
    addRequirements(amp,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_amp.trigger();
    m_intake.intake();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_amp.stopTrigger();
    m_intake.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_amp.isNotObjectDetected();
  }
}
