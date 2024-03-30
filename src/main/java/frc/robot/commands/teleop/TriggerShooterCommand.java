package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class TriggerShooterCommand extends Command {

  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  public TriggerShooterCommand(ShooterSubsystem shooter,IntakeSubsystem intake) {
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(shooter,intake);
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    m_shooter.trigger();
    m_intake.intake_trig();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopTrigger();
    m_intake.stopIntake();
    m_shooter.stopShoot();
    

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.isNotObjectDetected();
  }
}
