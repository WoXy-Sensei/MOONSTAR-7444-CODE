package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class StopIntakeAndTriggerCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public StopIntakeAndTriggerCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntake();
    m_shooter.stopTrigger();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
