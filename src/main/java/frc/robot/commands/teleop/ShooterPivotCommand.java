package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterPivotCommand extends Command {

  private final ShooterSubsystem m_shooter;
  private double angle;

  public ShooterPivotCommand(ShooterSubsystem shooter,double angleValue) {
   
    m_shooter = shooter;
    angle = angleValue;
   
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setPivotAngle(angle);
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
    return m_shooter.pivotAtSetpoint();
  }
}
