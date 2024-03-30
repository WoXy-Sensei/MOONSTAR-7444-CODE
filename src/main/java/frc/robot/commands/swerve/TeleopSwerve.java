package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  
  double scale = 1;

  public TeleopSwerve(
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
   

    double translationVal = translationLimiter.calculate(translationSup.getAsDouble());
    double strafeVal = strafeLimiter.calculate(strafeSup.getAsDouble());
    double rotationVal = rotationLimiter.calculate(rotationSup.getAsDouble());

  
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
        rotationVal * SwerveConstants.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true); // True -> driving based on percent output, False -> driving based on PID, FeedForward
  }
}