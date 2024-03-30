package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.teleop.AmpFeederCommand;
import frc.robot.commands.teleop.AmpAfterFeedCommand;
import frc.robot.commands.teleop.ShooterPivotCommand;
import frc.robot.subsystems.Amp.AmpSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ElevatorFeederCommandGroup extends SequentialCommandGroup {
    public ElevatorFeederCommandGroup(ShooterSubsystem m_Shooter, IntakeSubsystem m_Intake,AmpSubsystem m_Amp) {

        addCommands(
            Commands.runOnce(() -> RobotState.setGamePieceAmp(true)),
            new ShooterPivotCommand(m_Shooter, 20),
            new WaitCommand(0.1),
            new AmpFeederCommand(m_Intake, m_Shooter, m_Amp),
            new AmpAfterFeedCommand(m_Amp, m_Intake)

        );
    }
}
