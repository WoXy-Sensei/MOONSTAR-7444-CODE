package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.AmpTriggerCommand;
import frc.robot.subsystems.Amp.AmpSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class AmpTriggerCommandGroup extends SequentialCommandGroup {
    public AmpTriggerCommandGroup(ShooterSubsystem m_Shooter, IntakeSubsystem m_Intake,AmpSubsystem m_Amp) {

        addCommands(
            new AmpTriggerCommand(m_Amp, m_Intake)
        );
    }
}
