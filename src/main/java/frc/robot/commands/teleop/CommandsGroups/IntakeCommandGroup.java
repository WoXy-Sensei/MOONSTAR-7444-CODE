package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.IntakeAndTriggerCommand;
import frc.robot.commands.teleop.TriggerShooterCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(ShooterSubsystem s_Shooter, IntakeSubsystem s_Intake) {

        addCommands(
                new IntakeAndTriggerCommand(s_Intake, s_Shooter),
                new ReverseTriggerCommand(s_Shooter).onlyIf(s_Shooter::isShooterNotRunning),
                new TriggerShooterCommand(s_Shooter, s_Intake).onlyIf(s_Shooter::getShooteMotorState)
        );
    }
}
