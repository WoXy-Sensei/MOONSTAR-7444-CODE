package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ReverseTriggerAndShooterCommandGroup extends SequentialCommandGroup {
    public ReverseTriggerAndShooterCommandGroup(ShooterSubsystem s_Shooter) {

        addCommands(
                Commands.runOnce(() -> s_Shooter.setTriggerSpeed(0.1)),
                Commands.runOnce(() -> s_Shooter.setShooterSpeed(-0.15)),
                new WaitCommand(0.28),
                Commands.runOnce(() -> s_Shooter.stopTrigger()),
                Commands.runOnce(() -> s_Shooter.stopShoot())

        );
    }
}
