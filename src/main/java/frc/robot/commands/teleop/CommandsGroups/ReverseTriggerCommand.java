package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ReverseTriggerCommand extends SequentialCommandGroup {
    public ReverseTriggerCommand(ShooterSubsystem s_Shooter) {

        addCommands(
                Commands.runOnce(() -> s_Shooter.setTriggerSpeed(0.1)),
                new WaitCommand(0.2),
                Commands.runOnce(() -> s_Shooter.stopTrigger())

        );
    }
}
