package frc.robot.commands.teleop.CommandsGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.teleop.TriggerShooterCommand;
import frc.robot.subsystems.Amp.AmpSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class AmpShootCommandGroup extends SequentialCommandGroup {
    public AmpShootCommandGroup(ShooterSubsystem m_Shooter, IntakeSubsystem m_Intake,AmpSubsystem m_Amp) {

        addCommands(
            Commands.runOnce(() -> RobotState.setShootAmp(true)),
            Commands.runOnce(() -> m_Shooter.setPivotAngle(185)),
            Commands.runOnce(() -> m_Shooter.shootAmp()),
            new WaitCommand(0.4),
            new TriggerShooterCommand(m_Shooter,m_Intake),
            Commands.runOnce(() -> RobotState.setShootAmp(false))
            


        );
    }
}
