// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.teleop.StopIntakeAndTriggerCommand;
import frc.robot.commands.teleop.TriggerShooterCommand;
import frc.robot.commands.teleop.CommandsGroups.AmpShootCommandGroup;
import frc.robot.commands.teleop.CommandsGroups.ElevatorFeederCommandGroup;
import frc.robot.commands.teleop.CommandsGroups.IntakeCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimbSubsystem;
import frc.robot.subsystems.Amp.AmpSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.LEDsubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer {

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_driverAsisstant = new CommandXboxController(
            OperatorConstants.kAsisstantControllerPort);

    private List<Subsystem> m_allSubsystems = new ArrayList<>();

    private final SwerveSubsystem m_Swerve = SwerveSubsystem.getInstance();
    private final IntakeSubsystem m_Intake = IntakeSubsystem.getInstance();
    private final ClimbSubsystem m_Climb = ClimbSubsystem.getInstance();
    private final ShooterSubsystem m_Shooter = ShooterSubsystem.getInstance();
    private final AmpSubsystem m_Amp = AmpSubsystem.getInstance();
    private final LEDsubsystem m_LED = LEDsubsystem.getInstance();

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {

        m_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_Swerve,
                        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), SwerveConstants.stickDeadband), // translationAxis
                        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), SwerveConstants.stickDeadband), // strafeAxis
                        () -> MathUtil.applyDeadband(m_driverController.getRightX(), SwerveConstants.stickDeadband), // rotationAxis
                        () -> false));

        configureAutonomous();
        configSubsystems();
        configureBindings();
    }

    private void configureBindings() {
        m_driverController.start().onTrue(Commands.runOnce(() -> m_Swerve.zeroGyro()));
        m_driverController.b().onTrue(Commands.runOnce(() -> m_Swerve.zeroGyro()));

        m_driverController.rightTrigger().onTrue(
                Commands.either(
                        new IntakeCommandGroup(m_Shooter, m_Intake),
                        new TriggerShooterCommand(m_Shooter, m_Intake),
                        m_Shooter::isNotObjectDetected));

        m_driverController.leftTrigger().onTrue(new StopIntakeAndTriggerCommand(m_Intake, m_Shooter));

        m_driverController.leftBumper().onTrue(Commands.sequence(
                Commands.runOnce(() -> m_Amp.trigger()),
                Commands.runOnce(() -> m_Intake.intake()),
                Commands.runOnce(() -> m_Shooter.triggerReverse()),
                Commands.runOnce(() -> m_Shooter.setShooterSpeed(-0.2)),
                Commands.runOnce(() -> RobotState.setGamePieceAmp(false))
        ));


        m_driverController.leftBumper().onFalse(Commands.sequence(
                Commands.runOnce(() -> m_Amp.stopTrigger()),
                Commands.runOnce(() -> m_Intake.stopIntake()),
                Commands.runOnce(() -> m_Shooter.stopTrigger()),
                Commands.runOnce(() -> m_Shooter.stopShoot())
        ));

        m_driverAsisstant.rightTrigger().toggleOnTrue(Commands.either(
                Commands.runOnce(() -> m_Shooter.stopShoot()),
                Commands.runOnce(() -> m_Shooter.shoot()), 
                m_Shooter::getShooteMotorState));

        m_driverAsisstant.b().toggleOnTrue(Commands.either(
                Commands.sequence(
                        Commands.runOnce(() -> m_Intake.stopIntake()),
                        Commands.runOnce(() -> m_Shooter.stopTrigger()),
                        Commands.runOnce(() -> m_Amp.stopTrigger())),
                new ElevatorFeederCommandGroup(m_Shooter, m_Intake, m_Amp),
                m_Amp::getElevatorTriggerMotorState));


         m_driverAsisstant.y().onTrue(new AmpShootCommandGroup(m_Shooter, m_Intake, m_Amp));
        
          m_driverAsisstant.povLeft().onTrue(Commands.sequence(
                Commands.runOnce(() -> m_Shooter.setPivotAngle(198)),
                Commands.runOnce(() -> RobotState.setLimelight(true))
          ));

        m_driverController.povUp().toggleOnTrue(Commands.either(
                Commands.runOnce(() -> m_Climb.stop()),
                Commands.runOnce(() -> m_Climb.climbUp()),
                 m_Climb::getClimbMotorState));

        m_driverController.povDown().toggleOnTrue(Commands.either(
                Commands.runOnce(() -> m_Climb.stop()),
                Commands.runOnce(() -> m_Climb.climbDown()),
                 m_Climb::getClimbMotorState));


        m_driverController.rightBumper().onTrue(Commands.sequence(
                Commands.runOnce(() -> m_Intake.intake_reverse()),
                Commands.runOnce(() -> m_Shooter.triggerReverse())
        ));

        m_driverController.rightBumper().onFalse(Commands.sequence(
                Commands.runOnce(() -> m_Intake.stopIntake()),
                Commands.runOnce(() -> m_Shooter.stopTrigger())
        ));

    }

    public void configureAutonomous() {
        HashMap<String, Command> autonomous = new HashMap<>();

        m_chooser.setDefaultOption("NO Autonom", null);
        autonomous.forEach((commandName, command) -> m_chooser.addOption(commandName, command));

        Shuffleboard.getTab("Autonomous").add(m_chooser);
    }

    public void configSubsystems() {
        m_allSubsystems.add(m_Swerve);
        m_allSubsystems.add(m_Intake);
        m_allSubsystems.add(m_Climb);
        m_allSubsystems.add(m_Shooter);
        m_allSubsystems.add(m_Amp);
        m_allSubsystems.add(m_LED);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public List<Subsystem> getSubsystems() {
        return m_allSubsystems;
    }

}