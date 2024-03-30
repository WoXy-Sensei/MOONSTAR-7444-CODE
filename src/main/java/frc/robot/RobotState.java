// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
package frc.robot;

/** Add your docs here. */
public class RobotState {

    private static RobotState robotState;
    public enum SwerveState {
        MOVING,
        REST
    }

    public static boolean gamePieceInShooter = false;
    public static boolean gamePieceInAmp = false;
    public static SwerveState currentSwerveState = SwerveState.REST;
    public static boolean shooterMotorState = false;
    public static boolean shooterMotorReady = false;
    public static boolean robotAlignSpeaker = false;
    public static boolean pivotReady = false;
    public static boolean shootAmp = false;
    public static boolean limelightDeactive = false;

    private RobotState() {
        currentSwerveState = SwerveState.REST;
    }

    public static boolean isTeleop() {
        return edu.wpi.first.wpilibj.RobotState.isTeleop();
    }

    public static void setGamePieceShooter(boolean val) {
        gamePieceInShooter = val;
    }

    public static void setGamePieceAmp(boolean val) {
        gamePieceInAmp = val;
    }

    public static void setShootAmp(boolean val) {
        shootAmp = val;
    }

    public static void setLimelight(boolean val) {
        limelightDeactive = val;
    }

    public static void setSwerve(SwerveState swerveState) {
        currentSwerveState = swerveState;
    }

    public static RobotState getInstance() {
        if (robotState == null) {
            robotState = new RobotState();
        }
        return robotState;
    }

}
