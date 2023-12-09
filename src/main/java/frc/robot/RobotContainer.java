// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class RobotContainer {

    /****************** CONSTANTS ******************/

    /* Logitech Dual Action */
    public static final int LDALeftJoystickX = 0;
    public static final int LDALeftJoystickY = 1;
    public static final int LDARightJoystickX = 2;
    public static final int LDARightJoystickY = 3;

    public static final int LDALeftTrigger = 7;
    public static final int LDARightTrigger = 8;
    public static final int LDALeftBumper = 5;
    public static final int LDARightBumper = 6;
    public static final int LDAButtonA = 2;
    public static final int LDAButtonB = 3;
    public static final int LDAButtonX = 1;
    public static final int LDAButtonY = 4;
    public static final int LDABackButton = 9;
    public static final int LDAStartButton = 10;
    public static final int LDALeftJoystick = 11;
    public static final int LDARightJoystick = 12;

    public static final double LDAForwardAxisAttenuation = -0.5;
    public static final double LDALateralAxisAttenuation = 0.5;
    public static final double LDAYawAxisAttenuation = 0.5;

    /* RadioMaster TX12 */
    public static final int RMLeftGimbalX = 0;
    public static final int RMLeftGimbalY = 1;
    public static final int RMRightGimbalX = 3;
    public static final int RMRightGimbalY = 2;

    public static final int RMSliderF = 5;
    public static final int RMSliderE = 4;
    public static final int RMSliderC = 6;

    public static final int RMButtonD = 2;
    public static final int RMButtonA = 1;

    public static final double FowardAxisAttenuation = 1.0;
    public static final double LateralAxisAttenuation = 1.0;
    public static final double YawAxisAttenuation = 0.6;

    /* Xbox */
    public static final int XLeftStickX = 0;
    public static final int XLeftStickY = 1;
    public static final int XLeftTrigger = 2;
    public static final int XRightTrigger = 4;
    public static final int XRightStickX = 4;
    public static final int XRightStickY = 5;

    public static final int XButtonA = 1;
    public static final int XButtonB = 2;
    public static final int XButtonX = 3;
    public static final int XButtonY = 4;
    public static final int XLeftBumper = 5;
    public static final int XRightBumper = 6;
    public static final int XBackButton = 7;
    public static final int XStartButton = 8;
    public static final int XLeftStick = 9;
    public static final int XRightStick = 10;
    public static final int XWindowButton = 7;

    public static final double XForwardAxisAttenuation = -0.5;
    public static final double XLateralAxisAttenuation = 0.5;
    public static final double XYawAxisAttenuation = 0.5;

    /***********************************************/

    /* Subsystems */
    public final Drive driveSub = new Drive();

    /* Controllers */
    public Joystick driverController; // Joystick 1
    public Joystick operatorButtonController; // Joystick 2
    public Joystick operatorAxisController; // Joystick 3

    /* Buttons */
    public Trigger toggleTargetButton;
    public Trigger autoAlignButton;

    /* Drive & Arm Movement */
    public int throttleJoystickID;
    public int turnJoystickID;
    public int angleJoystickID;
    public int extensionJoystickID;

    /* Shuffleboard */
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    final SendableChooser<String> stringChooser = new SendableChooser<String>();

    public RobotContainer() {
        configureButtonBindings();
        buildAutoOptions();
        driveSub.setDefaultCommand(
                new DriveRobot(driveSub, driverController, throttleJoystickID, turnJoystickID));
    }

    private void configureButtonBindings() {
        driverController = new Joystick(1);
        operatorButtonController = new Joystick(2);
    }

    private void buildAutoOptions() {
        stringChooser.setDefaultOption("Engage Only", "engageOnly");
        SmartDashboard.putData("String Chooser", stringChooser);
    }

    /* * * * * * GRAB PIECE LONG COMMUNITY SIDE * * * * * */

    InstrumentedSequentialCommandGroup Example() {
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        // path = loadPath(
        // "autoPath", 1.0, 3.0, true);

        theCommand.addCommands(new InstantCommand(() -> theCommand.addCommands(new WaitCommand(0.5))));

        return theCommand;
    }
    // Need to fix
    // private void setupInstrumentation() {
    // PPRamseteCommand.setLoggingCallbacks(
    // (PathPlannerTrajectory traj) -> {
    // this.currentTrajectory = traj;
    // },
    // (Pose2d targetPose) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
    // .setDouble(targetPose.getX());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
    // .setDouble(targetPose.getY());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
    // .setDouble(targetPose.getRotation().getDegrees());
    // },
    // (ChassisSpeeds setpointSpeeds) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
    // .setDouble(setpointSpeeds.vxMetersPerSecond);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
    // .setDouble(setpointSpeeds.vyMetersPerSecond);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
    // .setDouble(setpointSpeeds.omegaRadiansPerSecond);
    // },
    // (Translation2d translationError, Rotation2d rotationError) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
    // .setDouble(translationError.getX());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
    // .setDouble(translationError.getY());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
    // .setDouble(rotationError.getDegrees());
    // });

    // PathFromCurrentLocation.setLoggingCallbacks(
    // (PathPlannerTrajectory traj) -> {
    // this.currentTrajectory = traj;
    // },
    // (Pose2d targetPose) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
    // .setDouble(targetPose.getX());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
    // .setDouble(targetPose.getY());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
    // .setDouble(targetPose.getRotation().getDegrees());
    // },
    // (ChassisSpeeds setpointSpeeds) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
    // .setDouble(setpointSpeeds.vxMetersPerSecond);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
    // .setDouble(setpointSpeeds.vyMetersPerSecond);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
    // .setDouble(setpointSpeeds.omegaRadiansPerSecond);
    // },
    // (Translation2d translationError, Rotation2d rotationError) -> {
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
    // .setDouble(translationError.getX());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
    // .setDouble(translationError.getY());
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
    // .setDouble(rotationError.getDegrees());
    // });

    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y").setDouble(0.0);
    // NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees").setDouble(0.0);
    // }

    public Command getAutonomousCommand() {
        this.currentTrajectory = new PathPlannerTrajectory();

        String choice = stringChooser.getSelected();
        if (choice == "autoPath") {
            return null;
        } else {
            return null;
        }
    }

    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }
}