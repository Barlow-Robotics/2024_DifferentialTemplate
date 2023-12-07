// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sim.PhysicsSim;

import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/* The VM is configured to automatically run this class, and to call the functions
corresponding to each mode, as described in the TimedRobot documentation. If you change
the name of this class or the package after creating this project, you must also update
the build.gradle file in the project. */
public class Robot extends LoggedRobot {
    // public final Arm armSub = new Arm();

    private Command autonomousCommand;

    private Field2d field;

    private RobotContainer robotContainer;

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings // O

    static long startTime = System.currentTimeMillis() ;

    static HashMap<Command, Long> startTimes = new HashMap() ;

    Logger logger = Logger.getInstance();

    @Override
    public void disabledInit() {
        robotContainer.driveSub.stopMoving();
    }

    @Override
    public void disabledPeriodic() {
    }

    // @Override
    public void autonomousInit() {
        robotContainer.driveSub.resetHeading();

        SequentialCommandGroup cg = new SequentialCommandGroup();

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            cg.addCommands(autonomousCommand);
        }

        cg.schedule();
    }

    // @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.driveSub.simulationInit();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    static public void reportCommandStart(Command c) {
        double deltaTime = ((double)System.currentTimeMillis() - startTime) / 1000.0 ;
        System.out.println(deltaTime + ": Started " + c.getName())    ;     
        startTimes.putIfAbsent(c, System.currentTimeMillis() ) ;
    }

    static public void reportCommandFinish(Command c) {
        if ( startTimes.containsKey(c)) {
            long currentTime = System.currentTimeMillis() ;
            double deltaTime = ((double)currentTime - startTime) / 1000.0 ;
            double elapsedTime = (double)(currentTime - startTimes.get(c)) / 1000.0  ;
            System.out.println(deltaTime + ": Finished (elapsed time " + elapsedTime + ")" + c.getName()) ;     
            startTimes.remove(c) ;
        }
    }


    public void robotInit() {

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(true) ;

        field = new Field2d();
        SmartDashboard.putData("Field", field) ;

        CommandScheduler.getInstance().onCommandInitialize( Robot::reportCommandStart ) ;
        CommandScheduler.getInstance().onCommandFinish(Robot::reportCommandFinish);        
        CommandScheduler.getInstance().onCommandInterrupt( this::handleInterrupted) ;

        logger.recordMetadata("ProjectName", "WPI-2023-Mantis"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            // setUseTiming(false); // Run as fast as possible
            // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            logger.addDataReceiver(new WPILOGWriter(""));
            logger.addDataReceiver(new NT4Publisher());
        }
        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(robotContainer.driveSub);
        field.setRobotPose(robotContainer.driveSub.getPose());
        if (robotContainer.getCurrentTrajectory() != null) {
            field.getObject("traj").setTrajectory(robotContainer.getCurrentTrajectory());
        }

        if (this.isDisabled()) {
        }
        CommandScheduler.getInstance().run();

    }

    private void handleInterrupted( Command c) {
        System.out.println("Commmand " + c + " named " + c.getName() + " was interrupted") ;
    }

}