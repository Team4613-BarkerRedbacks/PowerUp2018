package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * @author Ben Schwarz, Darin Huang, Lucas Parker, Matthew Brian, Mitchell Barker, Sean Zammit
 */
public class Robot extends ArachneRobot
{
	// Subsystems
	public static SubsystemSensors sensors = new SubsystemSensors();
	public static SubsystemDriver driver = new SubsystemDriver();
	public static SubsystemIntake intake = new SubsystemIntake();
	public static SubsystemClimber climber = new SubsystemClimber();
	public static SubsystemArm arm = new SubsystemArm();
	public static SubsystemShooter shooter = new SubsystemShooter();

	public static OI oi = new OI();

	// Control variables
	public static boolean isLimelightVision = false;
	public boolean hasCameraStarted = false;

	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
		arm.setDefaultCommand(armLoop.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		return Auto.getAutonomous(autoID);
	}

	public void initialiseRobot() {
		// Motion constants for path-following
		MotionSettings2.encoderTicksPerMetre = 25850;
		MotionSettings2.trajectoryMaxNegSpeed = -0.8;
		MotionSettings2.trajectoryMaxPosSpeed = 0.8;
		
		if(!hasCameraStarted) {
			CameraServer.getInstance().startAutomaticCapture();
			hasCameraStarted = true;
		}
	}

	public void initialiseAuto() {
		driver.centreEncoderSol.set(false);
	}

	public void initialiseTeleop() {
		Scheduler.getInstance().removeAll();
		driver.centreEncoderSol.set(true);
	}
	
	public void initialiseDisabled() {
		Scheduler.getInstance().removeAll();
	}
}
