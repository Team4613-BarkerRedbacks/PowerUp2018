package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * 
 * 
 * @author Sean Zammit, Matthew Brian, Ben Schwarz, Lucas Parker, Mitchell Barker, Darin Huang
 */
public class Robot extends ArachneRobot
{
	public static SubsystemSensors sensors = new SubsystemSensors();
	public static SubsystemDriver driver = new SubsystemDriver();
	public static SubsystemIntake intake = new SubsystemIntake();
	public static SubsystemClimber climber = new SubsystemClimber();
	public static SubsystemArm arm = new SubsystemArm();
	public static SubsystemShooter shooter = new SubsystemShooter();

	public static OI oi = new OI();

	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
		arm.setDefaultCommand(armLoop.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		return null;
	}

	public void initialiseTeleop() {
		Scheduler.getInstance().removeAll();
	}
	
	public void initialiseDisabled() {
		Scheduler.getInstance().removeAll();
	}
}
