package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.core.SubsystemBase;

/**
 * @author Sean Zammit, Matthew Brian, Ben Schwartz, Lucas Parker, Mitchell Barker, Darin Huang
 */

import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends ArachneRobot {
	
	public static SubsystemDriver		driver		= new SubsystemDriver();
	public static SubsystemIntake		intake		= new SubsystemIntake();
	public static SubsystemClimber		climber		= new SubsystemClimber();
	public static SubsystemArm			arm			= new SubsystemArm();
	public static SubsystemShooter		shooter		= new SubsystemShooter();
	public static SubsystemSensors		sensors		= new SubsystemSensors();
	public static SubsystemPneumatics	pneumatics	= new SubsystemPneumatics();
	public static SubsystemBase			monitor		= new SubsystemBase();
	
	public static OI oi = new OI();
	
	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
		monitor.setDefaultCommand(monitorRobot.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		return null;
	}
	
	public void initialiseRobot() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		
		table.getEntry("ledMode").setValue(1);
	}
}
