package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.subsystems.*;
import static redbacks.robot.CommandList.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * 
 * 
 * @author Sean Zammit, Matthew Brian, Ben Schwarz, Lucas Parker, Mitchell Barker, Darin Huang
 */
public class Robot extends ArachneRobot
{
	public static SubsystemSensors		sensors		= new SubsystemSensors();
	public static SubsystemDriver		driver		= new SubsystemDriver();
	public static SubsystemIntake		intake		= new SubsystemIntake();
	public static SubsystemClimber		climber		= new SubsystemClimber();
	public static SubsystemArm			arm			= new SubsystemArm();
	public static SubsystemShooter		shooter		= new SubsystemShooter();
	
	public static OI oi = new OI();
	
	public void initDefaultCommands() {
		driver.setDefaultCommand(drive.c());
		sensors.setDefaultCommand(readSensors.c());
//		arm.setDefaultCommand(armLoop.c());
		oi.mapOperations();
	}

	public CommandBase getAutonomous(int autoID) {
		//TODO enable for secondary programmer
		//return autoID < 100 ? Auto.getAutonomous(autoID) : Auto2.getAutonomous(autoID);
		return Auto.getAutonomous(autoID);
	}
	
	public void initialiseRobot() {
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		limelightTable.getEntry("ledMode").forceSetValue(1);
		
		MotionSettings2.encoderTicksPerMetre = 26713;
	}
	
	public void initialiseAuto() {
		driver.centreEncoderSol.set(false);
	}
	
	public void initialiseTeleop() {
		driver.centreEncoderSol.set(true);
	}
}
