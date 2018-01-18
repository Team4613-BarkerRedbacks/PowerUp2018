package redbacks.robot;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.robot.subsystems.SubsystemArm;
import redbacks.robot.subsystems.SubsystemClimber;
import redbacks.robot.subsystems.SubsystemDriver;
import redbacks.robot.subsystems.SubsystemIntake;
import redbacks.robot.subsystems.SubsystemSensors;
import redbacks.robot.subsystems.SubsystemShooter;

public class Robot extends ArachneRobot {
	
	public static SubsystemDriver 	driver  = new SubsystemDriver();
	public static SubsystemIntake 	intake  = new SubsystemIntake();
	public static SubsystemClimber  climber = new SubsystemClimber();
	public static SubsystemArm		arm		= new SubsystemArm();
	public static SubsystemShooter  shooter = new SubsystemShooter();
	public static SubsystemSensors  sensors = new SubsystemSensors();
	
	public static OI oi = new OI();
	
	@Override
	public void initDefaultCommands() {
		driver.setDefaultCommand(CommandList.drive.c());
		oi.mapOperations();
	}

	@Override
	public CommandBase getAutonomous(int autoID) {
		// TODO Auto-generated method stub
		return null;
	}

}
