package org.usfirst.frc.team4613.robot;

import org.usfirst.frc.team4613.robot.subsystems.SubsystemDriver;
import org.usfirst.frc.team4613.robot.subsystems.SubsystemIntake;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;

public class Robot extends ArachneRobot {
	
	public static SubsystemDriver driver = new SubsystemDriver();
	public static SubsystemIntake intake = new SubsystemIntake();
	
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
