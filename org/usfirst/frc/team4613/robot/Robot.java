package org.usfirst.frc.team4613.robot;

import org.usfirst.frc.team4613.robot.subsystems.SubsystemDriver;

import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.lib.commands.CommandBase;

public class Robot extends ArachneRobot {
	
	public static OI oi = new OI();
	public static SubsystemDriver driver = new SubsystemDriver();
	
	@Override
	public void initDefaultCommands() {
		// TODO Auto-generated method stub

	}

	@Override
	public CommandBase getAutonomous(int autoID) {
		// TODO Auto-generated method stub
		return null;
	}

}
