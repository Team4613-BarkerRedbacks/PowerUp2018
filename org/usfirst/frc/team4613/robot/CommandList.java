package org.usfirst.frc.team4613.robot;

import org.usfirst.frc.team4613.robot.actions.AcDrive;

import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.commands.CommandSetup;

public class CommandList extends CommandListStart {
	static {subsystemToUse = Robot.driver;}
	public static CommandSetup
		drive = newCom(new AcDrive());
}
