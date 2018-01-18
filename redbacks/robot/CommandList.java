package redbacks.robot;

import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.AcDrive;

public class CommandList extends CommandListStart {
	static {subsystemToUse = Robot.driver;}
	public static CommandSetup
		drive = newCom(new AcDrive());
}
