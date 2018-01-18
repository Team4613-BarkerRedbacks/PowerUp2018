package redbacks.robot;

import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.commands.CommandSetup;

public class CommandListClimber extends CommandList {
	
	static {subsystemToUse = Robot.climber;}
	public static CommandSetup
		climb = newCom(new AcMotor.Set(Robot.climber.climberMotor, 0.25, new ChFalse()));
	
}
