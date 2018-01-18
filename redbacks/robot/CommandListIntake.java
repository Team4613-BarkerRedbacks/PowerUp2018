package redbacks.robot;

import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.commands.CommandSetup;

public class CommandListIntake extends CommandList {
	
	static {subsystemToUse = Robot.intake;}
	public static CommandSetup
		intake = newCom(new AcMotor.Set(Robot.intake.intakeRightMotor, 0.25, new ChFalse()), (new AcMotor.Set(Robot.intake.intakeLeftMotor, 0.25, new ChFalse()))),
		outtake = newCom(new AcMotor.Set(Robot.intake.intakeRightMotor, -0.25, new ChFalse()), (new AcMotor.Set(Robot.intake.intakeRightMotor, -0.25, new ChFalse())));
}