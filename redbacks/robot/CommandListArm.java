package redbacks.robot;

import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandSetup;

import redbacks.arachne.lib.commands.CommandSetup;

public class CommandListArm extends CommandList {
	
	static {subsystemToUse = Robot.arm;}
	public static CommandSetup
		moveArm = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChFalse())),
		reverseArm = newCom(new AcMotor.Set(Robot.arm.aMotor, -0.3, new ChFalse())),
		setArm500 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(500, Robot.sensors.armEncoder, true, false, false))),
		setArm500m = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-500, Robot.sensors.armEncoder, true, false, false))),
		setArm300 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(300, Robot.sensors.armEncoder, true, false, false))),
		setArm300m = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-300, Robot.sensors.armEncoder, true, false, false))),
		setArm150 = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(150, Robot.sensors.armEncoder, true, false, false))),
		setArm150m = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(-150, Robot.sensors.armEncoder, true, false, false))),
		centreArm = newCom(new AcMotor.Set(Robot.arm.aMotor, 0.3, new ChNumSen(0, Robot.sensors.armEncoder, true, false, false)));
		
}	
