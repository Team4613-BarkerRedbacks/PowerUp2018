package redbacks.robot;
import redbacks.robot.subsystems.*;
import redbacks.arachne.core.references.CommandListStart;
import redbacks.arachne.lib.actions.AcWait;
import redbacks.arachne.lib.actions.actuators.AcMotor;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.robot.actions.AcDrive;
import redbacks.arachne.lib.actions.actuators.AcSolenoid;

public class CommandListShooter extends CommandList {
	static{subsystemToUse = Robot.shooter;}
	public static CommandSetup
		lowFire = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, false), 
			new AcSolenoid.Single(Robot.shooter.shooterSol, true), 
			new AcWait(0.5), 
			new AcSolenoid.Single(Robot.shooter.shooterSol, false)
		),
		highFirePrime = newCom(
			new AcSolenoid.Single(Robot.shooter.shooterLockSol, true),
						   new AcWait(0.5),
						   new AcSolenoid.Single(Robot.shooter.shooterSol, true)
		),
		highFireRelease = newCom()
	
	
}
