package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.AcPrint;
import redbacks.arachne.lib.actions.AcSeq;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.ChTime;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.trajectories.AcPath;


public class Auto extends AutoStart {
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			case(1): return createAuto(
				new AcPrint("Starting"),
				new AcPath(new ChFalse(), true, TestPaths.pathTest1, Robot.driver.drivetrain, -1, -1, 
						Robot.sensors.yaw, -0.01, Robot.sensors.driveCentreEncoder, true, 
						RobotMap.drivePIDMotorkP, RobotMap.drivePIDMotorkI, RobotMap.drivePIDMotorkD, new Tolerances.Absolute(250)),
				new AcPrint("Ending")
			);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(actions)).c();
	}
}
