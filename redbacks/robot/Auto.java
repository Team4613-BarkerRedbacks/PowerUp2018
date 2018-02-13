package redbacks.robot;

import redbacks.arachne.core.references.AutoStart;
import static redbacks.arachne.lib.override.MotionSettings2.encoderTicksPerMetre;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.actions.*;
import redbacks.arachne.lib.checks.*;
import redbacks.arachne.lib.checks.analog.ChNumSen;
import redbacks.arachne.lib.commands.CommandBase;
import redbacks.arachne.lib.commands.CommandSetup;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.robot.actions.AcResetSensors;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;
import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart {
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			case(1): 
				return createAuto(
					new AcPrint("Starting"),
					new AcPath(new ChFalse(), true, wallToLR2, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(250)),
					new AcPrint("Ending")
				);
			case(2): 
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR3.totalDistance - 0.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHR3, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(250),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.5),
							new AcPath.ChangeMinMax(wallToHR3, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.5))
				);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(actions)).c();
	}
}
