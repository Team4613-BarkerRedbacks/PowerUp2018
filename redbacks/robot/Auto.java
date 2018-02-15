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
import redbacks.robot.actions.AcSetArm;

import static redbacks.robot.Robot.*;
import static redbacks.robot.RobotMap.*;
import static redbacks.robot.PathList.*;
import static redbacks.robot.CommandList.*;

public class Auto extends AutoStart {
	
	public static CommandBase getAutonomous(int autoNumber) {
		switch(autoNumber) {
			case(2):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHR.totalDistance - 0.75 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHR, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(250),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), -0.5),
							new AcPath.ChangeMinMax(wallToHR, sensors.driveCentreEncoder, (int) (1.5 * encoderTicksPerMetre), 0.5))
				);
			case(3):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL2.totalDistance - 2 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(quickFire),
							new AcSetArm(75)
					),
					new AcPath(new ChFalse(), true, wallToHL2, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
				);
			case(4):
				return createAuto(
					new AcResetSensors(),
					new AcSeq.Parallel(
							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 4 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFirePrime),
							new AcDoNothing(new ChNumSen(wallToHL3.totalDistance - 1.25 * encoderTicksPerMetre, sensors.driveCentreEncoder, true, true, false)),
							new AcSeq.Parallel(highFireRelease)
					),
					new AcPath(new ChFalse(), true, wallToHL3, driver.drivetrain, 1, 1, 
							sensors.yaw, sensors.driveCentreEncoder, false, 
							new Tolerances.Absolute(0.1 * encoderTicksPerMetre))
				);
			default: return null;
		}
	}
	
	public static CommandBase createAuto(Action... actions) {
		return new CommandSetup(null, new AcSeq.Parallel(armLoop), new AcSeq.Parallel(actions)).c();
	}
}
