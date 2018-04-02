package redbacks.robot.actions;

import static redbacks.arachne.lib.override.MotionSettings2.drivePIDGyrokD;
import static redbacks.arachne.lib.override.MotionSettings2.drivePIDGyrokI;
import static redbacks.arachne.lib.override.MotionSettings2.drivePIDGyrokP;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.ext.motion.pid.AcMultiPID.PIDAxis;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.override.AcPIDControl2;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.robot.Robot;

public class AcTurnGimbal extends Action
{
	boolean onLeftSide;
	double angle;

	public PIDAxis rotationOut;
	public AcPIDControl2 acRotation;
	
	public AcTurnGimbal(double angle, boolean onLeftSide) {
		super(new ChFalse());
		this.onLeftSide = onLeftSide;
		this.angle = angle;
		this.rotationOut = new PIDAxis(1);
		this.acRotation = new AcPIDControl2(
					new ChFalse(), true,
					drivePIDGyrokP, drivePIDGyrokI, drivePIDGyrokD, 0,
					angle,
					new Tolerances.Absolute(3), Robot.sensors.yaw,
					true, -180, 180,
					PIDSourceType.kDisplacement, -1, 1, rotationOut
			);
	}
	
	public void onStart() {
		Robot.isIndivDriveControl = false;
		acRotation.initialise(command);
	}

	public void onRun() {
		acRotation.execute();
		double rot = rotationOut.output;
		
		if(onLeftSide) Robot.driver.drivetrain.tankDrive(rot, (rot > 0 ? -1 : 1) * MotionSettings2.driveMinVoltage);
		else Robot.driver.drivetrain.tankDrive((rot > 0 ? -1 : 1) * MotionSettings2.driveMinVoltage, rot);
		
		
	}
	
	public boolean isDone() {
		return acRotation.isDone();
	}
}