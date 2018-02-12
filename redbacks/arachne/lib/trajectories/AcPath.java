package redbacks.arachne.lib.trajectories;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.AcPIDControl;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.ext.motion.pid.AcMultiPID.PIDAxis;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.arachne.lib.sensors.NumericSensor;

import static redbacks.arachne.ext.motion.MotionSettings.*;

public class AcPath extends Action
{
	Path path;
	
	CtrlDrivetrain drivetrain;
	double[] driveMults = {1, 1};
	
	NumericSensor gyro;
	double gyroComp;

	NumericSensor encoder;
	boolean invertEncoder;

	PIDAxis linearOut;
	AcPIDControl acLinear;
	public AcPath(Check check, boolean shouldFinish, Path trajectory, CtrlDrivetrain drivetrain, double leftMult, double rightMult, NumericSensor gyro, double gyroComp,
			NumericSensor encoder, boolean invertEncoder, double p, double i, double d, Tolerances tolerance) {
		super(check);
		this.path = trajectory;
		this.drivetrain = drivetrain;
		this.driveMults[0] = leftMult;
		this.driveMults[1] = rightMult;
		this.gyro = gyro;
		this.gyroComp = gyroComp;
		this.encoder = encoder;
		this.invertEncoder = invertEncoder;
		
		this.linearOut = new PIDAxis(1);
		this.acLinear = new AcPIDControl(new ChFalse(), shouldFinish, p, i, d, 0, trajectory.totalDistance * (invertEncoder ? -1 : 1), tolerance, encoder, distanceEncoderIsContinuous, distanceEncoderMin, distanceEncoderMax, PIDSourceType.kDisplacement, trajectoryMaxNegSpeed, trajectoryMaxPosSpeed, linearOut);
	}
	
	public void onStart() {
		path.reset();
		gyro.set(0);
		encoder.set(0);
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
	}
	
	public void onRun() {
		acLinear.execute();
		drivetrain.tankDrive((linearOut.output - (getGyro() - path.getAngleFromDistance(Math.abs(encoder.get()))) * gyroComp * Math.abs(linearOut.output)) * driveMults[0], (linearOut.output + (getGyro() - path.getAngleFromDistance(Math.abs(encoder.get()))) * gyroComp * Math.abs(linearOut.output)) * driveMults[1]);
	}
	
	public void onFinish() {
		acLinear.end();
	}
	
	public boolean isDone() {
		return acLinear.isDone();
	}
	
	public double getGyro() {
		double angle = gyro.get() % 360;
		if(angle > 180) angle -= 360;
		else if(angle <= -180) angle += 360;
		return angle;
	}
}
