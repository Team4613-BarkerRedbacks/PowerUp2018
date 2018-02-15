package redbacks.arachne.lib.trajectories;

import static redbacks.arachne.lib.override.MotionSettings2.*;

import edu.wpi.first.wpilibj.PIDSourceType;
import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.ext.motion.pid.AcMultiPID.PIDAxis;
import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.logic.GettableNumber;
import redbacks.arachne.lib.motors.CtrlDrivetrain;
import redbacks.arachne.lib.override.AcPIDControl2;
import redbacks.arachne.lib.sensors.NumericSensor;

/**
 * 
 *
 * @author Sean Zammit
 */
public class AcPath extends Action
{
	Path path;
	
	CtrlDrivetrain drivetrain;
	double[] driveMults = {1, 1};
	
	NumericSensor gyro;

	NumericSensor encoder;
	boolean invertEncoder;

	PIDAxis rotationOut;
	AcPIDDynamicControl acRotation;
	
	PIDAxis linearOut;
	AcPIDControl2 acLinear;
	
	GettableNumber minOut, maxOut;
	
	public AcPath(Check check, boolean shouldFinish, Path path, CtrlDrivetrain drivetrain, double leftMult, double rightMult, NumericSensor gyro,
			NumericSensor encoder, boolean invertEncoder, Tolerances tolerance) {
		super(check);
		this.path = path;
		this.drivetrain = drivetrain;
		this.driveMults[0] = leftMult;
		this.driveMults[1] = rightMult;
		this.gyro = gyro;
		this.encoder = encoder;
		this.invertEncoder = invertEncoder;
		this.linearOut = new PIDAxis(1);
		this.rotationOut = new PIDAxis(1);
		this.acRotation =
			new AcPIDDynamicControl(
				new ChFalse(), false,
				drivePIDGyrokP, drivePIDGyrokI, drivePIDGyrokD, 0,
				new GyroSetpoint(path, encoder),
				new Tolerances.Absolute(0), gyro,
				true, -180, 180,
				PIDSourceType.kDisplacement, -1, 1, rotationOut
		);
		this.acLinear = 
			new AcPIDControl2(
				new ChFalse(), shouldFinish, 
				drivePIDMotorkP, drivePIDMotorkI, drivePIDMotorkD, 0, 
				path.totalDistance * (invertEncoder ? -1 : 1), 
				tolerance, encoder, 
				distanceEncoderIsContinuous, distanceEncoderMin, distanceEncoderMax, 
				PIDSourceType.kDisplacement, trajectoryMaxNegSpeed, trajectoryMaxPosSpeed, linearOut
		);
	}
	
	public AcPath(Check check, boolean shouldFinish, Path path, CtrlDrivetrain drivetrain, double leftMult, double rightMult, NumericSensor gyro,
			NumericSensor encoder, boolean invertEncoder, Tolerances tolerance, GettableNumber minOut, GettableNumber maxOut) {
		this(check, shouldFinish, path, drivetrain, leftMult, rightMult, gyro, encoder, invertEncoder, tolerance);
		this.minOut = minOut;
		this.maxOut = maxOut;
	}
	
	public void onStart() {
		path.reset();
		gyro.set(0);
		encoder.set(0);
		ArachneRobot.isIndivDriveControl = false;
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		acLinear.execute();
		acRotation.execute();
		
		double output = linearOut.output;
		
		if(minOut != null && maxOut != null) output = Math.max(Math.min(output, maxOut.get()), minOut.get());
		
		drivetrain.tankDrive(
				(output - (rotationOut.output < 0 ? getCurvatureCompensation() * Math.abs(output) : 0) + rotationOut.output * Math.abs(output)) * driveMults[0], 
				(output - (rotationOut.output > 0 ? getCurvatureCompensation() * Math.abs(output) : 0) * Math.abs(output) - rotationOut.output * Math.abs(output)) * driveMults[1]);
	}
	
	public void onFinish() {
		acLinear.end();
		acRotation.end();
	}
	
	public boolean isDone() {
		return acLinear.isDone();
	}
	
	public double getCurvatureCompensation() {
		return Math.max(Math.min(Math.log(Math.abs(path.getCurvatureFromDistance(Math.abs(encoder.get()))) + 1) / 1.2, 1), 0);
	}
	
	public static class GyroSetpoint implements GettableNumber
	{
		public Path path;
		public NumericSensor encoder;
		
		public GyroSetpoint(Path path, NumericSensor encoder) {
			this.path = path;
			this.encoder = encoder;
		}
		
		public double get() {
			return path.getAngleFromDistance(Math.abs(encoder.get()));
		}
	}
	
	public static class ChangeMinMax implements GettableNumber
	{
		public Path path;
		public NumericSensor encoder;
		public int endProx;
		public double newSpeed;
		
		public ChangeMinMax(Path path, NumericSensor encoder, int endProx, double newSpeed) {
			this.path = path;
			this.encoder = encoder;
			this.endProx = endProx;
			this.newSpeed = newSpeed;
		}
		
		public double get() {
			return Math.abs(encoder.get() - path.totalDistance) < endProx ? newSpeed : (newSpeed < 0 ? trajectoryMaxNegSpeed : trajectoryMaxPosSpeed);
		}
	}
}
