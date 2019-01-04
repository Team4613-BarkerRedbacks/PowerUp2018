package redbacks.robot.actions;

import static redbacks.arachne.ext.motion.MotionSettings.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.arachne.core.ArachneRobot;
import redbacks.arachne.ext.motion.MotionSettings;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.override.AcPIDControl2;
import redbacks.arachne.lib.override.MotionSettings2;
import redbacks.arachne.lib.sensors.NumericSensor;
import redbacks.arachne.lib.trajectories.AcPath;
import redbacks.arachne.lib.trajectories.Path;
import redbacks.robot.Robot;

/**
 * An action to trial PID driving values from the SmartDashboard rather than by deploying robot code.
 * 
 * @author Sean Zammit
 */
public class AcTuneLinearPID extends AcPath
{
	public AcTuneLinearPID(NumericSensor encoder) {
		super(new ChFalse(), false, new Path(new double[] {0, 0, 0}), Robot.driver.drivetrain, 1, 1, Robot.sensors.yaw, encoder, false, null);
	}
	
	public void onStart() {
		path.reset();
		encoder.set(0);
		ArachneRobot.isIndivDriveControl = false;
		
		this.acLinear = new AcPIDControl2(
				new ChFalse(), true,
				SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0), 0,
				SmartDashboard.getNumber("PID Target", 0) * MotionSettings.encoderTicksPerMetre * (invertEncoder ? -1 : 1),
				new Tolerances.Absolute(0.02 * MotionSettings2.encoderTicksPerMetre), encoder,
				distanceEncoderIsContinuous, distanceEncoderMin, distanceEncoderMax,
				PIDSourceType.kDisplacement, trajectoryMaxNegSpeed, trajectoryMaxPosSpeed, linearOut
		);
		
		acLinear.initialise(command);
		acRotation.initialise(command);
	}
	
	public void onRun() {
		System.out.println("Tuning PID...");
		
		acLinear.execute();
		acRotation.execute();
		
		double p = SmartDashboard.getNumber("kP", 0), i = SmartDashboard.getNumber("kI", 0), d = SmartDashboard.getNumber("kD", 0), target = SmartDashboard.getNumber("PID Target", 0);
		SmartDashboard.putNumber("kP", p);
		SmartDashboard.putNumber("kI", i);
		SmartDashboard.putNumber("kD", d);
		SmartDashboard.putNumber("PID Target", target);
		
		for(PIDController controller : acLinear.controllers) {
			controller.setPID(p, i, d);
			controller.setSetpoint(target * MotionSettings.encoderTicksPerMetre);
			
			if(acLinear.isDone()) {
				controller.reset();
				controller.enable();
			}
		}
		
		double linearOutput = linearOut.output;
		if(minOut != null && maxOut != null) linearOutput = Math.max(Math.min(linearOutput, maxOut.get()), minOut.get());
		
		if(!acLinear.isDone() && Math.abs(linearOutput) < MotionSettings2.driveMinVoltage) {
			if(linearOutput >= 0) linearOutput = MotionSettings2.driveMinVoltage;
			else linearOutput = -MotionSettings2.driveMinVoltage;
		}
		
		drivetrain.tankDrive(linearOutput + rotationOut.output, linearOutput - rotationOut.output);
	}
	
	public boolean isDone() {
		return false;
	}
}