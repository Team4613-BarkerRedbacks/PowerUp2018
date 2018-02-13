package redbacks.arachne.lib.trajectories;

import edu.wpi.first.wpilibj.*;
import redbacks.arachne.ext.motion.pid.Tolerances;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.logic.GettableNumber;

/**
 * An action for a {@link PIDController}. With a set of PID parameters, it will output to any number of valid {@link PIDOutput PIDOutputs}.
 *
 * @author Sean Zammit
 */
public class AcPIDDynamicControl extends AcPIDControl2
{
	GettableNumber dynamicTarget;

	/**
	 * Constructor for an action for a {@link PIDController}. It will end once it reaches its target.
	 * 
	 * @param p P multiplier.
	 * @param i I multiplier.
	 * @param d D multiplier.
	 * @param target The target value for the PIDController.
	 * @param tolerance The tolerance on the target value, used to determine whether the PIDController is at its target.
	 * @param input The sensor that the PIDController is using to determine its values.
	 * @param outputs Any {@link PIDOutput PIDOutputs} to be set by the PIDController.
	 */
	public AcPIDDynamicControl(double p, double i, double d, GettableNumber target, Tolerances tolerance, PIDSource input, PIDOutput... outputs) {
		super(PIDController.kDefaultPeriod, new ChFalse(), true, p, i, d, 0, 0, tolerance, input, false, 0, 0, PIDSourceType.kDisplacement, -1, 1, outputs);
		this.dynamicTarget = target;
	}

	/**
	 * Constructor for an action for a {@link PIDController}. It will end once it reaches its target.
	 * 
	 * @param p P multiplier.
	 * @param i I multiplier.
	 * @param d D multiplier.
	 * @param target The target value for the PIDController.
	 * @param tolerance The tolerance on the target value, used to determine whether the PIDController is at its target.
	 * @param input The sensor that the PIDController is using to determine its values.
	 * @param isContinuous Whether the sensor wraps around at a certain point, such as a gyro going from 180 to -180.
	 * @param minIn The minimum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param maxIn The maximum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param outputs Any {@link PIDOutput PIDOutputs} to be set by the PIDController.
	 */
	public AcPIDDynamicControl(double p, double i, double d, GettableNumber target, Tolerances tolerance, PIDSource input, boolean isContinuous, double minIn, double maxIn, PIDOutput... outputs) {
		super(PIDController.kDefaultPeriod, new ChFalse(), true, p, i, d, 0, 0, tolerance, input, isContinuous, minIn, maxIn, PIDSourceType.kDisplacement, -1, 1, outputs);
		this.dynamicTarget = target;
	}

	/**
	 * Constructor for an action for a {@link PIDController}. It can be set to end once it reaches its target, or when a check is true.
	 * 
	 * @param check A check that will determine whether the PIDController should stop.
	 * @param shouldFinish Whether the action should finish once the PIDController is on target.
	 * @param p P multiplier.
	 * @param i I multiplier.
	 * @param d D multiplier.
	 * @param target The target value for the PIDController.
	 * @param tolerance The tolerance on the target value, used to determine whether the PIDController is at its target.
	 * @param input The sensor that the PIDController is using to determine its values.
	 * @param isContinuous Whether the sensor wraps around at a certain point, such as a gyro going from 180 to -180.
	 * @param minIn The minimum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param maxIn The maximum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param type The {@link PIDSourceType} for the sensor. This can be either rate or displacement.
	 * @param minOut The minimum output to the {@link PIDOutput PIDOutputs}.
	 * @param maxOut The maximum output to the PIDOutputs.
	 * @param outputs AnyPIDOutputs to be set by the PIDController.
	 */
	public AcPIDDynamicControl(Check check, boolean shouldFinish, double p, double i, double d, GettableNumber target, Tolerances tolerance, PIDSource input, boolean isContinuous, double minIn, double maxIn, PIDSourceType type, double minOut, double maxOut, PIDOutput... outputs) {
		super(PIDController.kDefaultPeriod, check, shouldFinish, p, i, d, 0, 0, tolerance, input, isContinuous, minIn, maxIn, type, minOut, maxOut, outputs);
		this.dynamicTarget = target;
	}

	/**
	 * Constructor for an action for a {@link PIDController}. It can be set to end once it reaches its target, or when a check is true.
	 * 
	 * @param check A check that will determine whether the PIDController should stop.
	 * @param shouldFinish Whether the action should finish once the PIDController is on target.
	 * @param p P multiplier.
	 * @param i I multiplier.
	 * @param d D multiplier.
	 * @param f F multiplier.
	 * @param target The target value for the PIDController.
	 * @param tolerance The tolerance on the target value, used to determine whether the PIDController is at its target.
	 * @param input The sensor that the PIDController is using to determine its values.
	 * @param isContinuous Whether the sensor wraps around at a certain point, such as a gyro going from 180 to -180.
	 * @param minIn The minimum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param maxIn The maximum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param type The {@link PIDSourceType} for the sensor. This can be either rate or displacement.
	 * @param minOut The minimum output to the {@link PIDOutput PIDOutputs}.
	 * @param maxOut The maximum output to the PIDOutputs.
	 * @param outputs AnyPIDOutputs to be set by the PIDController.
	 */
	public AcPIDDynamicControl(Check check, boolean shouldFinish, double p, double i, double d, double f, GettableNumber target, Tolerances tolerance, PIDSource input, boolean isContinuous, double minIn, double maxIn, PIDSourceType type, double minOut, double maxOut, PIDOutput... outputs) {
		super(PIDController.kDefaultPeriod, check, shouldFinish, p, i, d, f, 0, tolerance, input, isContinuous, minIn, maxIn, type, minOut, maxOut, outputs);
		this.dynamicTarget = target;
	}

	/**
	 * Constructor for an action for a {@link PIDController}. It can be set to end once it reaches its target, or when a check is true.
	 * 
	 * @param check A check that will determine whether the PIDController should stop.
	 * @param shouldFinish Whether the action should finish once the PIDController is on target.
	 * @param p P multiplier.
	 * @param i I multiplier.
	 * @param d D multiplier.
	 * @param f F multiplier.
	 * @param target The target value for the PIDController.
	 * @param tolerance The tolerance on the target value, used to determine whether the PIDController is at its target.
	 * @param input The sensor that the PIDController is using to determine its values.
	 * @param isContinuous Whether the sensor wraps around at a certain point, such as a gyro going from 180 to -180.
	 * @param minIn The minimum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param maxIn The maximum value from the sensor. Defaults to 0 - this will be ignored.
	 * @param type The {@link PIDSourceType} for the sensor. This can be either rate or displacement.
	 * @param minOut The minimum output to the {@link PIDOutput PIDOutputs}.
	 * @param maxOut The maximum output to the PIDOutputs.
	 * @param outputs AnyPIDOutputs to be set by the PIDController.
	 */
	public AcPIDDynamicControl(double delay, Check check, boolean shouldFinish, double p, double i, double d, double f, GettableNumber target, Tolerances tolerance, PIDSource input, boolean isContinuous, double minIn, double maxIn, PIDSourceType type, double minOut, double maxOut, PIDOutput... outputs) {
		super(delay, check, shouldFinish, p, i, d, f, 0, tolerance, input, isContinuous, minIn, maxIn, type, minOut, maxOut, outputs);
		this.dynamicTarget = target;
	}
	
	public void onRun() {
		target = dynamicTarget.get();
		for(PIDController controller : controllers) controller.setSetpoint(target);
	}
}
