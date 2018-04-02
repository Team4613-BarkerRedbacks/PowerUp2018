package redbacks.robot.actions;

import static redbacks.arachne.ext.motion.MotionSettings.trajectoryMaxNegSpeed;
import static redbacks.arachne.ext.motion.MotionSettings.trajectoryMaxPosSpeed;

import redbacks.arachne.lib.logic.GettableNumber;
import redbacks.arachne.lib.sensors.NumericSensor;

/**
 * @author Sean Zammit
 */
public class AcStraight extends AcStraightFinishOnTarget
{
	public AcStraight(double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
		super(distance, angle, encoder, shouldReset, minOut, maxOut);
	}
	
	public AcStraight(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
		super(distance, angle, encoder, shouldReset);
	}

	public boolean startsLessThan;
	
	public void onStart() {
		super.onStart();
		
		startsLessThan = encoder.get() < path.totalDistance;
	}
	
	public boolean isDone() {
		return super.isDone() || (startsLessThan ? encoder.get() > path.totalDistance : encoder.get() < path.totalDistance);
	}
	
	public static class ChangeMinMax implements GettableNumber
	{
		public NumericSensor encoder;
		public int triggerDistance;
		public double newSpeed;
		
		public ChangeMinMax(NumericSensor encoder, int triggerDistance, double newSpeed) {
			this.encoder = encoder;
			this.triggerDistance = triggerDistance;
			this.newSpeed = newSpeed;
		}
		
		public double get() {
			return Math.abs(encoder.get()) > triggerDistance ? newSpeed : (newSpeed < 0 ? trajectoryMaxNegSpeed : trajectoryMaxPosSpeed);
		}
	}
	
	public static class ChangeMinMaxNeg extends ChangeMinMax
	{
		public ChangeMinMaxNeg(NumericSensor encoder, int triggerDistance, double newSpeed) {
			super(encoder, triggerDistance, newSpeed);
		}
		
		public double get() {
			return Math.abs(encoder.get()) < triggerDistance ? newSpeed : (newSpeed < 0 ? trajectoryMaxNegSpeed : trajectoryMaxPosSpeed);
		}
	}
}