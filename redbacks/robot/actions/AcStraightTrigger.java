package redbacks.robot.actions;

import redbacks.arachne.lib.logic.GettableNumber;
import redbacks.arachne.lib.sensors.NumericSensor;

/**
 * @author Sean Zammit
 */
public class AcStraightTrigger extends AcStraight
{
	public AcStraightTrigger(double distance, double angle, NumericSensor encoder, boolean shouldReset, GettableNumber minOut, GettableNumber maxOut) {
		super(distance, angle, encoder, shouldReset, minOut, maxOut);
	}
	
	public AcStraightTrigger(double distance, double angle, NumericSensor encoder, boolean shouldReset) {
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
}