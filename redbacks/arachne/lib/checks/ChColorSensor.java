package redbacks.arachne.lib.checks;

import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.sensors.ModernRoboticsColorSensor;

/**
 * A basic check for the Modern Robotics color sensor. As it's nigh on impossible to write a catch-all check condition,
 * an abstract method {@link #reachedColorThreshold} is included to make use easier.
 *
 * @author Sean Zammit
 */
public abstract class ChColorSensor extends Check
{
	public ModernRoboticsColorSensor colorSensor;
	
	public ChColorSensor(ModernRoboticsColorSensor colorSensor) {
		this.colorSensor = colorSensor;
	}
	
	public boolean isDone() {
		return reachedColorThreshold(colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue());
	}

	public abstract boolean reachedColorThreshold(double r, double g, double b);
}
