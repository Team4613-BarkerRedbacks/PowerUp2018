package redbacks.robot.checks;

import redbacks.arachne.lib.checks.Check;
import redbacks.arachne.lib.sensors.ModernRoboticsColorSensor;

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
