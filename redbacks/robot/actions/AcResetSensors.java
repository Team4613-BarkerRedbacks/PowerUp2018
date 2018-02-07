package redbacks.robot.actions;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChTrue;
import redbacks.robot.Robot;

public class AcResetSensors extends Action
{
	public AcResetSensors() {
		super(new ChTrue());
	}
	
	public void onFinish() {
		Robot.sensors.resetSensors();
	}
}
