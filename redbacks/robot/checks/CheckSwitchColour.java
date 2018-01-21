package redbacks.robot.checks;

import redbacks.arachne.lib.checks.Check;

public class CheckSwitchColour extends Check
{
	public int pos;

	public CheckSwitchColour(int pos) {
		this.pos = pos;
	}

	public boolean isDone() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		return gameData.charAt(pos) == 'L';
	}
}
