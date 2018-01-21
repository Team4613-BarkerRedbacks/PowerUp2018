package redbacks.robot.checks;

import edu.wpi.first.wpilibj.DriverStation;
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
