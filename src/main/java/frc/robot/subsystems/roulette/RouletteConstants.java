
package frc.robot.subsystems.roulette;

import com.revrobotics.ColorSensorV3.RawColor;

public class RouletteConstants {
	public static final int kRouletteArmMotor = 26;
	public static final int kRotateRouletteMotor = 27;

	public static final RawColor kMinBlue = new RawColor(0, 0, 1500, 0);
	public static final RawColor kMaxBlue = new RawColor(1000, 10000, 3000, 1000);
	public static final RawColor kMinRed = new RawColor(1000, 0, 0, 0);
	public static final RawColor kMaxRed = new RawColor(3000, 10000, 900, 0);

	public static final double kRouletteProximity = 10;
	public static final double kArmOpenCloseWaitDuration = 0.2;
	public static final double kDeafultSpeed = 0.6;

	public static final int kMinSemiRotation = 8;
	public static final int kMaxSemiRotation = 9;

	public static final double kWaitingTime = 0.5;
}
