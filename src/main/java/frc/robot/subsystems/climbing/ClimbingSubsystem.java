package frc.robot.subsystems.climbing;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
    private static ClimbingSubsystem instance;

    public static ClimbingSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimbingSubsystem();
        }
        return instance;
    }

    private TalonFX pulleyLeftMotor, pulleyRightMotor;
    private TalonSRX scissorsLeftMotor, scissorsRightMotor;
    private Encoder scissorsLeftEncoder, scissorsRightEncoder;

    private ClimbingSubsystem() {
        this.pulleyLeftMotor = new TalonFX(ClimbingConstants.kPulleyLeftMotorID);
        this.pulleyRightMotor = new TalonFX(ClimbingConstants.kPulleyRightMotorID);
        this.pulleyRightMotor.setInverted(true);

        this.scissorsLeftMotor = new TalonSRX(ClimbingConstants.kScissorsLeftMotorID);
        this.scissorsRightMotor = new TalonSRX(ClimbingConstants.kScissorsRightMotorID);
        this.scissorsRightMotor.setInverted(true);

        this.scissorsLeftEncoder = new Encoder(ClimbingConstants.kScissorsEncoderLeftChannelA,
                ClimbingConstants.kScissorsEncoderLeftChannelB);

        this.scissorsRightEncoder = new Encoder(ClimbingConstants.kScissorsEncoderRightChannelA,
                ClimbingConstants.kScissorsEncoderRightChannelB);
    }

    public void setPulleyLeft(double speed) {
        this.pulleyLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setPulleyRight(double speed) {
        this.pulleyRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setScissorsLeft(double speed) {
        this.scissorsLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setScissorsRight(double speed) {
        this.scissorsRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public int getScissorsEncoderLeft() {
        return this.scissorsLeftEncoder.get();
    }

    public int getScissorsEncoderRight() {
        return this.scissorsRightEncoder.get();
    }

    public void resetEncoderLeft() {
        scissorsLeftEncoder.reset();
    }

    public void resetEncoderRight() {
        scissorsRightEncoder.reset();
    }
}
