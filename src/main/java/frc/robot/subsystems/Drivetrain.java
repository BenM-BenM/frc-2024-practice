package frc.robot.subsystems;

import frc.robot.constants.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class Drivetrain extends SubsystemBase {
    private final TalonSRX rightMaster = new TalonSRX(CANBus.RIGHT_MASTER);
    private final TalonSRX leftMaster = new TalonSRX(CANBus.LEFT_MASTER);
    private final TalonSRX rightFollower = new TalonSRX(CANBus.RIGHT_FOLLOWER);
    private final TalonSRX leftFollower = new TalonSRX(CANBus.LEFT_FOLLOWER);

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotationLimiter =  new SlewRateLimiter(2);

    private double targetTranslation;
    private double targetRotation;


    public Drivetrain(){
        this.rightFollower.follow(this.rightMaster);
        this.leftFollower.follow(this.leftMaster);

        this.rightMaster.setInverted(true);
    }
    
    public void periodic() {
        double translation = translationLimiter.calculate(targetTranslation);
        double rotation = rotationLimiter.calculate(targetRotation);

        double left = (translation + rotation);
        double right = (translation - rotation);

        setMotors(left, right);
    }

    private void setMotors(double left, double right) {
        leftMaster.set(TalonSRXControlMode.PercentOutput, left);
        rightMaster.set(TalonSRXControlMode.PercentOutput, right);
    }

    public void arcadeDrive(double translation, double rotation) {
        targetTranslation = translation;
        targetRotation = rotation;
    }
} 