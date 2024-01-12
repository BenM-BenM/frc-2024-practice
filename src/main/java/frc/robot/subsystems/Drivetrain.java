package frc.robot.subsystems;

import frc.robot.constants.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Drivetrain extends SubsystemBase {
    private final VictorSP rightMaster = new VictorSP(CANBus.RIGHT_MASTER);
    private final VictorSP leftMaster = new VictorSP(CANBus.LEFT_MASTER);
    private final VictorSP rightFollower = new VictorSP(CANBus.RIGHT_FOLLOWER);
    private final VictorSP leftFollower = new VictorSP(CANBus.LEFT_FOLLOWER);

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotationLimiter =  new SlewRateLimiter(2);

    private double targetTranslation;
    private double targetRotation;


    public Drivetrain(){
        this.rightMaster.addFollower(this.rightFollower);
        this.leftMaster.addFollower(this.leftFollower);

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
        leftMaster.set(left);
        rightMaster.set(right);
    }

    public void arcadeDrive(double translation, double rotation) {
        targetTranslation = translation;
        targetRotation = rotation;
    }
} 