package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.util.Dampener;

public class TeleopDrive extends Command {
    private final DriveTrain driveTrain;

    private final XboxController controller;

    private final int X_AXIS;
    private final int Y_AXIS;

    private Dampener xDampener;
    private Dampener yDampener;

    public TeleopDrive(DriveTrain driveTrain, XboxController controller, int fwdRevAxis, int leftRightAxis) {
        this.driveTrain = driveTrain;
        this.controller = controller;

        this.X_AXIS = leftRightAxis;
        this.Y_AXIS = fwdRevAxis;

        this.xDampener = new Dampener(DriveConstants.CONTROLLER_DEADZONE, 6);
        this.yDampener = new Dampener(DriveConstants.CONTROLLER_DEADZONE, 4);
    }

    @Override
    public void execute() {
        double y = yDampener.dampen(y) * controller.getRawAxis(Y_AXIS);
        double x = xDampener.dampen(x) * controller.getRawAxis(X_AXIS);

        driveTrain.arcadeDrive(y * 0.85, x * 0.25);
    }
}