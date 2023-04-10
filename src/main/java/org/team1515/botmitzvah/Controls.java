package org.team1515.botmitzvah;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    public static final Trigger RESET_GYRO = new Trigger(RobotContainer.mainController::getBackButton);
    public static final Trigger DRIVE_ROBOT_ORIENTED = new Trigger(RobotContainer.mainController::getLeftBumper);
    
    public static boolean getLeftTrigger() {
        return RobotContainer.mainController.getLeftTriggerAxis() >= 0.250;
    }
}

// sticks for manual
// abx for set