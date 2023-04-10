package org.team1515.botmitzvah.Utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope {
    private final AHRS navx;
    private double offset = 0;

    public float rollOffset = 0;
    private float pitchOffset = 0;

    public Gyroscope() {
        navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    /**
     * resets gyro yaw to zero
     */
    public void zeroYaw() {
        navx.zeroYaw();
    }

    /**
     * @return float yaw of the robot in degrees
     */
    public float getYaw() {
        return navx.getYaw();
    }

    /**
     * @return float pitch of the robot in degrees
     */
    public float getPitch() {
        return navx.getPitch() - pitchOffset;
    }


    /**
     * @return float roll of the robot in degrees
     */
    public float getRoll() {
        return navx.getRoll() - rollOffset;
    }

    /**
     * Sets the robot's roll to zero
     */
    public void zeroRoll() {
        rollOffset = navx.getRoll();
    }

    public void zeroPitch(){
        pitchOffset = navx.getPitch();
    }
}
