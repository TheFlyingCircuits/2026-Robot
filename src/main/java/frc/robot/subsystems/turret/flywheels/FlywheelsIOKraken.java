package frc.robot.subsystems.turret.flywheels;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class FlywheelsIOKraken implements FlywheelsIO {
    private Kraken frontWheelKraken;
    private Kraken frontWheelKrakenFollower;
    private Kraken hoodWheelKraken;

    public FlywheelsIOKraken() {
        frontWheelKraken = new Kraken(TurretConstants.frontWheelKrakenID, UniversalConstants.canivoreName);
        frontWheelKrakenFollower = new Kraken(TurretConstants.frontWheelFollowerKrakenID, UniversalConstants.canivoreName);
        hoodWheelKraken = new Kraken(TurretConstants.hoodWheelKrakenID, UniversalConstants.canivoreName);
    }
    
}
