package frc.robot.subsystems.turret;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class HoodIOKraken implements HoodIO{
    private Kraken hoodKraken;

    public HoodIOKraken() {
        hoodKraken = new Kraken(TurretConstants.hoodKrakenID, UniversalConstants.canivoreName);
    }

}
