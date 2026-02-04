package frc.robot.subsystems.turret.flywheels;

import frc.robot.Constants.TurretConstants;

public class FlywheelsIOSim implements FlywheelsIO{

    public FlywheelsIOSim() {

    }

    double frontWheelVelocityRPS = 0.0;
    double hoodWheelVelocityRPS = 0.0;

    
    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.frontWheelVelocityRPS = frontWheelVelocityRPS;
        inputs.frontWheelVelocityMPS = frontWheelVelocityRPS/(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        inputs.targetFrontWheelVelocityMPS = inputs.frontWheelVelocityMPS;

        inputs.hoodWheelVelocityRPS = hoodWheelVelocityRPS;
        inputs.hoodWheelVelocityMPS = hoodWheelVelocityRPS/(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);
        inputs.targetHoodWheelVelocityMPS = inputs.hoodWheelVelocityMPS;


    }

    @Override
    public void setFrontWheelVolts(double volts) {

    }

    @Override
    public void setFrontFollowerWheelVolts(double volts) {

    }


    @Override
    public void setHoodWheelVolts(double volts) {}

    @Override
    public void setTargetFrontWheelVelocity(double targetVelocityRPS) {
        frontWheelVelocityRPS = targetVelocityRPS;
    }

    @Override
    public void setTargetHoodWheelVelocity(double targetVelocityRPS) {
        hoodWheelVelocityRPS = targetVelocityRPS;
    }


}
