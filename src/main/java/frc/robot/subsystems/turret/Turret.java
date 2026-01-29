package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.aimer.AimerIO;
import frc.robot.subsystems.turret.flywheels.FlywheelsIO;
import frc.robot.subsystems.turret.hood.HoodIO;

public class Turret extends SubsystemBase{

    AimerIOInputsAutoLogged aimerInputs;
    FlywheelsIOInputsAutoLogged flywheelsInputs;
    HoodIOInputsAutoLogged hoodInputs;

    AimerIO aimer;
    FlywheelsIO flywheels;
    HoodIO hood;

    public Turret(AimerIO aimer, FlywheelsIO flywheels, HoodIO hood) {
        this.aimer = aimer;
        this.flywheels = flywheels;
        this.hood = hood;

        aimerInputs = new AimerIOInputsAutoLogged();
        flywheelsInputs = new FlywheelsIOInputsAutoLogged();
        hoodInputs = new HoodIOInputsAutoLogged();
    }

    @Override 
    public void periodic() {
        aimer.updateInputs(aimerInputs);
        Logger.processInputs("aimerInputs", aimerInputs);
    }

    public void aimAtTarget(double targetAngleDegrees) {
        Logger.recordOutput("aimerInputs/targetAngleDegrees", targetAngleDegrees);
        aimer.setTargetAimerPosition(targetAngleDegrees);
    }

    public void setAimerVolts(double volts) {
        aimer.setAimerVolts(volts);
    }

    public Command aimAtTargetCommand(DoubleSupplier targetAngleDegrees) {
        return this.run(() -> aimAtTarget(targetAngleDegrees.getAsDouble()));
    }
        public Command setAimerVoltsCommand(DoubleSupplier volts) {
        return this.run(() -> setAimerVolts(volts.getAsDouble()));
    }
}
