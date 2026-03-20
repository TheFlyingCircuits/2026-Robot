package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class Flywheeltest extends Command {
    private Turret turret;


    public Flywheeltest(Turret turret) {
        this.turret=turret;

        addRequirements(turret);
    }

    @Override 
    public void initialize() {
        System.out.println("------------------------------------------------------------------------------------------");
        turret.setHoodWheelSpeed(3.0);
    }

    @Override 
    public void execute() {
        System.out.println("running");
    }
}
