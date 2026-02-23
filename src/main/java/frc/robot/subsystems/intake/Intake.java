package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs;
    IntakeIO io;

    boolean isIntakeDown = false;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        isIntakeDown = inputs.intakePositionDegrees < 10.0;
    }

    public boolean isIntakeDown() {
        return isIntakeDown;
    }

    public void intakeRunRollers() {
        io.setTargetRollerBottomVelocity(5.0);
        io.setTargetRollerTopVelocity(5.0);
    }

    public void intakeRollersStop() {
        io.setTargetRollerBottomVelocity(0.0);
        io.setTargetRollerTopVelocity(0.0);
    }

    public void intakeDown() {
        if(inputs.intakePositionDegrees > 10.0) {
            isIntakeDown = false;
            io.setTargetIntakePositionDegrees(0); 
        } else {
            isIntakeDown = true;
            intakeDefault();
        }
    }

    public void intakeDownAndIntake() {
        intakeDown();
        intakeRunRollers();
    }

    public void intakeDefualtAndIntake() {
        intakeDefault();
        intakeRunRollers();
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(30);
    }

    public void intakeDefault() {
        io.setIntakeVolts(0);
    }

    public void rollerAndIntakeNoVolts() {
        intakeDefault();
        intakeRollersStop();
    }

    public void setAllVolts(double topRollerVolts, double bottomRollerVolts, double pivotVolts) {
        io.setRollerTopVolts(topRollerVolts);
        io.setRollerBottomVolts(bottomRollerVolts);
        io.setIntakeVolts(pivotVolts);
    }

    public Command noVoltageCommand() {
        return this.run(() -> rollerAndIntakeNoVolts());
    }

    public Command intakeDownCommand() {
        return this.run(() -> intakeDown());
    }
    
    public Command intakeUpCommand() {
        return this.run(() -> intakeUp());
    }

    public Command intakeRunRollersCommand() {
        return this.run(() -> intakeRunRollers());
    }

    public Command intakeDownAndIntakeCommand() {
        return this.run(() -> intakeDownAndIntake());
    }

    public Command intakeDefualtAndIntakeCommand() {
        return this.run(() -> intakeDefualtAndIntake());
    }

    public Command setAllVoltsCommand(Supplier<Double> topRollerVolts, Supplier<Double> bottomRollerVolts, Supplier<Double> pivotVolts) {
        return this.run(() -> setAllVolts(topRollerVolts.get(), bottomRollerVolts.get(), pivotVolts.get()));
    }

}

