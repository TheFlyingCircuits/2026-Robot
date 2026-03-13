package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
        Logger.processInputs("intakeInputs", inputs);
        isIntakeDown = inputs.intakePositionDegrees < 15.0;
    }

    public boolean isIntakeDown() {
        return isIntakeDown;
    }

    public void intakeRunRollers() {
        io.setTargetRollerBottomVelocity(40.0);
        io.setTargetRollerTopVelocity(40.0);
    }

    public void inAutoIntake() {
        intakeDefault();
        io.setTargetRollerBottomVelocity(47.5);
        io.setTargetRollerTopVelocity(47.5);
    }

    public void intakeRollersStop() {
        io.setTargetRollerBottomVelocity(0.0);
        io.setTargetRollerTopVelocity(0.0);
    }

    public void reverseIntake() {
        io.setTargetRollerBottomVelocity(-45.0);
        io.setTargetRollerTopVelocity(-45.0);
    }

    public void intakeDown() {
        reverseIntake();
        if(inputs.intakePositionDegrees > 15.0) {
            isIntakeDown = false;
            io.setIntakeVolts(-8.0);
            // io.setTargetIntakePositionDegrees(0); 
        } else {
            isIntakeDown = true;
            intakeDefault();
        }
    }

    public void intakeDownThenIntake() {
        if(inputs.intakePositionDegrees > 15.0) {
            isIntakeDown = false;
            // io.setTargetIntakePositionDegrees(0);
            io.setIntakeVolts(-7.0);
            // reverseIntake();
        } else {
            isIntakeDown = true;
            if(inputs.intakePositionDegrees < 10.0) {
                intakeRunRollers();
            }
            intakeDefault();
        }
    }

    public void intakeDefualtAndIntake() {
        
        intakeRunRollers();
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(45);
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

    public Command inAutoIntakeCommand() {
        return this.run(() -> inAutoIntake());
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

    public Command reverseIntakeCommand() {
        return this.run(() -> reverseIntake());
    }

    public Command intakeDownThenIntakeCommand() {
        return this.run(() -> intakeDownThenIntake());
    }

    public Command intakeDefualtAndIntakeCommand() {
        return this.run(() -> intakeDefualtAndIntake());
    }

    public Command setAllVoltsCommand(Supplier<Double> topRollerVolts, Supplier<Double> bottomRollerVolts, Supplier<Double> pivotVolts) {
        return this.run(() -> setAllVolts(topRollerVolts.get(), bottomRollerVolts.get(), pivotVolts.get()));
    }

}

