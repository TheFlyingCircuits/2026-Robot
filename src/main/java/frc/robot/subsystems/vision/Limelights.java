package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.List;

public class Limelights {
    public ArrayList<String> camNames = new ArrayList<String>();

    public Limelights(ArrayList<String> camNames) {
        this.camNames=camNames;
    }

    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingTrig, double robotAngleDeg) {
        return new ArrayList<>();
    }

}
