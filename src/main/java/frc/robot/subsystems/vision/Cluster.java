package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation3d;

public class Cluster {
    public Translation3d centerOfCluster;
    public int fuelCount;

    public Cluster(Translation3d centerOfCluster, int fuelCount) {
        this.centerOfCluster = centerOfCluster;
        this.fuelCount = fuelCount;
    }
}
