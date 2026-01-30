package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public class AdvantageScopeDrawingUtils {

    /**<pre>&nbsp;
     *       0------1
     *      /|     /|
     *     3-+----2 |
     *     | |    | |
     *   Z | 4----+-5
     *     |/     |/ X
     *     7------6   
     *        Y
     *</pre>
     * Positive X goes from 6 to 5.
     * Positive Y goes from 6 to 7.
     * Positive Z goes from 6 to 2.
     */
    private static List<Translation3d> getBoxCorners(double frontX, double backX, double leftY, double rightY, double topZ, double bottomZ) {
        List<Translation3d> corners = new ArrayList<>(8);
        // top half
        corners.add(new Translation3d(frontX, leftY,  topZ));
        corners.add(new Translation3d(frontX, rightY, topZ));
        corners.add(new Translation3d(backX,  rightY, topZ));
        corners.add(new Translation3d(backX,  leftY,  topZ));

        // bottom half
        corners.add(new Translation3d(frontX, leftY,  bottomZ));
        corners.add(new Translation3d(frontX, rightY, bottomZ));
        corners.add(new Translation3d(backX,  rightY, bottomZ));
        corners.add(new Translation3d(backX,  leftY,  bottomZ));

        return corners;
    }
    private static void drawBox(String nameInLog, List<Translation3d> boxCorners_boxFrame, Pose3d boxPose_fieldFrame) {
        // 1) Get the coords of the box corners in the field frame
        List<Translation3d> boxCorners_fieldFrame = new ArrayList<>();
        for (Translation3d boxCorner_boxFrame : boxCorners_boxFrame) {
            Translation3d boxCorner_fieldFrame = boxCorner_boxFrame.rotateBy(boxPose_fieldFrame.getRotation()).plus(boxPose_fieldFrame.getTranslation());
            boxCorners_fieldFrame.add(boxCorner_fieldFrame);
        }


        // 2) Connect the dots in the field frame
        List<Translation3d> pencilTrajectory = new ArrayList<>();

        //      0------1
        //     /|     /|
        //    3-+----2 |
        //    | |    | |
        //    | 4----+-5
        //    |/     |/ 
        //    7------6   

        // top half
        pencilTrajectory.add(boxCorners_fieldFrame.get(0));
        pencilTrajectory.add(boxCorners_fieldFrame.get(1));
        pencilTrajectory.add(boxCorners_fieldFrame.get(2));
        pencilTrajectory.add(boxCorners_fieldFrame.get(3));
        pencilTrajectory.add(boxCorners_fieldFrame.get(0)); // close the loop

        // bottom half, returning back up at each corner for the vertical edges
        pencilTrajectory.add(boxCorners_fieldFrame.get(4));

        pencilTrajectory.add(boxCorners_fieldFrame.get(5)); // 5 <-> 1
        pencilTrajectory.add(boxCorners_fieldFrame.get(1));
        pencilTrajectory.add(boxCorners_fieldFrame.get(5));

        pencilTrajectory.add(boxCorners_fieldFrame.get(6)); // 6 <-> 2
        pencilTrajectory.add(boxCorners_fieldFrame.get(2));
        pencilTrajectory.add(boxCorners_fieldFrame.get(6));

        pencilTrajectory.add(boxCorners_fieldFrame.get(7)); // 7 <-> 3
        pencilTrajectory.add(boxCorners_fieldFrame.get(3));
        pencilTrajectory.add(boxCorners_fieldFrame.get(7));

        pencilTrajectory.add(boxCorners_fieldFrame.get(4));
        pencilTrajectory.add(boxCorners_fieldFrame.get(0)); // back home to finish the loop

        Logger.recordOutput(nameInLog, pencilTrajectory.toArray(new Translation3d[0]));
    }

    public static void eraseDrawing(String name) {
        Logger.recordOutput(name, new Translation3d[0]);
    }

    public static void drawCircle(String name, Translation2d center, double radius) {
        // https://www.desmos.com/calculator/mzf8vovsmd
        double acceptableVisualDeviationMeters = 0.01;
        double inputToArccos = (radius - acceptableVisualDeviationMeters) / (radius + acceptableVisualDeviationMeters);
        int numVertices = (int)Math.ceil(Math.PI / Math.acos(inputToArccos));
        if (numVertices < 8) {
            numVertices = 8;
        }
        double radsBetweenVertices = (2.0 * Math.PI) / numVertices;

        List<Translation2d> points = new ArrayList<>();
        for (int i = 0; i < numVertices; i += 1) {
            double rads = radsBetweenVertices * i;
            Translation2d radiusVector = new Translation2d(radius * Math.cos(rads), radius * Math.sin(rads));
            points.add(center.plus(radiusVector));
        }

        // close the loop
        points.add(points.get(0));

        Logger.recordOutput(name, points.toArray(new Translation2d[0]));
    }

    private static List<Translation3d> getBumperBox_robotFrame() {
        // our bumpers are square, so their length in X and Y is the same
        double halfBumperLength = DrivetrainConstants.bumperWidthMeters/2.0;
        double bumperThicknessZ = Units.inchesToMeters(5); // from the bottom surface of the bumper to the top surface of the bumper.
        double groundToBumper = Units.inchesToMeters(0.5); // from the surface of the ground to the bottom surface of the bumper.
        return getBoxCorners(halfBumperLength, -halfBumperLength,
                             halfBumperLength, -halfBumperLength,
                             groundToBumper+bumperThicknessZ, groundToBumper);
    }
    public static void drawBumpers(String nameInLog, Pose2d robotPose) {
        drawBox(nameInLog, getBumperBox_robotFrame(), new Pose3d(robotPose));
    }
}
