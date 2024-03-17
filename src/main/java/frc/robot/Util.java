package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public final class Util {
    private Util(){}
    public static boolean isNearPose(Pose2d p1,Pose2d p2,double tolerance,double headingTolerance) {
        boolean translationNear = isNearTranslation(p1.getTranslation(), p2.getTranslation(), tolerance);
        boolean headingNear = MathUtil.isNear(p1.getRotation().getDegrees(), p2.getRotation().getDegrees(), headingTolerance);
        return translationNear && headingNear;
    }
    public static boolean isNearTranslation(Translation2d t1, Translation2d t2,double tolerance) {
         boolean xNear = MathUtil.isNear(t1.getX(), t2.getX(), tolerance);
        boolean yNear = MathUtil.isNear(t1.getY(), t2.getY(), tolerance);
        return xNear && yNear;
    }
}
