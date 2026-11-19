package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

/**
 * {@link PoseSource} backed by the Pedro {@link Follower}, for autonomous. The follower already localizes off the
 * Pinpoint, so auto-aim can read its pose instead of opening a second reader on the same chip. The alliance is fixed
 * for the OpMode (Red / Blue), and the pose is always considered known once the OpMode has seeded the start pose.
 */
public class FollowerPoseSource implements PoseSource {

    private final Follower follower;
    private final Alliance alliance;

    public FollowerPoseSource(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
    }

    @Override
    public double getX() { return follower.pose().x(); }

    @Override
    public double getY() { return follower.pose().y(); }

    @Override
    public double getHeading() { return follower.pose().heading(); }

    @Override
    public Alliance getAlliance() { return alliance; }

    @Override
    public boolean isPoseKnown() { return alliance != null && follower.pose() != null; }
}
