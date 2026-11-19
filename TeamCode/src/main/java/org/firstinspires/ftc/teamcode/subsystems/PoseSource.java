package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

/**
 * The minimal field pose + alliance that {@link AutoAimSubsystem} needs. Coordinates are Pedro field inches and the
 * heading is radians (0 = +x, counterclockwise positive), the same frame Paths and the Follower use.
 *
 * TeleOp supplies this through {@link LocalizerSubsystem} (its own Pinpoint); autonomous supplies it through
 * {@link FollowerPoseSource}, which reads the Pedro Follower's pose so there is no second Pinpoint reader.
 */
public interface PoseSource {
    double getX();
    double getY();
    double getHeading();
    Alliance getAlliance();

    /** Position, heading and alliance all known: what auto-aim needs before it produces a velocity. */
    boolean isPoseKnown();
}
