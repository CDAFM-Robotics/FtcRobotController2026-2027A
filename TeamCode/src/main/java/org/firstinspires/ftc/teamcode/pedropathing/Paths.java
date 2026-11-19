package org.firstinspires.ftc.teamcode.pedropathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private static final PoseFactory redPoseFactory = PoseFactory.degrees();

    // Nov 16: measured Red Garden start spot (was 56, 8, 90). Heading 150.98 = 180 - 29.02 from the start geometry.
    public static final Pose POSE_START_RED_GARDEN = redPoseFactory.of(32.07, 12.37, 150.98);
    public static final Pose POSE_SHOOT_RED_GARDEN = redPoseFactory.of(32.07, 12.37, 150.98);   // Nov 16: shoot from the start spot (was 56, 12, 90)
    // Backed off the Garden corner so the fully-extended intake (~5 in) clears the walls: faces -90 (toward the audience
    // wall y = 0), so y 15 leaves the intake tip ~1 in off that wall, and x 12 keeps the robot's left side off the red wall.
    public static final Pose POSE_PICKUP_RED_GARDEN = redPoseFactory.of(12, 15, -90);   // was 9, 9
    public static final Pose POSE_PICKUP_RED_GARDEN_FLOWER = redPoseFactory.of(9, 40, 90);
    public static final Pose POSE_PARK_RED_GARDEN = redPoseFactory.of(12, 84, 90);   // finish moved 4 in -y (was 12, 88)

    // Red Flower start/shoot: front-left robot corner (18x18) on the midpoint of the 2nd-tile-from-red-wall's far-wall
    // edge (35.375, 141.5), rotated so the Pollen launcher (6.148 in behind center, fires right) points at the far CELL
    // (58, 86.5). Solving 22.625*cos h - 55*sin h = -15.148 gives h = 37.11 deg, center (33.63, 128.89). Shoot = start spot.
    public static final Pose POSE_START_RED_FLOWER = redPoseFactory.of(33.63, 128.89, 37.11);
    public static final Pose POSE_SHOOT_RED_FLOWER = redPoseFactory.of(33.63, 128.89, 37.11);
    public static final Pose POSE_PICKUP_RED_FLOWER = redPoseFactory.of(38, 132.5, 0);
    public static final Pose POSE_PARK_RED_FLOWER = redPoseFactory.of(14, 117.7512, 11.9656);   // moved +4 x, -2 y (was 10, 119.7512)
    
    private static final PoseFactory bluePoseFactory = PoseFactory.degrees().mirrorX(70.75);

    public static final Pose POSE_START_BLUE_GARDEN = bluePoseFactory.of(56, 8, 90); //TODO fix all blue
    public static final Pose POSE_SHOOT_BLUE_GARDEN = bluePoseFactory.of(56, 12, 90);
    public static final Pose POSE_PICKUP_BLUE_GARDEN = bluePoseFactory.of(10, 11, -90);
    public static final Pose POSE_PICKUP_BLUE_GARDEN_FLOWER_LEFT = bluePoseFactory.of(9, 40, 90);
    public static final Pose POSE_PARK_BLUE_GARDEN = bluePoseFactory.of(12, 88, 90);

    public static final Pose POSE_START_BLUE_FLOWER = bluePoseFactory.of(56, 132.5, -90);
    public static final Pose POSE_SHOOT_BLUE_FLOWER = bluePoseFactory.of(56, 129.5, -90);
    public static final Pose POSE_PICKUP_BLUE_FLOWER = bluePoseFactory.of(38, 132.5, 0);
    public static final Pose POSE_PARK_BLUE_FLOWER = bluePoseFactory.of(10, 119.7512, 11.9656);

    // --- Shoot-four-then-park autos (Red/Blue Near/Far). Blue = 180-deg rotation of the Red inputs about the field
    //     center (70.75, 70.75): feeding each Blue pose the Red numbers gives the rotated Blue pose. ---
    private static final PoseFactory blue180 = PoseFactory.degrees().rotateAround(70.75, 70.75, 180);

    public static final Pose POSE_SHOOT_BLUE_NEAR = blue180.of(32.07, 12.37, 150.98);   // 180-rot of POSE_SHOOT_RED_GARDEN
    public static final Pose POSE_PARK_BLUE_NEAR  = blue180.of(12, 84, 90);             // 180-rot of POSE_PARK_RED_GARDEN
    public static final Pose POSE_SHOOT_BLUE_FAR  = blue180.of(33.63, 128.89, 37.11);   // 180-rot of POSE_SHOOT_RED_FLOWER
    public static final Pose POSE_PARK_BLUE_FAR   = blue180.of(14, 117.7512, 11.9656);  // 180-rot of POSE_PARK_RED_FLOWER

    public static class RedGardenPaths {

        // Shifted with the new shoot spot (start x 56 -> 32): the old control sat to +x of the new start, so the curve
        // bulged out to the right against the robot's up-left (150.98 deg) facing. Moving it to x 10.5 leaves the shoot
        // pose heading up-left and keeps the upward sweep (peak ~y 29) that collects the garden balls. (was 34.5, 48)
        private static final Pose pickupGardenRedControl1 = redPoseFactory.of(10.5, 48, 0);
        private static final Pose pickupFlowerGardenControl1 = redPoseFactory.of(24, 18, 0);
        private static final Pose shootToParkControl1 = redPoseFactory.of(12, 24, 0);

//        public static Path startToShoot() {
//            return line(POSE_START_RED_GARDEN, POSE_SHOOT_RED_GARDEN).linear(POSE_START_RED_GARDEN, POSE_SHOOT_RED_GARDEN);
//        }
//        No longer needed as POSE_START_RED_GARDEN and POSE_SHOOT_RED_GARDEN are equivalent

        public static Path pickupGardenRed() {
            return curve(POSE_SHOOT_RED_GARDEN, pickupGardenRedControl1, POSE_PICKUP_RED_GARDEN).linear(POSE_SHOOT_RED_GARDEN, POSE_PICKUP_RED_GARDEN);
        }

        public static Path gardenToShoot() {
            return line(POSE_PICKUP_RED_GARDEN, POSE_SHOOT_RED_GARDEN).linear(POSE_PICKUP_RED_GARDEN, POSE_SHOOT_RED_GARDEN);
        }

        public static Path pickupFlowerGarden() {
            return curve(POSE_SHOOT_RED_GARDEN, pickupFlowerGardenControl1, POSE_PICKUP_RED_GARDEN_FLOWER).linear(POSE_SHOOT_RED_GARDEN, POSE_PICKUP_RED_GARDEN_FLOWER);
        }

        public static Path flowerGardenToShoot() {
            return line(POSE_PICKUP_RED_GARDEN_FLOWER, POSE_SHOOT_RED_GARDEN).linear(POSE_PICKUP_RED_GARDEN_FLOWER, POSE_SHOOT_RED_GARDEN);
        }

        public static Path shootToPark() {
            return curve(POSE_SHOOT_RED_GARDEN, shootToParkControl1, POSE_PARK_RED_GARDEN).constant(POSE_SHOOT_RED_GARDEN);
        }
    }
    public static class RedFlowerPaths {
        
        private final Pose pickupRedFlowerControl1 = redPoseFactory.of(47, 117.5, 0);
        private final Pose redFlowerToShootControl1 = redPoseFactory.of(47, 117.5, 0);
        
        public Path startToShoot() {
            return line(POSE_START_RED_FLOWER, POSE_SHOOT_RED_FLOWER).linear(POSE_START_RED_FLOWER, POSE_SHOOT_RED_FLOWER);
        }

        public Path pickupRedFlower() {
            return curve(POSE_SHOOT_RED_FLOWER, pickupRedFlowerControl1, POSE_PICKUP_RED_FLOWER).linear(POSE_SHOOT_RED_FLOWER, POSE_PICKUP_RED_FLOWER);
        }

        public Path redFlowerToShoot() {
            return curve(POSE_PICKUP_RED_FLOWER, redFlowerToShootControl1, POSE_SHOOT_RED_FLOWER).linear(POSE_PICKUP_RED_FLOWER, POSE_SHOOT_RED_FLOWER);
        }

        public Path shootToPark() {
            return line(POSE_SHOOT_RED_FLOWER, POSE_PARK_RED_FLOWER).reverseTangent();
        }
    }
    public static class BlueGardenPaths {

        private static final Pose pickupGardenBlueControl1 = bluePoseFactory.of(34.5, 48, 0);
        private static final Pose pickupFlowerGardenControl1 = bluePoseFactory.of(24, 18, 0);
        private static final Pose shootToParkControl1 = bluePoseFactory.of(12, 24, 0);

        public static Path startToShoot() {
            return line(POSE_START_BLUE_GARDEN, POSE_SHOOT_BLUE_GARDEN).linear(POSE_START_BLUE_GARDEN, POSE_SHOOT_BLUE_GARDEN);
        }

        public static Path pickupGardenBlue() {
            return curve(POSE_SHOOT_BLUE_GARDEN, pickupGardenBlueControl1, POSE_PICKUP_BLUE_GARDEN).linear(POSE_SHOOT_BLUE_GARDEN, POSE_PICKUP_BLUE_GARDEN);
        }

        public static Path gardenToShoot() {
            return line(POSE_PICKUP_BLUE_GARDEN, POSE_SHOOT_BLUE_GARDEN).constant(POSE_SHOOT_BLUE_GARDEN);
        }

        public static Path pickupFlowerGarden() {
            return curve(POSE_SHOOT_BLUE_GARDEN, pickupFlowerGardenControl1, POSE_PICKUP_BLUE_GARDEN_FLOWER_LEFT).linear(POSE_SHOOT_BLUE_GARDEN, POSE_PICKUP_BLUE_GARDEN_FLOWER_LEFT);
        }

        public static Path flowerGardenToShoot() {
            return line(POSE_PICKUP_BLUE_GARDEN_FLOWER_LEFT, POSE_SHOOT_BLUE_GARDEN).constant(POSE_SHOOT_BLUE_GARDEN);
        }

        public static Path shootToPark() {
            return curve(POSE_SHOOT_BLUE_GARDEN, shootToParkControl1, POSE_PARK_BLUE_GARDEN).constant(POSE_PARK_BLUE_GARDEN);
        }
    }
    public static class BlueFlowerPaths {
        
        private final Pose pickupBlueFlowerControl1 = bluePoseFactory.of(47, 117.5, 0);
        private final Pose blueFlowerToShootControl1 = bluePoseFactory.of(47, 117.5, 0);

        public Path startToShoot() {
            return line(POSE_START_BLUE_FLOWER, POSE_SHOOT_BLUE_FLOWER).linear(POSE_START_BLUE_FLOWER, POSE_SHOOT_BLUE_FLOWER);
        }

        public Path pickupBlueFlower() {
            return curve(POSE_SHOOT_BLUE_FLOWER, pickupBlueFlowerControl1, POSE_PICKUP_BLUE_FLOWER).linear(POSE_SHOOT_BLUE_FLOWER, POSE_PICKUP_BLUE_FLOWER);
        }

        public Path blueFlowerToShoot() {
            return curve(POSE_PICKUP_BLUE_FLOWER, blueFlowerToShootControl1, POSE_SHOOT_BLUE_FLOWER).linear(POSE_PICKUP_BLUE_FLOWER, POSE_SHOOT_BLUE_FLOWER);
        }

        public Path shootToPark() {
            return line(POSE_SHOOT_BLUE_FLOWER, POSE_PARK_BLUE_FLOWER).reverseTangent();
        }
    }

}
