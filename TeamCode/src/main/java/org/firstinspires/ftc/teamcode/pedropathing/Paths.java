package org.firstinspires.ftc.teamcode.pedropathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private static final PoseFactory redPoseFactory = PoseFactory.degrees();

    public static final Pose POSE_START_RED_GARDEN = redPoseFactory.of(56, 8, 90);
    public static final Pose POSE_SHOOT_RED_GARDEN = redPoseFactory.of(56, 12, 90);
    public static final Pose POSE_PICKUP_RED_GARDEN = redPoseFactory.of(9, 9, -90);
    public static final Pose POSE_PICKUP_RED_GARDEN_FLOWER = redPoseFactory.of(9, 40, 90);
    public static final Pose POSE_PARK_RED_GARDEN = redPoseFactory.of(12, 88, 90);

    public static final Pose POSE_START_RED_FLOWER = redPoseFactory.of(56, 132.5, -90);
    public static final Pose POSE_SHOOT_RED_FLOWER = redPoseFactory.of(56, 129.5, -90);
    public static final Pose POSE_PICKUP_RED_FLOWER = redPoseFactory.of(38, 132.5, 0);
    public static final Pose POSE_PARK_RED_FLOWER = redPoseFactory.of(10, 119.7512, 11.9656);
    
    private static final PoseFactory bluePoseFactory = PoseFactory.degrees().mirrorX(70.75);

    public static final Pose POSE_START_BLUE_GARDEN = bluePoseFactory.of(56, 8, 90);
    public static final Pose POSE_SHOOT_BLUE_GARDEN = bluePoseFactory.of(56, 12, 90);
    public static final Pose POSE_PICKUP_BLUE_GARDEN = bluePoseFactory.of(9, 9, -90);
    public static final Pose POSE_PICKUP_BLUE_GARDEN_FLOWER_LEFT = bluePoseFactory.of(9, 40, 90);
    public static final Pose POSE_PARK_BLUE_GARDEN = bluePoseFactory.of(12, 88, 90);

    public static final Pose POSE_START_BLUE_FLOWER = bluePoseFactory.of(56, 132.5, -90);
    public static final Pose POSE_SHOOT_BLUE_FLOWER = bluePoseFactory.of(56, 129.5, -90);
    public static final Pose POSE_PICKUP_BLUE_FLOWER = bluePoseFactory.of(38, 132.5, 0);
    public static final Pose POSE_PARK_BLUE_FLOWER = bluePoseFactory.of(10, 119.7512, 11.9656);

    public static class RedGardenPaths {

        private static final Pose pickupGardenRedControl1 = redPoseFactory.of(34.5, 48, 0);
        private static final Pose pickupFlowerGardenControl1 = redPoseFactory.of(24, 18, 0);
        private static final Pose shootToParkControl1 = redPoseFactory.of(12, 24, 0);

        public static Path startToShoot() {
            return line(POSE_START_RED_GARDEN, POSE_SHOOT_RED_GARDEN).linear(POSE_START_RED_GARDEN, POSE_SHOOT_RED_GARDEN);
        }

        public static Path pickupGardenRed() {
            return curve(POSE_SHOOT_RED_GARDEN, pickupGardenRedControl1, POSE_PICKUP_RED_GARDEN).linear(POSE_SHOOT_RED_GARDEN, POSE_PICKUP_RED_GARDEN);
        }

        public static Path gardenToShoot() {
            return line(POSE_PICKUP_RED_GARDEN, POSE_SHOOT_RED_GARDEN).constant(POSE_SHOOT_RED_GARDEN);
        }

        public static Path pickupFlowerGarden() {
            return curve(POSE_SHOOT_RED_GARDEN, pickupFlowerGardenControl1, POSE_PICKUP_RED_GARDEN_FLOWER).linear(POSE_SHOOT_RED_GARDEN, POSE_PICKUP_RED_GARDEN_FLOWER);
        }

        public static Path flowerGardenToShoot() {
            return line(POSE_PICKUP_RED_GARDEN_FLOWER, POSE_SHOOT_RED_GARDEN).constant(POSE_SHOOT_RED_GARDEN);
        }

        public static Path shootToPark() {
            return curve(POSE_SHOOT_RED_GARDEN, shootToParkControl1, POSE_PARK_RED_GARDEN).constant(POSE_PARK_RED_GARDEN);
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
