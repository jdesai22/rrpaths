import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.5

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPose = Pose2d(-62.0, -25.0, 0.0.toRadians)
    private val secondPose = Pose2d(-62.0, -20.0, 0.0.toRadians)
    private val launchPose = Pose2d(0.0, -40.0, 0.0.toRadians);

    private val ZoneAPose = Pose2d(15.0, -40.0, 0.0)
    private val ZoneBPose1 = Pose2d(43.0, -20.0, 0.0)
    private val ZoneBPose2 = Pose2d(0.0, -20.0, 0.0)
    private val ZoneBPose3 = Pose2d(0.0, -37.0, 0.0)
    private val ZoneCPose = Pose2d(62.0, -40.0, 0.0)
    private val ZoneCPose1 = Pose2d(-30.0, -37.0, 0.0)
    private val ZoneCPose2 = Pose2d(0.0, -40.0, 0.0)

    private val PreWobble2 = Pose2d(-50.0, -35.0, 0.0)

    private val FinalZoneA = Pose2d(10.0, -40.0, 0.0)
    private val FinalZoneB = Pose2d(36.0, -20.0, 0.0)
    private val FinalZoneC = Pose2d(55.0, -40.0, 0.0)

    private val RingNumber = 1;

//    0 = A
//    1 = B
//    4 = C

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val AvoidRings = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
        val MoveToLaunchArea = TrajectoryBuilder(secondPose, secondPose.heading, combinedConstraints)

//        deliver 1st wobble goal
        val ZoneA = TrajectoryBuilder(launchPose, launchPose.heading, combinedConstraints)
        val ZoneB = TrajectoryBuilder(launchPose, launchPose.heading, combinedConstraints)
        val ZoneC = TrajectoryBuilder(launchPose, launchPose.heading, combinedConstraints)

//        pick up extra rings, if possible, and then setup for 2nd wobble goal
        val ZoneAReturn = TrajectoryBuilder(ZoneAPose, ZoneAPose.heading, combinedConstraints)
        val ZoneBReturn1 = TrajectoryBuilder(ZoneBPose1, ZoneBPose1.heading, combinedConstraints)
        val ZoneBReturn2 = TrajectoryBuilder(ZoneBPose2, ZoneBPose2.heading, combinedConstraints)
        val ZoneBReturn3 = TrajectoryBuilder(ZoneBPose3, ZoneBPose3.heading, combinedConstraints)
        val ZoneCReturn = TrajectoryBuilder(ZoneCPose, ZoneCPose.heading, combinedConstraints)
        val ZoneCReturn1 = TrajectoryBuilder(ZoneCPose1, ZoneCPose1.heading, combinedConstraints)
        val ZoneCReturn2 = TrajectoryBuilder(ZoneCPose2, ZoneCPose2.heading, combinedConstraints)

//        deliver final wobble goal
        val ZoneA1 = TrajectoryBuilder(PreWobble2, PreWobble2.heading, combinedConstraints)
        val ZoneB1 = TrajectoryBuilder(PreWobble2, PreWobble2.heading, combinedConstraints)
        val ZoneC1 = TrajectoryBuilder(PreWobble2, PreWobble2.heading, combinedConstraints)

//        final park
        val ParkFromA = TrajectoryBuilder(FinalZoneA, FinalZoneA.heading, combinedConstraints)
        val ParkFromB = TrajectoryBuilder(FinalZoneB, FinalZoneB.heading, combinedConstraints)
        val ParkFromC = TrajectoryBuilder(FinalZoneC, FinalZoneC.heading, combinedConstraints)

//        START OF ACTUAL STUFF

//      detect number of rings

//      avoid rings
        AvoidRings.strafeLeft(5.0);

//        move to launch area
        MoveToLaunchArea.forward(40.0);
        MoveToLaunchArea.splineTo(Vector2d(0.0, -40.0), 0.0)

//        shoot 3 rings

//        move to a zone
        ZoneA.splineTo(Vector2d(15.0, -40.0), 0.0)
        ZoneB.splineTo(Vector2d(43.0, -20.0), 0.0)
        ZoneC.splineTo(Vector2d(62.0, -40.0), 0.0)

//        drop wobble goal

//        move back to pick up other wobble goal ring
//        in real thing add at set reversed here
        ZoneAReturn
            .splineTo(Vector2d(0.0, -40.0), 180.0.toRadians)
//            .splineTo(Vector2d(-20.0, -20.0), 180.0.toRadians)
            .splineTo(Vector2d(-50.0, -35.0), 180.0.toRadians)


        ZoneBReturn1
            .splineTo(Vector2d(-0.0, -20.0), 180.0.toRadians)

        ZoneBReturn2
            .strafeRight(17.0)

//      picking up rings

        ZoneBReturn3
//                find a way to slow down here and pick up rings
            .back(20.0)
            .back(10.0)

//            run the intake

            .splineTo(Vector2d(-50.0, -35.0), 180.0.toRadians)



        ZoneCReturn
            .splineTo(Vector2d(0.0, -37.0), 180.0.toRadians)
//            .back(30.0)
            .splineTo(Vector2d(-30.0, -37.0), 180.0.toRadians)
//        intake

        ZoneCReturn1
            .splineTo(Vector2d(0.0, -40.0), 0.0)
//        launch 3 rings

        ZoneCReturn2
//               intake on to pick up final ring
            .splineTo(Vector2d(-50.0, -35.0), 180.0.toRadians)
//        if have time irl to shoot ring, add another spline

//            .splineTo(Vector2d(-50.0, -35.0), 180.0.toRadians)

//        pick up wobble goal 2

//        deliver wobble goal
        ZoneA1.splineTo(Vector2d(10.0, -40.0), 0.0)

        ZoneB1
            .splineTo(Vector2d(0.0, -35.0), 0.0)
//                shoot last ring
            .splineTo(Vector2d(36.0, -20.0), 0.0)


        ZoneC1.splineTo(Vector2d(55.0, -40.0), 0.0)

//        time to park


        ParkFromB.back(20.0)
        ParkFromC.back(40.0)


//        COMPILE ALL THE TRAJECTORIES

        list.add(AvoidRings.build())
        list.add(MoveToLaunchArea.build())

        when (RingNumber) {
            0 -> {
                list.add(ZoneA.build())
                list.add(ZoneAReturn.build())
                list.add(ZoneA1.build())
            };
            1 -> {
                list.add(ZoneB.build())
                list.add(ZoneBReturn1.build())
                list.add(ZoneBReturn2.build())
                list.add(ZoneBReturn3.build())
                list.add(ZoneB1.build())
                list.add(ParkFromB.build())
            };
            4 -> {
                list.add(ZoneC.build())
                list.add(ZoneCReturn.build())
                list.add(ZoneCReturn1.build())
                list.add(ZoneCReturn2.build())
                list.add(ZoneC1.build())
                list.add(ParkFromC.build())
            }
        }


        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
