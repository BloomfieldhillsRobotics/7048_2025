
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;

public class Autonomous {

    /**
     * Builds a standard autonomous cycle command.
     * This sequence involves aligning, shooting, going to the stack, collecting,
     * returning to the backboard, shooting again, and then parking.
     *
     * @param alignPath       The path to align for the first shot.
     * @param initialShot     The command to score the first (purple) pixel.
     * @param toPickupPath    The path to the pixel stack.
     * @param scoopPath       The short path to scoop up pixels.
     * @param reverseScoopPath The short path to back away from the stack.
     * @param backToScorePath The path from the stack back to the scoring position.
     * @param finalShot       The command to score the collected pixels.
     * @param leavePath       The path to park.
     * @return A SequentialGroup command that runs the entire autonomous routine.
     */
    public static Command buildCycleCommand(
            PathChain alignScan, PathChain alignPath, Command initialShot,
            PathChain toPickupPath, PathChain scoopPath,
            PathChain reverseScoopPath, PathChain backToScorePath,
            Command finalShot, PathChain leavePath) {

        return new SequentialGroup(
                new FollowPath(alignPath, true, 0.9),
                initialShot,
                PurpleProtonRobot.INSTANCE.IntakeRun,
                new FollowPath(toPickupPath, true, 0.9),
                new FollowPath(scoopPath, true, 0.3),
                PurpleProtonRobot.INSTANCE.IntakeStop,
                new FollowPath(reverseScoopPath, true, 0.9),
                new FollowPath(backToScorePath, true, 0.9),
                finalShot,
                new FollowPath(leavePath, true, 0.9)
        );
    }

    public static class PathContainer {
        public PathChain alignScan;
        public PathChain alignPPG, toPickup1PPG, scoopPPG, reversescoopPPG, backToScorePPG, leavePPG;
        public PathChain alignPGP, toPickup1PGP, scoopPGP, reversescoopPGP, backToScorePGP, leavePGP;
        public PathChain alignGPP, toPickup1GPP, scoopGPP, reversescoopGPP, backToScoreGPP, leaveGPP;
    }

    public static PathContainer buildPaths(
            Pose startPose, Pose scanPose, Pose scoring1, Pose scoring2,
            Pose pickup1PPG, Pose pickup2PPG,
            Pose pickup1PGP, Pose pickup2PGP,
            Pose pickup1GPP, Pose pickup2GPP
    ) {
        PathContainer paths = new PathContainer();

        paths.alignScan = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();
        paths.alignPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scanPose, scoring1))
                .setLinearHeadingInterpolation(scanPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1PPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PPG))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PPG.getHeading())
                .build();

        paths.scoopPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PPG, pickup2PPG))
                .setLinearHeadingInterpolation(pickup1PPG.getHeading(), pickup2PPG.getHeading())
                .build();

        paths.reversescoopPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PPG, pickup1PPG))
                .setLinearHeadingInterpolation(pickup2PPG.getHeading(), pickup1PPG.getHeading())
                .build();

        paths.backToScorePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickup1PPG, scoring1))
                .setLinearHeadingInterpolation(pickup1PPG.getHeading(), scoring1.getHeading())
                .build();

        paths.leavePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setLinearHeadingInterpolation(scoring1.getHeading(), scoring2.getHeading())
                .build();

        paths.alignPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scanPose, scoring1))
                .setLinearHeadingInterpolation(scanPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1PGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PGP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PGP.getHeading())
                .build();

        paths.scoopPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1PGP, pickup2PGP))
                .setLinearHeadingInterpolation(pickup1PGP.getHeading(), pickup2PGP.getHeading())
                .build();

        paths.reversescoopPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2PGP, pickup1PGP))
                .setLinearHeadingInterpolation(pickup2PGP.getHeading(), pickup1PGP.getHeading())
                .build();

        paths.backToScorePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickup1PGP, scoring1))
                .setLinearHeadingInterpolation(pickup1PGP.getHeading(), scoring1.getHeading())
                .build();

        paths.leavePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setLinearHeadingInterpolation(scoring1.getHeading(), scoring2.getHeading())
                .build();

        paths.alignGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scanPose, scoring1))
                .setLinearHeadingInterpolation(scanPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1GPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1GPP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1GPP.getHeading())
                .build();

        paths.scoopGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup1GPP, pickup2GPP))
                .setLinearHeadingInterpolation(pickup1GPP.getHeading(), pickup2GPP.getHeading())
                .build();

        paths.reversescoopGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2GPP, pickup1GPP))
                .setLinearHeadingInterpolation(pickup2GPP.getHeading(), pickup1GPP.getHeading())
                .build();

        paths.backToScoreGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(pickup1GPP, scoring1))
                .setLinearHeadingInterpolation(pickup1GPP.getHeading(), scoring1.getHeading())
                .build();

        paths.leaveGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setLinearHeadingInterpolation(scoring1.getHeading(), scoring2.getHeading())
                .build();
        
        return paths;
    }
}
