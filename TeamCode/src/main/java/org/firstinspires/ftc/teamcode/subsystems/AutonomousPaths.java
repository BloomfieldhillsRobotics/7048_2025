
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.extensions.pedro.PedroComponent;

public class AutonomousPaths {

    public static class PathContainer {
        public PathChain alignScan, scanToScore;
        public PathChain alignShoot, toPickup1PPG, scoopPPG, reversescoopPPG, backToScorePPG, leavePPG;
        public PathChain toPickup1PGP, scoopPGP, reversescoopPGP, backToScorePGP, leavePGP;
        public PathChain toPickup1GPP, scoopGPP, reversescoopGPP, backToScoreGPP, leaveGPP;
    }

    public static PathContainer buildPaths(
            Pose scanPose,
            Pose startPose, Pose scoring1, Pose scoring2,
            Pose pickup1PPG, Pose pickup2PPG,
            Pose pickup1PGP, Pose pickup2PGP,
            Pose pickup1GPP, Pose pickup2GPP
    ) {
        PathContainer paths = new PathContainer();

        paths.alignScan = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();

        paths.scanToScore = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scanPose, scoring1))
                .setLinearHeadingInterpolation(scanPose.getHeading(), scoring1.getHeading())
                .build();

        paths.alignShoot = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
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
