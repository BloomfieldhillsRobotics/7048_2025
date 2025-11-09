
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.extensions.pedro.PedroComponent;

public class AutonomousPaths {

    public static class PathContainer {
        public PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
        public PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
        public PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;
    }

    public static PathContainer buildPaths(
            Pose startPose, Pose scoring1, Pose scoring2,
            Pose pickup1PPG, Pose pickup2PPG,
            Pose pickup1PGP, Pose pickup2PGP,
            Pose pickup1GPP, Pose pickup2GPP
    ) {
        PathContainer paths = new PathContainer();

        paths.alignPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1PPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup1PPG))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup1PPG.getHeading())
                .build();

        paths.scoopPPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup2PPG))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup2PPG.getHeading())
                .build();

        paths.backToScorePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring1.getHeading())
                .build();

        paths.leavePPG = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring2))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring2.getHeading())
                .build();

        paths.alignPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1PGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup1PGP))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup1PGP.getHeading())
                .build();

        paths.scoopPGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup2PGP))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup2PGP.getHeading())
                .build();

        paths.backToScorePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring1.getHeading())
                .build();

        paths.leavePGP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring2))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring2.getHeading())
                .build();

        paths.alignGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        paths.toPickup1GPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup1GPP))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup1GPP.getHeading())
                .build();

        paths.scoopGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), pickup2GPP))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), pickup2GPP.getHeading())
                .build();

        paths.backToScoreGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring1))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring1.getHeading())
                .build();

        paths.leaveGPP = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), scoring2))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scoring2.getHeading())
                .build();
        
        return paths;
    }
}
