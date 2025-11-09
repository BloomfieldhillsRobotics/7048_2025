
package org.firstinspires.ftc.teamcode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousPaths;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import static org.firstinspires.ftc.teamcode.constants.AutoPoses.BlueShort.*;

@Autonomous(name = "AutoBlueShort", preselectTeleOp = "Los Protos")
public class AutoBlueShort extends NextFTCOpMode {
    public AutoBlueShort() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    // === Timers ===
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final Timer pathTimer = new Timer();
    private final Timer opmodeTimer = new Timer();

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 0;
    private static final int DETECTION_TIMEOUT = 1000;

    // === Pathing ===

    private TelemetryManager panelsTelemetry;
    private int pathState = 0;
    private int foundID   = 0;

    // === Limelight ===
    private Limelight3A limelight;

    // === Path Chains ===
    private PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;

    // === Shooting ===
    private boolean isShooting = false;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // === Timer Class ===
    public static class Timer {
        private final ElapsedTime t = new ElapsedTime();
        public void resetTimer() { t.reset(); }
        public double getElapsedTimeSeconds() { return t.seconds(); }
    }

    // === Logging ===
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsTelemetry != null) panelsTelemetry.debug(caption + ": " + value);
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        log("Path State", pathState);
    }

    // === Path Building ===
    private void buildPaths() {
        AutonomousPaths.PathContainer paths = AutonomousPaths.buildPaths(
                startPose, scoring1, scoring2,
                pickup1PPG, pickup2PPG,
                pickup1PGP, pickup2PGP,
                pickup1GPP, pickup2GPP
        );
        alignPPG = paths.alignPPG;
        toPickup1PPG = paths.toPickup1PPG;
        scoopPPG = paths.scoopPPG;
        backToScorePPG = paths.backToScorePPG;
        leavePPG = paths.leavePPG;
        alignPGP = paths.alignPGP;
        toPickup1PGP = paths.toPickup1PGP;
        scoopPGP = paths.scoopPGP;
        backToScorePGP = paths.backToScorePGP;
        leavePGP = paths.leavePGP;
        alignGPP = paths.alignGPP;
        toPickup1GPP = paths.toPickup1GPP;
        scoopGPP = paths.scoopGPP;
        backToScoreGPP = paths.backToScoreGPP;
        leaveGPP = paths.leaveGPP;
    }

    // === AprilTag Detection (runs in start()) ===
    private void detectAprilTag() {
        int timeout = 0;
        while (timeout < DETECTION_TIMEOUT && foundID == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagID = fiducials.get(0).getFiducialId();
                    if (tagID == PPG_TAG_ID || tagID == PGP_TAG_ID || tagID == GPP_TAG_ID) {
//                    if (tagID == PPG_TAG_ID || tagID == GPP_TAG_ID) {
                        foundID = tagID;
                        log("Detected Tag", tagID);
                        return;
                    }
                }
            }
            new Delay(5);
            timeout++;
        }
        if(foundID == 0 && timeout > DETECTION_TIMEOUT)
//            foundID = PGP_TAG_ID;
            foundID = PPG_TAG_ID;
        log("Warning", "No tag detected – using PPG");
    }

    // ==============================================================
    //  OpMode Lifecycle – ONLY THESE 5 METHODS
    // ==============================================================

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setMaxPower(0.75);
        PedroComponent.follower().setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        opmodeTimer.resetTimer();

        buildPaths();

        log("Status", "INIT: Ready");
    }

    @Override
    public void onWaitForStart() {
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
            opmodeTimer.resetTimer();
            detectAprilTag(); // This should run before building the command
            log("Status", "START: Running");

            // 1. Create a placeholder for the command we are about to build
            Command autonomousCommand;

            // 2. Build the correct sequence of commands based on the detected tag
            switch (foundID) {
                case PPG_TAG_ID:
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignPPG, true, .8),
                            PurpleProtonRobot.INSTANCE.AutoPPG3ShortShot,
//                            PurpleProtonRobot.INSTANCE.IntakeRun,
//                            new FollowPath(toPickup1PPG, true, .8),
//                            new FollowPath(scoopPPG, true, 0.3),
//                            new Delay(1),
//                            new FollowPath(scoopPPG, true, 0.3),
//                            new FollowPath(toPickup1PPG, true, .8),
//                            PurpleProtonRobot.INSTANCE.IntakeStop,
//                            new FollowPath(backToScorePPG, true, .8),
//                            PurpleProtonRobot.INSTANCE.Auto3ShortShot,
                            new FollowPath(leavePPG, true, .8)
                    );
                    break;
                case PGP_TAG_ID:
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignPGP, true, .8),
                            PurpleProtonRobot.INSTANCE.AutoPGP3ShortShot,
//                            PurpleProtonRobot.INSTANCE.IntakeRun,
//                            new FollowPath(toPickup1PGP, true, .8),
//                            new FollowPath(scoopPGP, true, 0.3),
//                            new Delay(1),
//                            new FollowPath(scoopPGP, true, 0.3),
//                            new FollowPath(toPickup1PGP, true, .8),
//                            PurpleProtonRobot.INSTANCE.IntakeStop,
//                            new FollowPath(backToScorePGP, true, .8),
//                            PurpleProtonRobot.INSTANCE.Auto3ShortShot,
                            new FollowPath(leavePGP, true, .8)
                    );
                    break;
                case GPP_TAG_ID:
                default: // Default to GPP if something goes wrong
                    autonomousCommand = new SequentialGroup(
                            new FollowPath(alignGPP, true, .8),
                            PurpleProtonRobot.INSTANCE.AutoGPP3ShortShot,
//                            PurpleProtonRobot.INSTANCE.IntakeRun,
//                            new FollowPath(toPickup1GPP, true, .8),
//                            new FollowPath(scoopGPP, true, 0.3),
//                            new Delay(1),
//                            new FollowPath(scoopGPP, true, 0.3),
//                            new FollowPath(toPickup1GPP, true, .8),
//                            PurpleProtonRobot.INSTANCE.IntakeStop,
//                            new FollowPath(backToScoreGPP, true, .8),
//                            PurpleProtonRobot.INSTANCE.Auto3ShortShot,
                            new FollowPath(leaveGPP, true, .8)
                    );
                    break;
            }

            // 3. Schedule the one, big command to run. The scheduler handles the rest.
            autonomousCommand.schedule();
    };

    @Override
    public void onUpdate() {
            // onUpdate can be simplified. The command scheduler runs automatically.
            // You only need Pedro's update and your telemetry.
            PedroComponent.follower().update();

            if (panelsTelemetry != null) panelsTelemetry.update();

            Pose pose = PedroComponent.follower().getPose();
            double normH = Math.toDegrees((pose.getHeading() + 2 * Math.PI) % (2 * Math.PI));

            log("X",            String.format("%.2f", pose.getX()));
            log("Y",            String.format("%.2f", pose.getY()));
            log("Heading",      String.format("%.2f°", normH));
            log("Found ID",     foundID);
            log("Time (s)",     String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));

            telemetry.update();
    }

    @Override
    public void onStop() {
        if (limelight != null) limelight.stop();
        log("Status", "STOP: Done");
    }
};
