package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BaseHook;
import org.firstinspires.ftc.teamcode.subsystems.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;
import java.util.List;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class BaseAutonomous extends NextFTCOpMode {

    // Abstract methods for subclasses to provide specific values
    protected abstract Pose getStartPose();
    protected abstract Pose getScanPose();
    protected abstract Pose getScoring1Pose();
    protected abstract Pose getScoring2Pose();
    protected abstract Pose getPickup1PPGPose();
    protected abstract Pose getPickup2PPGPose();
    protected abstract Pose getPickup1PGPPose();
    protected abstract Pose getPickup2PGPPose();
    protected abstract Pose getPickup1GPPPose();
    protected abstract Pose getPickup2GPPPose();

    protected abstract Command getPpgShot();
    protected abstract Command getPgpShot();
    protected abstract Command getGppShot();
    protected abstract Command getFinalShot();

    public BaseAutonomous() {
        addComponents(
                new SubsystemComponent(PurpleProtonRobot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    // === Timers ===
    private final Timer opmodeTimer = new Timer();

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int SEQUENCE_DETECT_PIPELINE = 0;
    private static final int DISTANCE_DETECT_PIPELINE = 1;
    private static final int DETECTION_TIMEOUT = 1000;

    // === Pathing ===

    private TelemetryManager panelsTelemetry;
    private int foundID   = 0;

    // === Limelight ===
    private Limelight3A limelight;

    // === Path Chains ===
    private PathChain alignScan;
    private PathChain alignPPG, toPickup1PPG, scoopPPG, reversescoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, reversescoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, reversescoopGPP, backToScoreGPP, leaveGPP;

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

    // === Path Building ===
    private void buildPaths() {
        Autonomous.PathContainer paths = Autonomous.buildPaths(
                getScanPose(),
                getStartPose(), getScoring1Pose(), getScoring2Pose(),
                getPickup1PPGPose(), getPickup2PPGPose(),
                getPickup1PGPPose(), getPickup2PGPPose(),
                getPickup1GPPPose(), getPickup2GPPPose()
        );
        alignPPG = paths.alignPPG;
        alignScan = paths.alignScan;
        toPickup1PPG = paths.toPickup1PPG;
        scoopPPG = paths.scoopPPG;
        reversescoopPPG = paths.reversescoopPPG;
        backToScorePPG = paths.backToScorePPG;
        leavePPG = paths.leavePPG;
        alignPGP = paths.alignPGP;
        toPickup1PGP = paths.toPickup1PGP;
        scoopPGP = paths.scoopPGP;
        reversescoopPGP = paths.reversescoopPGP;
        backToScorePGP = paths.backToScorePGP;
        leavePGP = paths.leavePGP;
        alignGPP = paths.alignGPP;
        toPickup1GPP = paths.toPickup1GPP;
        scoopGPP = paths.scoopGPP;
        reversescoopGPP = paths.reversescoopGPP;
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
                    for (int i = 0; i < fiducials.size(); i++) {
                        int tagID = fiducials.get(i).getFiducialId();
                        if (tagID == PPG_TAG_ID || tagID == PGP_TAG_ID || tagID == GPP_TAG_ID) {
                            foundID = tagID;
                            log("Detected Tag", tagID);
                            return;
                        }
                    }
                }
            }
            new Delay(5);
            timeout++;
        }
        if(foundID == 0 && timeout > DETECTION_TIMEOUT)

        log("Warning", "No tag detected – using PGP");
    }
    private Command createAutonomousCycleCommand() {
        switch (foundID) {
            case PPG_TAG_ID:
                log("Selected Path", "PPG");
                return Autonomous.buildCycleCommand(
                        alignScan, alignPPG, getPpgShot(),
                        toPickup1PPG, scoopPPG, reversescoopPPG,
                        backToScorePPG, getFinalShot(), leavePPG
                );
            case PGP_TAG_ID:
                log("Selected Path", "PGP");
                return Autonomous.buildCycleCommand(
                        alignScan, alignPGP, getPgpShot(),
                        toPickup1PGP, scoopPGP, reversescoopPGP,
                        backToScorePGP, getFinalShot(), leavePGP
                );
            case GPP_TAG_ID:
            case 100:
                log("Selected Path", "GPP (or default)");
                return Autonomous.buildCycleCommand(
                        alignScan, alignGPP, getGppShot(),
                        toPickup1GPP, scoopGPP, reversescoopGPP,
                        backToScoreGPP, getFinalShot(), leaveGPP
                );
            default:
                log("No Path Found","aligning");
                foundID = 100;
                return new SequentialGroup(
                        new FollowPath(alignScan, true, 0.9)
                );
        }
    };
    private Command createAutonomousCycleCommand2() {
        detectAprilTag();
        return createAutonomousCycleCommand();
    }

    // ==============================================================
    //  OpMode Lifecycle – ONLY THESE 5 METHODS
    // ==============================================================
    public static void drawOnlyCurrent(Pose robotPose) {
        try {
            Drawing.drawRobot(robotPose);
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setMaxPower(0.8);
        PedroComponent.follower().setStartingPose(getStartPose());
        drawOnlyCurrent(PedroComponent.follower().getPose());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(SEQUENCE_DETECT_PIPELINE);
        limelight.start();

        opmodeTimer.resetTimer();

        buildPaths();

        BaseHook.INSTANCE.BaseHookLock(); // Lock the servo to hold the base plate

        log("Status", "INIT: Ready");
    }

    @Override
    public void onWaitForStart() {
        log("Starting pose", getStartPose());
        log("Current pose", PedroComponent.follower().getPose());
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
        drawOnlyCurrent(PedroComponent.follower().getPose());
    }

    @Override
    public void onStartButtonPressed() {
            opmodeTimer.resetTimer();
            detectAprilTag(); // This should run before building the command
            limelight.pipelineSwitch(DISTANCE_DETECT_PIPELINE); // Switch to pipeline 1 to focus on navigation tags 20, 24
            PedroComponent.follower().setStartingPose(getStartPose()); // Try setting starting pose again as backup after init.
            drawOnlyCurrent(PedroComponent.follower().getPose());

            //log("Time (s)", String.format("%.2f"));
            log("Status", "START: Running");
            createAutonomousCycleCommand().schedule();
            createAutonomousCycleCommand2().schedule();
    }

    @Override
    public void onUpdate() {
            // onUpdate can be simplified. The command scheduler runs automatically.
            // You only need Pedro's update and your telemetry.
            PedroComponent.follower().update();

            if (panelsTelemetry != null) panelsTelemetry.update();

            Pose pose = PedroComponent.follower().getPose();
            drawOnlyCurrent(pose);
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
}
