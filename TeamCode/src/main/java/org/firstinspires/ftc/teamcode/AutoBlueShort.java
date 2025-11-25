package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;
import dev.nextftc.core.commands.Command;

import static org.firstinspires.ftc.teamcode.constants.AutoPoses.BlueShort.*;

@Autonomous(name = "AutoBlueShort", preselectTeleOp = "Los Protos")
public class AutoBlueShort extends BaseAutonomous {

    @Override
    protected Pose getStartPose() { return startPose; }
    @Override
    protected Pose getScanPose() { return scanPose; }
    @Override
    protected Pose getScoring1Pose() { return scoring1; }
    @Override
    protected Pose getScoring2Pose() { return scoring2; }
    @Override
    protected Pose getPickup1PPGPose() { return pickup1PPG; }
    @Override
    protected Pose getPickup2PPGPose() { return pickup2PPG; }
    @Override
    protected Pose getPickup1PGPPose() { return pickup1PGP; }
    @Override
    protected Pose getPickup2PGPPose() { return pickup2PGP; }
    @Override
    protected Pose getPickup1GPPPose() { return pickup1GPP; }
    @Override
    protected Pose getPickup2GPPPose() { return pickup2GPP; }

    @Override
    protected Command getPpgShot() { return PurpleProtonRobot.INSTANCE.AutoPPG3ShortShot; }
    @Override
    protected Command getPgpShot() { return PurpleProtonRobot.INSTANCE.AutoPGP3ShortShot; }
    @Override
    protected Command getGppShot() { return PurpleProtonRobot.INSTANCE.AutoGPP3ShortShot; }
    @Override
    protected Command getFinalShot() { return PurpleProtonRobot.INSTANCE.Auto3ShortShot; }
}
