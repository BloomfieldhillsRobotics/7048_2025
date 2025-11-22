
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.PurpleProtonRobot;
import org.firstinspires.ftc.teamcode.subsystems.TestBed;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;


@TeleOp(name = "TestBed")
@Configurable //Panels
public class TeleOpTestBed extends NextFTCOpMode {
    private double targetvel = 1500;

    public TeleOpTestBed() {
        addComponents(
                new SubsystemComponent(TestBed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


//    private HuskyLensTagDetector tagDetector;


    @Override public void onInit() {
    }

    @Override public void onUpdate() {
        // Update Pedro Pathing and Panels every iteration
        telemetry.update();
        BindingManager.update();


    }
    public void onStartButtonPressed() {
        Gamepads.gamepad2().b()
                .whenBecomesTrue(TestBed.INSTANCE.FlyWheelLongShot)
                .whenBecomesFalse(TestBed.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().a()
                .whenBecomesTrue(TestBed.INSTANCE.FlyWheelSuperLongShot)
                .whenBecomesFalse(TestBed.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(TestBed.INSTANCE.FlyWheelShortShot)
                .whenBecomesFalse(TestBed.INSTANCE.FlyWheelStop);
        Gamepads.gamepad2().y()
                .whenBecomesTrue(new InstantCommand(() -> {
                    FlyWheel.INSTANCE.setTargetSpeed(1250);
                }))
                .whenBecomesFalse(TestBed.INSTANCE.FlyWheelStop);
        ;
    }
}