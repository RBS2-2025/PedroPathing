package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.PanelHelper;

// 패널 대시보드 사용법 teleop
@Configurable
@TeleOp
public class test extends LinearOpMode {
    public static int ConfigTest = 10;

    @Override
    public void runOpMode() {
        PanelHelper p = new PanelHelper(this);

        p.addline("Initialized");

        waitForStart();

        while (opModeIsActive()) {

            p.updateGamepads();

            if (p.gamepad1.a) p.debug("Panel g1 A pressed");
            if (p.gamepad2.a) p.debug("Panel g2 A pressed");

            if (gamepad1.a) p.debug("DS g1 A pressed");
            if (gamepad2.a) p.debug("DS g2 A pressed");

            if(p.gamepad1.b) p.addData("number",ConfigTest);

            p.update();
        }
    }
}