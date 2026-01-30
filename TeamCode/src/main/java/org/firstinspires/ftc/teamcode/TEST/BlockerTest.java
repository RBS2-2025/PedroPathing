package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Configurable
@TeleOp(name = "Blocker Test")
public class BlockerTest extends OpMode {
    Robot robot;
    public static double def_pos = 0.6;
    public static double act_pos = 0.43;
    @Override
    public void init() {
        this.robot = new Robot(hardwareMap,true);
        robot.blocker.setPosition(def_pos);
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()){
            robot.blocker.setPosition(def_pos);
        }
        if(gamepad1.aWasReleased()){
            robot.blocker.setPosition(act_pos);
        }
    }
}
