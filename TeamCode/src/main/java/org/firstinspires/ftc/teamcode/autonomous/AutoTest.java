
package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTest", group = "Autonomous")
@Configurable // Panels
public class AutoTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer;
    private Servo servo;

    //골대 시작 각도: -36도(블루)

    private enum ServoState {
        IDLE,
        OPEN,
        CLOSE
    }
    private ServoState servoState = ServoState.IDLE;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        pathTimer.resetTimer();

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




//#region paths
    public static class Paths {
        public PathChain OuttakeStart;
        public PathChain IntakeStart;
        public PathChain IntakeEnd;
        public PathChain OuttakeAfterIntake;
        public PathChain OpenFromOuttake;
        public PathChain OuttakeAfterOpen;

        public Paths(Follower follower) {
            OuttakeStart = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 122.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                    .build();

            IntakeStart = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(52.796, 59.104),
                                    new Pose(40.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 60.000),

                                    new Pose(20.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            OuttakeAfterIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 60.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            OpenFromOuttake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(34.587, 64.752),
                                    new Pose(11.922, 60.548)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))

                    .build();

            OuttakeAfterOpen = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.922, 60.548),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))

                    .build();
        }
    }
//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                followPath(paths.OuttakeStart,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    delay(1);
                    followPath(paths.IntakeStart,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    followPath(paths.IntakeEnd,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    delay(0.2);
                    followPath(paths.OuttakeAfterIntake,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.OpenFromOuttake,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    delay(2);
                    followPath(paths.OuttakeAfterOpen,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.OpenFromOuttake,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    delay(2);
                    followPath(paths.OuttakeAfterOpen,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
            case -1:
                //IDLE
                panelsTelemetry.addData("status: ","complete");
                break;

        }
        switch (servoState){
            case IDLE:
                break;
            case OPEN:
                servo.setPosition(1);
                setServoState(ServoState.IDLE);
                break;
            case CLOSE:
                servo.setPosition(0);
                setServoState(ServoState.IDLE);
                break;
        }
    }
    void setPathState(int state){
        pathState = state;
    }
    void setServoState(ServoState state){
        servoState = state;
    }
    void followPath(PathChain path,boolean holdEnd){
        pathTimer.resetTimer();
        follower.followPath(path,holdEnd);
    }
    void delay(double t){
        Timer waitTimer = new Timer();
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTimeSeconds() <= t){
            panelsTelemetry.addData("status: ", "waiting");
            panelsTelemetry.update();
        }
        return;
    }
}