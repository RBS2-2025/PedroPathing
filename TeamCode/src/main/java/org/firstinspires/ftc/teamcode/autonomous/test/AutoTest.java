
package org.firstinspires.ftc.teamcode.autonomous.test;

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
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(144)));

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
        public PathChain IntakeStartMiddle;
        public PathChain IntakeEndMiddle;
        public PathChain OuttakeAfterIntakeMiddle;
        public PathChain OpenFromOuttake;
        public PathChain OuttakeAfterOpen;
        public PathChain IntakeStartTop;
        public PathChain IntakeEndTop;
        public PathChain IntakeStartBottom;
        public PathChain IntakeEndBottom;
        public PathChain OuttakeAfterIntake;
        public PathChain Parking;

        public Paths(Follower follower) {
            OuttakeStart = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 122.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            IntakeStartMiddle = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(52.796, 59.104),
                                    new Pose(40.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEndMiddle = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 60.000),

                                    new Pose(20.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            OuttakeAfterIntakeMiddle = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 60.000),
                                    new Pose(47.104, 69.983),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            OpenFromOuttake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(45.022, 61.413),
                                    new Pose(11.922, 60.548)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))

                    .build();

            OuttakeAfterOpen = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.922, 60.548),
                                    new Pose(55.300, 64.465),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))

                    .build();

            IntakeStartTop = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(75.261, 83.235),
                                    new Pose(40.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEndTop = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 84.000),

                                    new Pose(20.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakeStartBottom = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 84.000),
                                    new Pose(46.904, 58.922),
                                    new Pose(40.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeEndBottom = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 36.000),

                                    new Pose(20.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            OuttakeAfterIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 36.000),
                                    new Pose(48.548, 62.722),
                                    new Pose(48.209, 95.791)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.209, 95.791),

                                    new Pose(48.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }



//#endregion paths

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                followPath(paths.OuttakeStart,1,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    delay(0.2);
                    followPath(paths.IntakeStartMiddle,1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    delay(0.1);
                    followPath(paths.IntakeEndMiddle,0.4,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    delay(0.1);
                    followPath(paths.OuttakeAfterIntakeMiddle,1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.OpenFromOuttake,1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    delay(2);
                    followPath(paths.OuttakeAfterOpen,1,true);
                    setPathState(8);
                }
                break;
//            case 6:
//                if(!follower.isBusy()){
//                    delay(1);
//                    followPath(paths.OpenFromOuttake,true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()){
//                    delay(2);
//                    followPath(paths.OuttakeAfterOpen,true);
//                    setPathState(8);
//                }
//                break;
            case 8:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.IntakeStartTop,1,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    delay(0.1);
                    followPath(paths.IntakeEndTop,0.4,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    delay(0.1);
                    followPath(paths.OuttakeAfterIntake,1,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.IntakeStartBottom,1,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    delay(0.1);
                    followPath(paths.IntakeEndBottom,0.4,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    delay(0.1);
                    followPath(paths.OuttakeAfterIntake,1,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    delay(1);
                    followPath(paths.Parking,1,true);
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
    void followPath(PathChain path,double maxPower,boolean holdEnd){
        pathTimer.resetTimer();
        follower.followPath(path,maxPower,holdEnd);
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