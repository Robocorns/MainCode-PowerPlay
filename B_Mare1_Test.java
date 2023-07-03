package Autonomii;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import Autonomii.AprilTagDetectionPipeline;
import Autonomii.HardwareMapPowerPlay;
@Autonomous (name="B_Mare1_Test", group = "#*")
@Disabled



public class B_Mare1_Test extends LinearOpMode {
    DistanceSensor dsensor,ssensor;
    private FtcDashboard dashboard;
    HardwareMapPowerPlay robot = new HardwareMapPowerPlay();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    int Left = 1;
    int Mid = 2;
    int Right = 3;
    int Mijloc=0,Stanga=0,Dreapta=0;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //distance sensor
        telemetry = new MultipleTelemetry(telemetry, dashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        robot.initPP(hardwareMap);


        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :( ");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");


                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

                if (tagOfInterest == null || tagOfInterest.id == Mid) {
                    Stanga=0;
                    Mijloc=1;
                    Dreapta=0;
                } else if (tagOfInterest==null||tagOfInterest.id == Left) {
                    Stanga=1;
                    Mijloc=0;
                    Dreapta=0;

                } else if (tagOfInterest==null||tagOfInterest.id == Right) {
                    Stanga=0;
                    Mijloc=0;
                    Dreapta=1;
                }

            }
            pickCone();
            telemetry.update();
        }
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        drive.setPoseEstimate(new Pose2d(35,-64,Math.toRadians(90)));
        TrajectorySequence stalpMij = drive.trajectorySequenceBuilder(new Pose2d(-35,-64,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34,-10,Math.toRadians(135)))
                .addDisplacementMarker(2, () ->{
                    Level();
                })
                .forward(10)
                .build();



        TrajectorySequence conuri = drive.trajectorySequenceBuilder(stalpMij.end())
                .back(12)
                .addDisplacementMarker(5,  () -> StackLevel())
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(70,-12,Math.toRadians(0)))
                .build();

        TrajectorySequence stalpMij2 = drive.trajectorySequenceBuilder(new Pose2d(70,-12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(34,-10,Math.toRadians(135)))
                .addDisplacementMarker(2, () ->{
                    Level();
                })
                .forward(10)
                .build();

        TrajectorySequence conuri2 = drive.trajectorySequenceBuilder(stalpMij2.end())
                .back(12)
                .addDisplacementMarker(5,  () -> StackLevel())
                .lineToLinearHeading(new Pose2d(60,-12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(70,-12,Math.toRadians(0)))
                .build();

        TrajectorySequence finish = drive.trajectorySequenceBuilder(stalpMij2.end())
                .back(12)
                .addDisplacementMarker(10,() -> Low())
                .turn(Math.toRadians(135))
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(finish.end())
                .back(25)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(finish.end())
                .forward(25)
                .build();

        waitForStart();
        if(opModeIsActive()&&!isStopRequested())
        {
            drive.followTrajectorySequence(stalpMij);
            dropCone();
        }
        if (isStopRequested())
            return;
    }

    public void Level(){
        telemetry.addLine("Ridicha scripeti mij");
        telemetry.update();
        robot.scripS.setTargetPosition(3900);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }

    public void StackLevel(){
        robot.scripS.setTargetPosition(1400);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }

    public void LevelCon1 (){
        robot.scripS.setTargetPosition(1000);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);}

    public void LevelCon2 (){
        robot.scripS.setTargetPosition(900);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }

    public void LevelCon3 (){
        robot.scripS.setTargetPosition(800);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }


    public void Low(){
        robot.scripS.setTargetPosition(0);
        robot.scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.scripS.setPower(1);
    }

    public void pickCone(){
        telemetry.addLine("Pick Cone");
        telemetry.update();
        robot.gheara.setPosition(0.3);
    }
    public void dropCone(){
        telemetry.addLine("Drop Cone");
        telemetry.update();
        robot.gheara.setPosition(0.6);

    }



    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addData("Left", Stanga);
        telemetry.addData("Middle", Mijloc);
        telemetry.addData("Right",Dreapta);
    }
    void TelementrySensor()
    {
        telemetry.addData("Distanta Dreapta: ",dsensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distanta Sranga:",ssensor.getDistance(DistanceUnit.CM));
        telemetry.update();}

}

