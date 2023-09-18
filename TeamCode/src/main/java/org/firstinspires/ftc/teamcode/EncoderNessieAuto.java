package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="EncoderNessieAuto")
public class EncoderNessieAuto extends LinearOpMode {

    class lowerArmToLowPosition extends TimerTask {
        public void run() {
            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition );
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition );

            telemetry.addData("AAAAA", 3);
            telemetry.update();
        }
    }

    class lowerArmToMediumPosition extends TimerTask {
        public void run() {

            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition - 0.03);
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition + 0.03);
        }
    }

    class lowerArmToHighPosition extends TimerTask {
        public void run() {
            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition - 0.07);
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition + 0.07);

            telemetry.addData("AAAAA", 3);
            telemetry.update();
        }
    }

    class closeClaw extends TimerTask {
        public void run() {
            Finger.setPosition(FingerGrabPosition);
        }
    }

    class openClaw extends TimerTask {
        public void run() {
            Finger.setPosition(FingerReleasePosition);
        }
    }
    private final int numberOfRowsToScanInImage = 30;
    private final int timeToRaiseArmToMediumJunction = 1200;
    private Servo Finger;
    private CRServo Spinner;
    private CRServo ElbowL;
    private CRServo ElbowR;
    private DcMotor VerticalSlidePackL;
    private DcMotor VerticalSlidePackR;
    private final double SlidePackSpeed = 1.0;
    private final double FingerReleasePosition = 0.61;
    private final double FingerGrabPosition = 0.65;
    private final double SpinnerForwardPosition = .6;//0.9;
    private final double SpinnerBackwardPosition = .05; //0.35;
    private final double SpinnerIntermediatePosition = .78; //0.68;
    //    private final double SpinnerGrabbingPosition = 1.0;
    private final double ElbowLForwardPosition = 0.15;
    private final double ElbowLBackwardPosition = 0.95;
    private final double ElbowLIntermediatePosition = 0.39;
    private final double ElbowRForwardPosition = 0.81;
    private final double ElbowRBackwardPosition = 0.05;
    private final double ElbowRIntermediatePosition = 0.575;
    private NessieTeleop.PoleHeight CurrentPoleHeight = NessieTeleop.PoleHeight.GROUND;
    private NessieTeleop.FingerHeight CurrentFingerHeight = NessieTeleop.FingerHeight.LOW;
    private NessieAuto.ParkingSpace parkingSpace = NessieAuto.ParkingSpace.UNO;
    private static final String VUFORIA_KEY = "AVPRW+T/////AAABmYg0Njwhc0n/teI+7Sz8f/Baxyp0o6W48fBEflz8RZs3G/bVjI/5PyebGV6SkXhE1unHTRVzOVCo2cuuePhML8YCeHWm1dHZ2KbshLfc/yne7rfe2VaKPR3rrJXPF5CdMTWj4nTxm6w7KxiqvtvF2p2si1FrculcXUwbHeZ9X3O6VSntXMuNJDxXJEC3O5hT5kb7ZzsSWlot9YfUqJRxttrYYz8Xu1D2IhtOs26a2A9FC8afgGouyHucBDfl+WP59+H6wYaXRbyvcFdytq9Fp7mlSsA9RA6DV70PtJWDmehLO5hhOKq4ihVNCjJcgG38UefDAyDhWWMdjRwsiaVctq6QkmG1oMuTfIF1Dun2lDpZ";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private Timer timer = new Timer();
    private ElapsedTime eTime = new ElapsedTime();
    private final double ANGLE_1 = 0.40;

    @Override
    public void runOpMode() {
        Finger = hardwareMap.servo.get("FG");
        Spinner = hardwareMap.crservo.get("SP");
        ElbowL = hardwareMap.crservo.get("EL");
        ElbowR = hardwareMap.crservo.get("ER");
        VerticalSlidePackL = hardwareMap.dcMotor.get("VSPL");
        VerticalSlidePackR = hardwareMap.dcMotor.get("VSPR");

        // Set Directions
        Finger.setDirection(Servo.Direction.FORWARD);
        Spinner.setDirection(CRServo.Direction.FORWARD);
        VerticalSlidePackL.setDirection(DcMotor.Direction.REVERSE);
        VerticalSlidePackR.setDirection(DcMotor.Direction.FORWARD);

        VerticalSlidePackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VerticalSlidePackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequenceBuilder tsb = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Finger.setPosition(FingerGrabPosition);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerIntermediatePosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLIntermediatePosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRIntermediatePosition);
                })
                .lineTo(new Vector2d(36, -14))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(48, -14))
                .turn(ANGLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, timeToRaiseArmToMediumJunction);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    timer.schedule(new openClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    timer.schedule(new lowerArmToHighPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerBackwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, timeToRaiseArmToMediumJunction);
                })
                .waitSeconds(2.5)
                .turn(-ANGLE_1)
                .lineTo(new Vector2d(59.5, -14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    timer.schedule(new closeClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerIntermediatePosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLIntermediatePosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRIntermediatePosition);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(47, -14))
                .turn(ANGLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, timeToRaiseArmToMediumJunction);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    timer.schedule(new openClaw(), 0);
                    timer.schedule(new closeClaw(), 500);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    timer.schedule(new lowerArmToLowPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerIntermediatePosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, timeToRaiseArmToMediumJunction);
                })
                .waitSeconds(2.5)
                .turn(-ANGLE_1);

        boolean isCameraReady = getCameraReady();

        waitForStart();

        if(isStopRequested()) return;

        if (isCameraReady) {
            parkingSpace = getCameraReading();
//            parkingSpace = NessieAuto.ParkingSpace.TRES;
        }

        telemetry.addData("parkingSpace", parkingSpace);
        telemetry.update();

        switch (parkingSpace) {
            case UNO:
                tsb.lineTo(new Vector2d(12, -13));
                break;
            case DOS:
                tsb.lineTo(new Vector2d(36, -13));
                break;
            case TRES:
                tsb.lineTo(new Vector2d(60, -13));
                break;
        }

        TrajectorySequence ts = tsb.waitSeconds(30).build();

        Finger.setPosition(FingerGrabPosition);
        VerticalSlidePackL.setPower(0.05);
        VerticalSlidePackR.setPower(0.05);

        drive.followTrajectorySequence(ts);
    }

    private void moveSlidePack(NessieTeleop.SlidePackDirection spd, double power, double time) {
        eTime.reset();
        switch (spd) {
            case UP:
                VerticalSlidePackL.setPower(power);
                VerticalSlidePackR.setPower(power);
                break;
            case DOWN:
                VerticalSlidePackL.setPower(-power);
                VerticalSlidePackR.setPower(-power);
                break;
        }
        while (opModeIsActive() && eTime.milliseconds() < time) {
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        VerticalSlidePackL.setPower(0.05);
        VerticalSlidePackR.setPower(0.05);
    }

    private void initVuforia() {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(10);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }

    private boolean getCameraReady() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            telemetry.addData("TFOD is Activated", "");

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }

        if (tfod == null) {
            telemetry.addData("TFOD is Null", "");
            telemetry.update();
            return false;
        }
        return true;
    }

    private NessieAuto.ParkingSpace getCameraReading() {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch(Exception e) {
            telemetry.addData("e", "E");
            telemetry.update();
        }
        if (frame == null) return NessieAuto.ParkingSpace.UNO;
        long numImages = frame.getNumImages();
        telemetry.addData("numImages", numImages);
        telemetry.update();
        Image image = null;
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(i);
            }
        }

        int[] colors = {0, 0, 0, 0};

        if (image != null) {
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            int imgWidth = image.getWidth();
            int imgHeight = image.getHeight();
            int[] startingIndexes = getRowStartingIndexes(imgHeight, imgWidth, numberOfRowsToScanInImage);
            for (int i = numberOfRowsToScanInImage / 3; i < numberOfRowsToScanInImage * 2 / 3; i++) {
                for (int j = startingIndexes[i] + imgWidth * 2 / 3; j < startingIndexes[i] + imgWidth * 2 * 2/3; j += 2) {
                    colors[getColor(pixelArray[j], pixelArray[j+1])]++;
                    telemetry.addData("width", imgWidth);
                    telemetry.addData("height", imgHeight);
                    telemetry.addData("startingIndexes[i]", startingIndexes[i]);
                    telemetry.addData("yellow", colors[0]);
                    telemetry.addData("green", colors[1]);
                    telemetry.addData("black", colors[2]);
                    telemetry.update();
                }
            }
        }

        frame.close();
        int max_index = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[max_index])
                max_index = i;
        }

        telemetry.addData("yellow", colors[0]);
        telemetry.addData("green", colors[1]);
        telemetry.addData("black", colors[2]);
        telemetry.update();
//        sleep(5000);

        if (max_index == 0)
            return NessieAuto.ParkingSpace.UNO;
        if (max_index == 1)
            return NessieAuto.ParkingSpace.DOS;
        return NessieAuto.ParkingSpace.TRES;
    }

    private int[] getRowStartingIndexes(int height, int width, int numRows) {
        int[] newArr = new int[numRows];
        int stepSize = 2 * height / numRows * width;
        for (int i = 1; i < numRows; i++) {
            newArr[i] = i * stepSize;
        }
        return newArr;
    }

    private int getColor(byte b1, byte b2) {
        // GGGBBBBB RRRRRGGG;
        String s1 = String.format("%8s", Integer.toBinaryString(b2 & 0xFF)).replace(' ', '0');
        String s2 = String.format("%8s", Integer.toBinaryString(b1 & 0xFF)).replace(' ', '0');
        // RRRRRGGG GGGBBBBB;
        int[] color = new int[3];
        String r = s1.substring(0, 5);
        String g = s1.substring(5) + s2.substring(0, 3);
        String b = s2.substring(3);
        color[0] = convertBitStringToInt(r);
        color[1] = convertBitStringToInt(g);
        color[2] = convertBitStringToInt(b);
        double[] hsv = convertRGBtoHSV(color);
        telemetry.addData("hsv", hsv[0]);
        telemetry.addData("hsv", hsv[1]);
        telemetry.addData("hsv", hsv[2]);
        telemetry.addData("b1", b1);
        telemetry.addData("b2", b2);
        telemetry.addData("hsv[2]", hsv[2]);
        if (hsv[2] < 0.3)
            return 2;
        if (hsv[0] >= 70 && hsv[0] <= 155 && hsv[1] > 0.15 && hsv[2] > 0.2)
            return 1;
        if (hsv[0] >= 45 && hsv[0] <= 70 && hsv[1] > 0.15 && hsv[2] > 0.5)
            return 0;
        return 3;
    }

    private double[] convertRGBtoHSV(int[] rgb) {
        double rPrime = (double) rgb[0]/31;
        double gPrime = (double) rgb[1]/63;
        double bPrime = (double) rgb[2]/31;
        double cMax = Math.max(rPrime, Math.max(gPrime, bPrime));
        double cMin = Math.min(rPrime, Math.min(gPrime, bPrime));
        double delta = cMax - cMin;
        double[] hsv = new double[3];

        // calculate hue
        if (delta == 0)
            hsv[0] = 0;
        else if (cMax == rPrime) {
            double temp = ((gPrime - bPrime) / delta) % 6;
            if (temp < 0)
                temp += 6;
            hsv[0] = 60 * temp;
        }
        else if (cMax == gPrime)
            hsv[0] = 60 * (((bPrime - rPrime) / delta) + 2);
        else
            hsv[0] = 60 * (((rPrime - gPrime) / delta) + 4);

        // calculate saturation
        if (cMax == 0)
            hsv[1] = 0;
        else
            hsv[1] = delta / cMax;

        // calculate value
        hsv[2] = cMax;

        return hsv;
    }

    private int convertBitStringToInt(String s) {
        int sum = 0;
        // Little Endian
        // int digit = 0;
        // for (char c : s.toCharArray()) {
        //     if (c == '1') {
        //         sum += Math.pow(2, digit);
        //     }
        //     digit++;
        // }
        // Big Endian
        int digit = s.length() - 1;
        for (char c : s.toCharArray()) {
            if (c == '1') {
                sum += Math.pow(2, digit);
            }
            digit--;
        }
        return sum;
    }
}
