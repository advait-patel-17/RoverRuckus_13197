package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
public class Vision extends LinearOpMode{

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    public static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public void runOpMode(){

    }

    public void initVision(){
        initTfod();
        initVuforia();
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public boolean checkForGold(LinearOpMode om){
        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }
        }

        boolean mineralPosition = false;
        double startTime = om.getRuntime();
        while (opModeIsActive() && om.getRuntime()-startTime<2.5 && !mineralPosition) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 1) {
                        int goldMineralX = -1;
                        //int silverMineral1X = -1;
                        //int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getBottom()>520){
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1) {
                                telemetry.addData("Gold Status", "Detected");
                                mineralPosition = true;
                            }
                            else{
                                telemetry.addData("Gold Status", "Not Detected");
                            }
                        }
                    }
                }
            }
            telemetry.update();
        }
        return mineralPosition;
    }

    public double degreesToGold(LinearOpMode om){
        if (tfod != null) {
            boolean foundGold = false;
            double startTime = om.getRuntime();
            double lowestY = -1;
            double centerX = 0;
            while(om.opModeIsActive() && om.getRuntime()-startTime < 2){
                List<Recognition> recog = getGold();
                if(recog != null) {
                    for (Recognition r : recog) {
                        //om.telemetry.addData("GoldLeft" , r.getLeft());
                        //om.telemetry.addData("GoldRight", r.getRight());
                        //om.telemetry.addData("GoldTop", r.getTop());
                        //om.telemetry.addData("GoldBottom", r.getBottom());
                        //om.telemetry.update();
                        if (r.getBottom() > 520) {
                            if (r.getBottom() > lowestY) {
                                lowestY = r.getBottom();
                                centerX = (r.getLeft() + r.getRight()) / 2;
                                //foundGold = true;
                            }
                        }
                    }
                }
            }
            return centerX*19/460;
        }
        else {
            return 0;}
    }

    public List<Recognition> getGold(){
        if(tfod != null){
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

}
