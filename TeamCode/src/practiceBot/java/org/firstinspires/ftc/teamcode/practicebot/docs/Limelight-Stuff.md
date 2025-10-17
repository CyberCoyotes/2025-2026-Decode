// https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

// 1. Initialization

// We need to set up our Limelight3A in our robot code.
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//
Limelight3A limelight;

@Override
public void init() {
limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
limelight.start(); // This tells Limelight to start looking!
}

Pipelines are like small, instantly swappable programs that change how Limelight looks at the world. You can set up 10 different pipelines in the Limelight web interface, each for a different task. Here's how you switch between them:

limelight.pipelineSwitch(0); // Switch to pipeline number 0

This is fire-and-forget. Limelight will change its pipeline in a matter of milliseconds, but your code will not wait for this before continuing. If you want to check the current pipeline index, call

result.getPipelineIndex()

See the next section to learn about getting the LLResult object.

3. Getting and Using Results
   LLResult is like a container full of information about what Limelight sees. Here's how we acquire and use that information:

LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
double tx = result.getTx(); // How far left or right the target is (degrees)
double ty = result.getTy(); // How far up or down the target is (degrees)
double ta = result.getTa(); // How big the target looks (0%-100% of the image)

    telemetry.addData("Target X", tx);
    telemetry.addData("Target Y", ty);
    telemetry.addData("Target Area", ta);
} else {
telemetry.addData("Limelight", "No Targets");
}