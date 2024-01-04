package org.firstinspires.ftc.teamcode.Utilities;

import com.google.gson.internal.Primitives;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryManager {
    private ElapsedTime runtime;

    //Declare Variables
    private Telemetry telemetry;
    private ArrayList<String> captionList = new ArrayList<>();
    private ArrayList<String> formatList = new ArrayList<>();
    private ArrayList<Primitives> dataList = new ArrayList<>();
    private int telemetryObjectCount = 0;

    //Constructor intakes hardwareMaps and runtime.
    public TelemetryManager(Telemetry telemetry, ElapsedTime runtime) {
        this.telemetry = telemetry;
        this.runtime = runtime;
    }

    public void addTelemetryObject(TelemetryObject telemetryObject) {
        captionList.add(telemetryObject.caption);
        formatList.add(telemetryObject.format);
        dataList.add(telemetryObject.data);
        telemetryObjectCount++;
    }

    public void removeTelemetryObject(TelemetryObject telemetryObject) {
        captionList.remove(telemetryObject.caption);
        formatList.remove(telemetryObject.format);
        dataList.remove(telemetryObject.data);
        telemetryObjectCount--;
    }

    public void updateTelemetry() {
        for (int i = 0; i < telemetryObjectCount; i++) {
            telemetry.addData(captionList.get(i), formatList.get(i), dataList.get(i));
        }
        telemetry.update();
    }

    public void toggleAutoClear() {
        if (telemetry.isAutoClear()) {
            telemetry.setAutoClear(false);
        }
        else {
            telemetry.setAutoClear(true);
        }
    }

    class TelemetryObject {
        String caption;
        String format;
        Primitives data;
        public TelemetryObject(String caption, String format, Primitives data) {
            this.caption = caption;
            this.format = format;
            this.data = data;
        }
    }
}
