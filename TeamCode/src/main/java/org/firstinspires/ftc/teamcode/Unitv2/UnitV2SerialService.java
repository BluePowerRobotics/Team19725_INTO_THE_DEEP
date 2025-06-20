package org.firstinspires.ftc.teamcode.Unitv2;
import static androidx.core.content.ContextCompat.getSystemService;

import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonSyntaxException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class UnitV2SerialService implements SerialInputOutputManager.Listener{

    private final LinearOpMode opMode;
    UsbSerialPort port;

    private SerialInputOutputManager serialIoManager;
    UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final Gson gson = new Gson();
    private DetectionResult latestResult;

    public UnitV2SerialService(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public boolean initialize() {
        // 查找可用串口设备
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(opMode.hardwareMap.appContext.getUsbManager());

        if (availableDrivers.isEmpty()) {
            opMode.telemetry.addData("UnitV2", "No USB devices found");
            return false;
        }

        // 获取第一个可用端口
        UsbSerialDriver driver = availableDrivers.get(0);
        port = driver.getPorts().get(0);
        UsbDeviceConnection connection = serialIoManager.openDevice(driver.getDevice());




//        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
//        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
//        if (availableDrivers.isEmpty()) {
//            return false;
//        }
//
//        // Open a connection to the first available driver.
//        UsbSerialDriver driver = availableDrivers.get(0);
//        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
//        if (connection == null) {
//            // add UsbManager.requestPermission(driver.getDevice(), ..) handling here
//            return;
//        }
//
//        UsbSerialPort port = driver.getPorts().get(0); // Most devices have just one port (port 0)
//        port.open(connection);
//        port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        try {
            port.open(connection);
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            // 启动串口监听
            serialIoManager = new SerialInputOutputManager(port, this);
            executor.submit((Runnable) serialIoManager);

            opMode.telemetry.addData("UnitV2", "Connected successfully");
            return true;
        } catch (Exception e) {
            opMode.telemetry.addData("UnitV2 Error", e.getMessage());
            return false;
        }
    }

    @Override
    public void onNewData(byte[] data) {
        // 处理接收到的数据
        String rawData = new String(data).trim();

        if (rawData.startsWith("{") && rawData.endsWith("}")) {
            try {
                // 解析JSON数据
                latestResult = gson.fromJson(rawData, DetectionResult.class);
                opMode.telemetry.addData("UnitV2", "Detected: " + latestResult.num + " objects");
            } catch (JsonSyntaxException e) {
                opMode.telemetry.addData("UnitV2 JSON Error", rawData);
            }
        }
    }

    @Override
    public void onRunError(Exception e) {
        opMode.telemetry.addData("UnitV2 Error", e.getMessage());
    }

    public DetectionResult getLatestResult() {
        return latestResult;
    }

    public void shutdown() {
        if (serialIoManager != null) {
            serialIoManager.stop();
        }
        executor.shutdown();
        if (port != null) {
            try {
                port.close();
            } catch (Exception e) {
                // 忽略关闭错误
            }
        }
    }

    // 定义检测结果数据结构
    public static class DetectionResult {
        public int num;
        public DetectedObject[] obj;
        public String running;
    }

    public static class DetectedObject {
        public String type;
        public double prob;
        public int x;
        public int y;
        public int w;
        public int h;
    }
}