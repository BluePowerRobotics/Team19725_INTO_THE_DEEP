package org.firstinspires.ftc.teamcode.ReadUSB;

import android.content.Context;

import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class Read_Write_USB extends AppCompatActivity implements SerialInputOutputManager.Listener {
    private UsbSerialPort port;
    private SerialInputOutputManager usbIoManager;
    private TextView textView;

//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//        textView = findViewById(R.id.textView);
//
//        connectToUsbDevice();
//    }

    private void connectToUsbDevice() {
        // 1. 获取USB管理器并查找驱动
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);

        if (availableDrivers.isEmpty()) {
            textView.setText("未找到USB设备");
            return;
        }

        // 2. 获取第一个设备并打开连接
        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());

        if (connection == null) {
            textView.setText("无法打开连接，需要权限");
            // 请求权限
            //UsbManager.requestPermission(driver.getDevice());
            return;
        }

        // 3. 配置串口参数
        port = driver.getPorts().get(0);
        try {
            port.open(connection);
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            // 4. 启动异步读取
            usbIoManager = new SerialInputOutputManager(port, this);
            usbIoManager.start();

            // 5. 发送测试数据
            port.write("hello".getBytes(), 1000);

        } catch (Exception e) {
            textView.setText("连接失败: " + e.getMessage());
        }
    }
    public void sendHelloToUsb() {
        if (port != null) {
            try {
                port.write("hello".getBytes(), 1000);
            } catch (Exception e) {
                runOnUiThread(() -> textView.append("\n发送失败: " + e.getMessage()));
            }
        } else {
            runOnUiThread(() -> textView.append("\n端口未打开，无法发送"));
        }
    }
    // 6. 接收数据回调
    @Override
    public void onNewData(byte[] data) {
        runOnUiThread(() -> textView.append(new String(data)));
    }

    @Override
    public void onRunError(Exception e) {
        runOnUiThread(() -> textView.append("\n通信错误: " + e.getMessage()));
    }


    protected void close_connection() {
        // 7. 关闭连接
        if (usbIoManager != null) {
            usbIoManager.stop();
        }
        try {
            if (port != null) {
                port.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

