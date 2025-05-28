package com.example.control;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.SharedPreferences;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.Button;

import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity {

    private static final String PREFS_NAME = "DronePrefs";
    private static final String PREF_IP_ADDRESS = "droneIpAddress";
    private static final String DEFAULT_IP_ADDRESS = "http://192.168.1.12";

    private EditText droneIpEditText;
    private FrameLayout leftJoystickLayout, rightJoystickLayout;
    private View leftKnob, rightKnob;
    private Switch armModeSwitch;
    private TextView switchLabel;

    private boolean isArmed = false;
    private JoystickState joystickState = new JoystickState();
    private long lastSentTime = 0;
    private String droneIpAddress = DEFAULT_IP_ADDRESS;
    private Timer sendDataTimer;


    @SuppressLint("ClickableViewAccessibility")
    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            // Đặt lại vị trí knob khi cửa sổ hoàn tất vẽ
            resetKnobPosition(leftKnob);
            resetKnobPosition(rightKnob);
        }
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        droneIpEditText = findViewById(R.id.droneIp);
        Button saveIpButton = findViewById(R.id.saveIpButton);
        leftJoystickLayout = findViewById(R.id.leftJoystick);
        rightJoystickLayout = findViewById(R.id.rightJoystick);
        leftKnob = findViewById(R.id.leftKnob);
        rightKnob = findViewById(R.id.rightKnob);
        armModeSwitch = findViewById(R.id.armSwitch);
        switchLabel = findViewById(R.id.switchLabel);

        loadIpAddress();

        saveIpButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                saveIpAddress();
            }
        });

        droneIpEditText.setOnFocusChangeListener((v, hasFocus) -> {
            if (!hasFocus) {
                saveIpAddress();
            }
        });
        droneIpEditText.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                droneIpEditText.setCursorVisible(true);
            }
        });


        setupJoystick(leftJoystickLayout, leftKnob, JoystickType.LEFT);
        setupJoystick(rightJoystickLayout, rightKnob, JoystickType.RIGHT);

        armModeSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isArmed = isChecked;
            joystickState.specialAction = isArmed ? 1.0 : 0.0;
            switchLabel.setText(isArmed ? "ARM MODE (ARMED)" : "ARM MODE (DISARMED)");

            if (!isArmed) {
                resetJoysticks();
                sendEmergencyStop();
            } else {
                sendJoystickData(); // Send ARM state immediately
            }
        });

        startDataSendingTimer();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopDataSendingTimer();
    }

    private void startDataSendingTimer() {
        sendDataTimer = new Timer();
        sendDataTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (isArmed) {
                    sendJoystickData();
                }
            }
        }, 0, 100); // Send data every 100ms
    }

    private void stopDataSendingTimer() {
        if (sendDataTimer != null) {
            sendDataTimer.cancel();
            sendDataTimer.purge();
            sendDataTimer = null;
        }
    }


    private void loadIpAddress() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        droneIpAddress = prefs.getString(PREF_IP_ADDRESS, DEFAULT_IP_ADDRESS);
        droneIpEditText.setText(droneIpAddress.replace("http://", "")); // Display without http://
    }

    private void saveIpAddress() {
        String enteredIp = droneIpEditText.getText().toString().trim();
        if (!enteredIp.startsWith("http://") && !enteredIp.startsWith("https://")) {
            enteredIp = "http://" + enteredIp; // Assume http if protocol is missing
        }
        droneIpAddress = enteredIp;
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putString(PREF_IP_ADDRESS, droneIpAddress);
        editor.apply();
        Toast.makeText(this, "IP Address Saved", Toast.LENGTH_SHORT).show();
        droneIpEditText.clearFocus();
        droneIpEditText.setCursorVisible(false);

        // Ẩn bàn phím mềm (nếu đang hiện)
        InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
        if (imm != null) {
            imm.hideSoftInputFromWindow(droneIpEditText.getWindowToken(), 0);
        }
    }

    private void sendEmergencyStop() {
        ControlData emergencyData = new ControlData(1500.0, 1500.0, 1500.0, 1500.0, 0.0);
        sendControlData(emergencyData);
        runOnUiThread(() -> Toast.makeText(MainActivity.this, "Emergency Stop Sent", Toast.LENGTH_SHORT).show());
    }

    private double scaleValue(double val) {
        if (Math.abs(val) < 0.1) return 1500.0; // Center value
        return ((val + 1) * 500.0 + 1000.0); // Scale -1..1 to 1000..2000
    }

    private void sendJoystickData() {
        if (!isArmed) return;
        long now = System.currentTimeMillis();
        if (now - lastSentTime < 100) return; // Throttle to 100ms
        lastSentTime = now;

        ControlData data = new ControlData(
                scaleValue(joystickState.leftY), // Roll
                scaleValue(joystickState.leftX), // Pitch
                scaleValue(joystickState.rightX), // Yaw
                scaleValue(joystickState.rightY), // Throttle
                joystickState.specialAction
        );

        sendControlData(data);
        Log.d("SendData", "Sending data: " + data.toJson());
    }

    private void sendControlData(ControlData data) {
        new SendControlDataTask().execute(data);
    }

    private class SendControlDataTask extends AsyncTask<ControlData, Void, String> {

        @Override
        protected String doInBackground(ControlData... controlData) {
            ControlData data = controlData[0];
            String response = "";
            HttpURLConnection urlConnection = null;
            try {
                // In ra URL đầy đủ để kiểm tra
                String fullUrl = droneIpAddress + "/control";
                Log.d("SendData", "Connecting to URL: " + fullUrl);

                URL url = new URL(fullUrl);
                urlConnection = (HttpURLConnection) url.openConnection();
                urlConnection.setRequestMethod("POST");
                urlConnection.setRequestProperty("Content-Type", "application/json");
                urlConnection.setConnectTimeout(5000); // 5 giây timeout
                urlConnection.setReadTimeout(5000);
                urlConnection.setDoOutput(true);

                String jsonData = data.toJson();
                Log.d("SendData", "Sending JSON: " + jsonData);

                OutputStream os = urlConnection.getOutputStream();
                os.write(jsonData.getBytes());
                os.flush();

                // Lấy và in response code
                int responseCode = urlConnection.getResponseCode();
                Log.d("SendData", "Response code: " + responseCode);

                if (responseCode == HttpURLConnection.HTTP_OK) {
                    response = readStream(urlConnection.getInputStream());
                    Log.d("SendData", "Response body: " + response);
                } else {
                    // Đọc lỗi từ error stream
                    String errorResponse = readStream(urlConnection.getErrorStream());
                    response = "HTTP Error: " + responseCode + " - " + errorResponse;
                    Log.e("SendData", response);
                }

            } catch (Exception e) {
                response = "Error: " + e.getMessage();
                Log.e("SendDataError", "Exception details:", e);
            } finally {
                if (urlConnection != null) {
                    urlConnection.disconnect();
                }
            }
            return response;
        }

        @Override
        protected void onPostExecute(String result) {
            if (result.startsWith("Error:") || result.startsWith("HTTP Error:")) {
                runOnUiThread(() ->
                        Toast.makeText(MainActivity.this,
                                "Network Error: " + result, Toast.LENGTH_LONG).show());
            }
        }
    }

    private String readStream(InputStream in) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(in));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line).append('\n');
        }
        return sb.toString();
    }


    private enum JoystickType {
        LEFT, RIGHT
    }

    @SuppressLint("ClickableViewAccessibility")
    private void setupJoystick(FrameLayout joystickLayout, View knob, JoystickType type) {
        // Kích thước của joystick và knob
        final float joystickRadius = joystickLayout.getWidth() / 2f; // Bán kính của joystick
        final float knobRadius = knob.getWidth() / 2f; // Bán kính của knob

        joystickLayout.post(() -> {
            // Lấy kích thước thực tế sau khi view được vẽ
            final float actualJoystickRadius = joystickLayout.getWidth() / 2f;
            final float actualKnobRadius = knob.getWidth() / 2f;

            // Đặt knob vào giữa ban đầu
            knob.setX(actualJoystickRadius - actualKnobRadius);
            knob.setY(actualJoystickRadius - actualKnobRadius);
        });

        joystickLayout.setOnTouchListener((v, event) -> {
            // Tính toán lại kích thước thực tế mỗi lần chạm
            final float actualJoystickRadius = joystickLayout.getWidth() / 2f;
            final float actualKnobRadius = knob.getWidth() / 2f;
            final float maxDistance = actualJoystickRadius - actualKnobRadius;

            if (event.getAction() == MotionEvent.ACTION_DOWN || event.getAction() == MotionEvent.ACTION_MOVE) {
                // Vị trí chạm tương đối so với tâm joystick
                float touchX = event.getX() - actualJoystickRadius;
                float touchY = event.getY() - actualJoystickRadius;

                // Tính khoảng cách từ điểm chạm tới tâm
                double distance = Math.sqrt(touchX * touchX + touchY * touchY);

                // Giới hạn di chuyển trong phạm vi joystick
                if (distance > maxDistance) {
                    touchX = (float) (touchX * maxDistance / distance);
                    touchY = (float) (touchY * maxDistance / distance);
                    distance = maxDistance;
                }

                // Cập nhật vị trí của knob
                knob.setX(actualJoystickRadius - actualKnobRadius + touchX);
                knob.setY(actualJoystickRadius - actualKnobRadius + touchY);

                // Chuyển đổi sang giá trị chuẩn hóa từ -1 đến 1
                float normalizedX = touchX / maxDistance;
                float normalizedY = touchY / maxDistance;

                // Cập nhật trạng thái joystick
                if (type == JoystickType.LEFT) {
                    joystickState.leftX = normalizedX;
                    joystickState.leftY = -normalizedY; // Đảo ngược trục Y để phù hợp với điều khiển drone
                } else {
                    joystickState.rightX = normalizedX;
                    joystickState.rightY = -normalizedY; // Đảo ngược trục Y
                }

                sendJoystickData(); // Gửi dữ liệu ngay lập tức
                return true;
            } else if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                // Nếu muốn giữ vị trí sau khi thả, không cần làm gì
                // Nếu muốn quay về tâm, bỏ comment dòng dưới
                resetKnobPosition(knob);
                if (type == JoystickType.LEFT) {
                    joystickState.leftX = 0;
                    joystickState.leftY = 0;
                } else {
                    joystickState.rightX = 0;
                    joystickState.rightY = 0;
                }
                sendJoystickData();
                return true;
            }
            return false;
        });
    }

    // Thêm một nút để reset vị trí joystick khi cần
    public void resetJoysticks() {
        resetKnobPosition(leftKnob);
        resetKnobPosition(rightKnob);
        joystickState.leftX = 0;
        joystickState.leftY = 0;
        joystickState.rightX = 0;
        joystickState.rightY = 0;
        joystickState.specialAction = isArmed ? 1.0 : 0.0;
    }

    private void resetKnobPosition(View knob) {
        FrameLayout parent = (FrameLayout) knob.getParent();
        float joystickRadius = parent.getWidth() / 2f;
        float knobRadius = knob.getWidth() / 2f;

        knob.animate()
                .x(joystickRadius - knobRadius)
                .y(joystickRadius - knobRadius)
                .setDuration(150)
                .start();
    }




    private static class JoystickState {
        public double leftX = 0;
        public double leftY = 0;
        public double rightX = 0;
        public double rightY = 0;
        public double specialAction = 0;
    }

    private static class ControlData {
        public double roll;
        public double pitch;
        public double yaw;
        public double throttle;
        public double AUX1;

        public ControlData(double roll, double pitch, double yaw, double throttle, double AUX1) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            this.throttle = throttle;
            this.AUX1 = AUX1;
        }

        public String toJson() {
            try {
                JSONObject jsonObject = new JSONObject();
                jsonObject.put("roll", String.format(Locale.US, "%.2f", roll));
                jsonObject.put("pitch", String.format(Locale.US,"%.2f", pitch));
                jsonObject.put("yaw", String.format(Locale.US,"%.2f", yaw));
                jsonObject.put("throttle", String.format(Locale.US,"%.2f", throttle));
                jsonObject.put("AUX1", String.format(Locale.US,"%.2f", AUX1));
                return jsonObject.toString();
            } catch (Exception e) {
                Log.e("ControlData", "JSON Encoding Error", e);
                return "{}";
            }
        }
    }
}