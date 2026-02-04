package org.firstinspires.ftc.teamcode.vision;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

    /* ===================== LIMELIGHT HELPERS ===================== */

    public class LimelightHelpers {

        private static String getJSON(String name) {
            try {
                URL url = new URL("http://" + name + ".local:5807/results");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setConnectTimeout(50);
                conn.setReadTimeout(50);
                InputStream is = conn.getInputStream();
                byte[] buffer = new byte[1024];
                StringBuilder sb = new StringBuilder();
                int n;
                while ((n = is.read(buffer)) != -1) {
                    sb.append(new String(buffer, 0, n, StandardCharsets.UTF_8));
                }
                return sb.toString();
            } catch (Exception e) {
                return "";
            }
        }

        public static boolean getTV(String name) {
            return getJSON(name).contains("\"tv\":true");
        }

        public static double getTX(String name) {
            try {
                String json = getJSON(name);
                int i = json.indexOf("\"tx\":");
                return Double.parseDouble(json.substring(i + 5, json.indexOf(',', i)));
            } catch (Exception e) {
                return 0.0;
            }
        }

        public static double[] getBotPose_WpiBlue(String name) {
            try {
                String json = getJSON(name);
                int i = json.indexOf("\"botpose_wpiblue\":[");
                int start = i + 20;
                int end = json.indexOf(']', start);
                String[] parts = json.substring(start, end).split(",");
                double[] out = new double[6];
                for (int j = 0; j < 6; j++) out[j] = Double.parseDouble(parts[j]);
                return out;
            } catch (Exception e) {
                return new double[0];
            }
        }
    }
