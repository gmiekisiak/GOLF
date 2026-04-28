#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <bluefruit.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define BNO_CS     7
#define FLASH_CS   6
#define BNO_INTN   3
#define BNO_NRST   1

BNO080 imu;
BLEUart bleuart;

// ---------- Tuning ----------
const float GYRO_START_THRESH      = 1.5f;
const float GYRO_END_THRESH        = 0.5f;
const int   START_CONFIRM          = 5;
const int   END_CONFIRM            = 80;
const uint32_t HEARTBEAT_PERIOD_MS = 2000;

// Reference vector in BODY frame — represents "where the wrist is relative to
// the device's reference orientation." Any non-zero vector works; choose one
// that's not parallel to the typical rotation axis. Body X (forearm long axis)
// would be degenerate for shaft rotation. Body Y or Z is fine.
// Using body Z = (0, 0, 1) — points "up" out of the device when flat.
const float REF_X = 0.0f;
const float REF_Y = 0.0f;
const float REF_Z = 1.0f;

// Transition detection on accel-magnitude (real swings have clear peaks)
const float TRANSITION_PEAK_MIN    = 8.0f;
const int   MIN_PHASE_SAMPLES      = 25;

// ---------- Buffers ----------
const int MAX_SWING_SAMPLES = 700;

struct Quat { float w, x, y, z; };
Quat quat_buf[MAX_SWING_SAMPLES];

// Cached accel magnitudes for transition detection (avoid re-rotating)
float a_mag_buf[MAX_SWING_SAMPLES];

// World-frame trajectory points — generated at swing end
float px_buf[MAX_SWING_SAMPLES];
float py_buf[MAX_SWING_SAMPLES];
float pz_buf[MAX_SWING_SAMPLES];

int sample_count = 0;
int transition_index = -1;

uint8_t back_ratios[MAX_SWING_SAMPLES];
uint8_t down_ratios[MAX_SWING_SAMPLES];
int back_count = 0, down_count = 0;

float n_back_x = 0, n_back_y = 0, n_back_z = 1;
float n_down_x = 0, n_down_y = 0, n_down_z = 1;

uint32_t swing_start_us = 0;
uint32_t back_dur_ms = 0, down_dur_ms = 0;

// ---------- State ----------
enum State { IDLE, TRACKING };
State state = IDLE;
int start_count = 0, end_count = 0;
uint32_t last_heartbeat_ms = 0;

// ---------- Quaternion -> rotation of vector ----------
// p_world = q * v_body * q_conj
void rotate_vec_by_quat(const Quat &q, float vx, float vy, float vz,
                        float &wx, float &wy, float &wz) {
  float tx = 2.0f * (q.y*vz - q.z*vy);
  float ty = 2.0f * (q.z*vx - q.x*vz);
  float tz = 2.0f * (q.x*vy - q.y*vx);
  wx = vx + q.w*tx + (q.y*tz - q.z*ty);
  wy = vy + q.w*ty + (q.z*tx - q.x*tz);
  wz = vz + q.w*tz + (q.x*ty - q.y*tx);
}

float plane_tilt_from_horizontal_deg(float nx, float ny, float nz) {
  (void)nx; (void)ny;
  float v = fabsf(nz);
  if (v > 1.0f) v = 1.0f;
  return acosf(v) * 180.0f / PI;
}

// ---------- 3x3 symmetric eigendecomposition (closed form) ----------
void eig3sym(float Cxx, float Cyy, float Czz,
             float Cxy, float Cxz, float Cyz,
             float lambda[3], float v[3][3]) {
  float p1 = Cxy*Cxy + Cxz*Cxz + Cyz*Cyz;
  if (p1 < 1e-20f) {
    lambda[0] = Cxx; lambda[1] = Cyy; lambda[2] = Czz;
    v[0][0]=1; v[1][0]=0; v[2][0]=0;
    v[0][1]=0; v[1][1]=1; v[2][1]=0;
    v[0][2]=0; v[1][2]=0; v[2][2]=1;
    return;
  }
  float q = (Cxx + Cyy + Czz) / 3.0f;
  float p2 = (Cxx-q)*(Cxx-q) + (Cyy-q)*(Cyy-q) + (Czz-q)*(Czz-q) + 2.0f*p1;
  float p = sqrtf(p2 / 6.0f);
  float Bxx = (Cxx-q)/p, Byy = (Cyy-q)/p, Bzz = (Czz-q)/p;
  float Bxy = Cxy/p, Bxz = Cxz/p, Byz = Cyz/p;
  float detB = Bxx*(Byy*Bzz - Byz*Byz) - Bxy*(Bxy*Bzz - Byz*Bxz) + Bxz*(Bxy*Byz - Byy*Bxz);
  float r = detB / 2.0f;
  if (r < -1.0f) r = -1.0f;
  if (r >  1.0f) r =  1.0f;
  float phi = acosf(r) / 3.0f;
  lambda[0] = q + 2.0f*p*cosf(phi);
  lambda[2] = q + 2.0f*p*cosf(phi + 2.0f*PI/3.0f);
  lambda[1] = 3.0f*q - lambda[0] - lambda[2];

  for (int k = 0; k < 3; k++) {
    float L = lambda[k];
    float r1x = Cxx-L, r1y = Cxy,   r1z = Cxz;
    float r2x = Cxy,   r2y = Cyy-L, r2z = Cyz;
    float r3x = Cxz,   r3y = Cyz,   r3z = Czz-L;
    float c1x = r1y*r2z - r1z*r2y;
    float c1y = r1z*r2x - r1x*r2z;
    float c1z = r1x*r2y - r1y*r2x;
    float c1m = c1x*c1x + c1y*c1y + c1z*c1z;
    float c2x = r1y*r3z - r1z*r3y;
    float c2y = r1z*r3x - r1x*r3z;
    float c2z = r1x*r3y - r1y*r3x;
    float c2m = c2x*c2x + c2y*c2y + c2z*c2z;
    float c3x = r2y*r3z - r2z*r3y;
    float c3y = r2z*r3x - r2x*r3z;
    float c3z = r2x*r3y - r2y*r3x;
    float c3m = c3x*c3x + c3y*c3y + c3z*c3z;
    float ex, ey, ez, em;
    if (c1m >= c2m && c1m >= c3m) { ex=c1x; ey=c1y; ez=c1z; em=c1m; }
    else if (c2m >= c3m)          { ex=c2x; ey=c2y; ez=c2z; em=c2m; }
    else                          { ex=c3x; ey=c3y; ez=c3z; em=c3m; }
    if (em < 1e-20f) {
      ex = (k==0)?1:0; ey = (k==1)?1:0; ez = (k==2)?1:0; em = 1;
    }
    float inv = 1.0f / sqrtf(em);
    v[0][k] = ex*inv; v[1][k] = ey*inv; v[2][k] = ez*inv;
  }
}

// Generate world-frame trajectory points by rotating REF vector through each quaternion.
void generate_trajectory() {
  for (int i = 0; i < sample_count; i++) {
    rotate_vec_by_quat(quat_buf[i], REF_X, REF_Y, REF_Z,
                       px_buf[i], py_buf[i], pz_buf[i]);
  }
}

// Fit plane to trajectory points [start, end_idx) via PCA.
// Plane normal = smallest-eigenvalue eigenvector of point covariance.
// Trajectory points are smooth (quaternion-derived) so PCA is stable here.
bool fit_plane_to_trajectory(int start, int end_idx, float &nx, float &ny, float &nz,
                             float &lambda_min, float &lambda_max) {
  if (end_idx - start < MIN_PHASE_SAMPLES) return false;
  float mx = 0, my = 0, mz = 0;
  int n = end_idx - start;
  for (int i = start; i < end_idx; i++) {
    mx += px_buf[i]; my += py_buf[i]; mz += pz_buf[i];
  }
  mx /= n; my /= n; mz /= n;

  float Cxx=0, Cyy=0, Czz=0, Cxy=0, Cxz=0, Cyz=0;
  for (int i = start; i < end_idx; i++) {
    float dx = px_buf[i] - mx;
    float dy = py_buf[i] - my;
    float dz = pz_buf[i] - mz;
    Cxx += dx*dx; Cyy += dy*dy; Czz += dz*dz;
    Cxy += dx*dy; Cxz += dx*dz; Cyz += dy*dz;
  }
  float invN = 1.0f / n;
  Cxx *= invN; Cyy *= invN; Czz *= invN;
  Cxy *= invN; Cxz *= invN; Cyz *= invN;

  float lambda[3];
  float v[3][3];
  eig3sym(Cxx, Cyy, Czz, Cxy, Cxz, Cyz, lambda, v);

  // Smallest eigenvector = plane normal (direction of least spread)
  nx = v[0][2]; ny = v[1][2]; nz = v[2][2];
  lambda_min = lambda[2];
  lambda_max = lambda[0];
  return true;
}

// Compute per-sample contamination as perpendicular distance of trajectory point
// from plane, normalized by trajectory spread (to give a unitless ratio).
int compute_ratios(int start, int end_idx,
                   float nx, float ny, float nz, uint8_t *out) {
  // Compute centroid for plane offset and a typical spread for normalization.
  float mx = 0, my = 0, mz = 0;
  int n = end_idx - start;
  if (n <= 0) return 0;
  for (int i = start; i < end_idx; i++) {
    mx += px_buf[i]; my += py_buf[i]; mz += pz_buf[i];
  }
  mx /= n; my /= n; mz /= n;

  // Spread: average distance of points from centroid (in-plane scale)
  float total_spread = 0;
  for (int i = start; i < end_idx; i++) {
    float dx = px_buf[i] - mx;
    float dy = py_buf[i] - my;
    float dz = pz_buf[i] - mz;
    total_spread += sqrtf(dx*dx + dy*dy + dz*dz);
  }
  float avg_spread = total_spread / n;
  if (avg_spread < 1e-6f) avg_spread = 1.0f;

  int written = 0;
  for (int i = start; i < end_idx && written < MAX_SWING_SAMPLES; i++) {
    float dx = px_buf[i] - mx;
    float dy = py_buf[i] - my;
    float dz = pz_buf[i] - mz;
    // Perpendicular distance to plane through centroid with normal n
    float d_perp = fabsf(dx*nx + dy*ny + dz*nz);
    // Ratio = perpendicular distance / typical in-plane distance
    float ratio = d_perp / avg_spread;
    if (ratio > 1.0f) ratio = 1.0f;
    out[written++] = (uint8_t)(ratio * 255.0f);
  }
  return written;
}

// Find transition by accel-magnitude minimum between peaks
int find_transition() {
  int peak1 = -1;
  for (int i = 0; i < sample_count; i++) {
    if (a_mag_buf[i] > TRANSITION_PEAK_MIN) { peak1 = i; break; }
  }
  if (peak1 < 0) return -1;
  int peak2 = -1;
  for (int i = sample_count - 1; i > peak1; i--) {
    if (a_mag_buf[i] > TRANSITION_PEAK_MIN) { peak2 = i; break; }
  }
  if (peak2 < 0 || peak2 - peak1 < MIN_PHASE_SAMPLES * 2) return -1;
  float min_val = 1e9f;
  int min_idx = -1;
  for (int i = peak1 + MIN_PHASE_SAMPLES/2; i < peak2 - MIN_PHASE_SAMPLES/2; i++) {
    if (a_mag_buf[i] < min_val) { min_val = a_mag_buf[i]; min_idx = i; }
  }
  if (min_idx < 0) return -1;
  if (min_idx < MIN_PHASE_SAMPLES) return -1;
  if (sample_count - min_idx < MIN_PHASE_SAMPLES) return -1;
  return min_idx;
}

// ---------- BLE ----------
void send_swing_packet() {
  uint8_t hdr_buf[1 + 4 + 4 + 2 + 2 + 2 + 2];
  int o = 0;
  hdr_buf[o++] = 0xAA;
  float a_back = plane_tilt_from_horizontal_deg(n_back_x, n_back_y, n_back_z);
  float a_down = plane_tilt_from_horizontal_deg(n_down_x, n_down_y, n_down_z);
  memcpy(hdr_buf + o, &a_back, 4); o += 4;
  memcpy(hdr_buf + o, &a_down, 4); o += 4;
  uint16_t bms = (uint16_t)min((uint32_t)65535, back_dur_ms);
  uint16_t dms = (uint16_t)min((uint32_t)65535, down_dur_ms);
  memcpy(hdr_buf + o, &bms, 2); o += 2;
  memcpy(hdr_buf + o, &dms, 2); o += 2;
  uint16_t nb = back_count, nd = down_count;
  memcpy(hdr_buf + o, &nb, 2); o += 2;
  memcpy(hdr_buf + o, &nd, 2); o += 2;
  bleuart.write(hdr_buf, o);
  delay(15);

  const int CHUNK = 60;
  for (int i = 0; i < back_count; i += CHUNK) {
    int n = min(CHUNK, back_count - i);
    bleuart.write(back_ratios + i, n);
    delay(15);
  }
  for (int i = 0; i < down_count; i += CHUNK) {
    int n = min(CHUNK, down_count - i);
    bleuart.write(down_ratios + i, n);
    delay(15);
  }
}

void send_heartbeat() {
  uint8_t hb[] = {0xBB, 0xBB};
  bleuart.write(hb, 2);
}

void start_advertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void connect_callback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (conn) {
    conn->requestConnectionParameter(24, 0, 400);
    conn->requestPHY();
    conn->requestMtuExchange(247);
    conn->requestDataLengthUpdate();
  }
}

void setup() {
  pinMode(FLASH_CS, OUTPUT); digitalWrite(FLASH_CS, HIGH);
  SPI.begin();
  imu.beginSPI(BNO_CS, 255, BNO_INTN, BNO_NRST, 3000000, SPI);
  delay(200);
  imu.enableLinearAccelerometer(1);
  imu.enableGyro(1);
  imu.enableGameRotationVector(1);
  delay(100);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName("Nimbrace");
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnInterval(16, 32);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  bleuart.begin();
  start_advertising();
}

void finalize_and_send() {
  uint32_t total_dur_ms = (micros() - swing_start_us) / 1000;

  // Generate world-frame trajectory by rotating REF vector through each quaternion
  generate_trajectory();

  transition_index = find_transition();
  int back_end = (transition_index > 0) ? transition_index : sample_count;

  float lmin, lmax;
  if (!fit_plane_to_trajectory(0, back_end, n_back_x, n_back_y, n_back_z, lmin, lmax)) {
    n_back_x = 0; n_back_y = 0; n_back_z = 1;
  }

  if (transition_index > 0) {
    if (!fit_plane_to_trajectory(transition_index, sample_count,
                                 n_down_x, n_down_y, n_down_z, lmin, lmax)) {
      n_down_x = n_back_x; n_down_y = n_back_y; n_down_z = n_back_z;
    }
    float dt = (float)total_dur_ms / sample_count;
    back_dur_ms = (uint32_t)(transition_index * dt);
    down_dur_ms = (uint32_t)((sample_count - transition_index) * dt);
  } else {
    n_down_x = n_back_x; n_down_y = n_back_y; n_down_z = n_back_z;
    back_dur_ms = total_dur_ms;
    down_dur_ms = 0;
  }

  back_count = compute_ratios(0, back_end, n_back_x, n_back_y, n_back_z, back_ratios);
  down_count = (transition_index > 0)
             ? compute_ratios(transition_index, sample_count,
                              n_down_x, n_down_y, n_down_z, down_ratios)
             : 0;

  if (bleuart.notifyEnabled()) send_swing_packet();
}

void loop() {
  if (state == IDLE && bleuart.notifyEnabled() &&
      millis() - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
    last_heartbeat_ms = millis();
    send_heartbeat();
  }

  if (!imu.dataAvailable()) {
    delay(1);
    return;
  }

  float ax_b = imu.getLinAccelX(), ay_b = imu.getLinAccelY(), az_b = imu.getLinAccelZ();
  float gx_b = imu.getGyroX(),     gy_b = imu.getGyroY(),     gz_b = imu.getGyroZ();
  Quat q;
  q.w = imu.getQuatReal();
  q.x = imu.getQuatI(); q.y = imu.getQuatJ(); q.z = imu.getQuatK();
  // Normalize quaternion
  float qm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (qm > 1e-6f) { q.w /= qm; q.x /= qm; q.y /= qm; q.z /= qm; }

  float a_mag = sqrtf(ax_b*ax_b + ay_b*ay_b + az_b*az_b);
  float g_mag = sqrtf(gx_b*gx_b + gy_b*gy_b + gz_b*gz_b);

  switch (state) {
    case IDLE:
      if (g_mag > GYRO_START_THRESH) {
        if (++start_count >= START_CONFIRM) {
          start_count = 0;
          sample_count = 0;
          transition_index = -1;
          back_count = down_count = 0;
          swing_start_us = micros();
          if (sample_count < MAX_SWING_SAMPLES) {
            quat_buf[sample_count] = q;
            a_mag_buf[sample_count] = a_mag;
            sample_count++;
          }
          state = TRACKING;
          end_count = 0;
        }
      } else {
        start_count = 0;
      }
      break;

    case TRACKING:
      if (sample_count < MAX_SWING_SAMPLES) {
        quat_buf[sample_count] = q;
        a_mag_buf[sample_count] = a_mag;
        sample_count++;
      }
      if (g_mag < GYRO_END_THRESH) {
        if (++end_count >= END_CONFIRM) {
          finalize_and_send();
          state = IDLE;
          end_count = 0;
          last_heartbeat_ms = millis();
        }
      } else {
        end_count = 0;
      }
      break;
  }
}
