#include "CYdLidar.h"
#include "common.h"
#include <map>

using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 214285;
  m_Intensities       = true;
  m_FixedResolution   = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  node_counts         = 720;
  each_angle          = 0.5;
  m_IgnoreArray.clear();

}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);

  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    tim_scan_start = nodes[0].stamp;
    tim_scan_end   = nodes[count - 1].stamp;
    float scan_time = (int(tim_scan_end - tim_scan_start)) * 1.0 / 1e9;

    double min_angle = (float)((nodes[0].angle_q6_checkbit >>
                                LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

    double max_angle = (float)((nodes[count - 1].angle_q6_checkbit >>
                                LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

    if (max_angle < min_angle) {
      if ((min_angle > 270 ) && (max_angle < 90 )) {
        outscan.config.ang_increment = angles::from_degrees((float)((
                                         360 + max_angle - min_angle) / ((count - 1) * 1.0)));
      } else {
        outscan.config.ang_increment = angles::from_degrees((float)((
                                         max_angle - min_angle) / ((count - 1) * 1.0)));
      }
    }

    min_angle = angles::from_degrees(min_angle);
    min_angle = angles::normalize_angle_positive(min_angle);
    min_angle = 2 * M_PI - min_angle;
    min_angle = angles::normalize_angle(min_angle);

    max_angle = angles::from_degrees(max_angle);
    max_angle = angles::normalize_angle_positive(max_angle);
    max_angle = 2 * M_PI - max_angle;
    max_angle = angles::normalize_angle(max_angle);

    if(min_angle > max_angle) {
	double tmp = min_angle;
	min_angle = max_angle;
        max_angle = tmp;
    }


    outscan.config.min_angle = min_angle;
    outscan.config.max_angle = max_angle;
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;

    outscan.ranges.resize(count);
    outscan.intensities.resize(count);
    float intensity = 0.0;

    for (int i = 0; i < count; i++) {
      float angle = (float)((nodes[i].angle_q6_checkbit >>
                             LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
      float range = (float)nodes[i].distance_q / 1000.f;
      uint8_t intensities = (uint8_t)(nodes[i].sync_quality >>
                                      LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      intensity = (float)intensities;
      angle = angles::from_degrees(angle);
      angle = angles::normalize_angle_positive(angle);
      angle = 2 * M_PI - angle;
      angle = angles::normalize_angle(angle);

      if (m_GlassNoise && intensities == GLASSNOISEINTENSITY) {
        intensity = 0.0;
        range     = 0.0;
      }

      if (m_SunNoise && intensities == SUNNOISEINTENSITY) {
        intensity = 0.0;
        range     = 0.0;
      }

      if (range > m_MaxRange || range < m_MinRange) {
        range = 0.0;
        intensity = 0.0;
      }

      int index = (angle - min_angle) / outscan.config.ang_increment + 0.5;

      if (index >= 0 && index < count) {
        outscan.ranges[i] = range;
        outscan.intensities[i] = intensity;
      }

      if (tim_scan_start > nodes[i].stamp) {
        tim_scan_start = nodes[i].stamp;
      }

    }

    if (scan_time < 0) {
      scan_time = 1.0 / 5000 * (count - 1);
    }

    outscan.system_time_stamp = tim_scan_start;
    outscan.self_time_stamp = tim_scan_start;
    outscan.config.time_increment = scan_time / (count - 1);
    outscan.config.scan_time = scan_time;


    return true;
  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
      //this->disconnect();
    }
  }

  return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  node_info nodes[2048];
  size_t   count = _countof(nodes);
  result_t op_result =  lidarPtr->grabScanData(nodes, count);

  if (IS_OK(op_result)) {
    return false;
  }

  op_result =  lidarPtr->grabScanData(nodes, count);
  return !IS_OK(op_result);
}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  bool ret = false;
  std::string buffer;
  buffer.clear();
  size_t size = 0;
  result_t op_result = lidarPtr->getDeviceInfo(buffer, size);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
  }

  std::vector<std::string> result;
  std::string delim = "EAI&TEMI]:";
  ydlidar::split(buffer, result, delim);

  for (int i = 0; i < result.size(); i++) {
    std::string string_buf = result[i];
    string_buf = ydlidar::trim(string_buf);
    delim = '!';
    string_buf.erase(string_buf.find_last_not_of(delim) + 1);
    delim = "\n";
    string_buf.erase(string_buf.find_last_not_of(delim) + 1);
    delim = "[";
    string_buf.erase(string_buf.find_last_not_of(delim) + 1);
    delim = "EAI&TEMI]:";
    string_buf.erase(string_buf.find_last_not_of(delim) + 1);
    fprintf(stdout, "match: %s\n", string_buf.c_str());
    fflush(stdout);
    ret = true;
  }

  return ret;
}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  getDeviceInfo();
  lidarPtr->setIntensities(m_Intensities);
  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  if (!checkCOMMs()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
    fflush(stderr);
    return false;
  }

  if (!turnOn()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR Turn ON.\n");
    fflush(stderr);
  }

  return true;
}
