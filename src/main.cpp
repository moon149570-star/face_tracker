// Linux / 久久派 2K0300：USB摄像头人脸检测 -> STM32协议v1。
// 编译需链接 OpenCV core/videoio/imgproc/imgcodecs/objdetect 和 pthread。
// 用法: ./camera_probe [串口] [摄像头] [人脸级联XML] [可选调试图片.jpg] [--preview]
// --preview通过局域网提供640x480 MJPEG预览，默认8080端口/15FPS/镜像/一个观看连接。
// --preview-fps 1..30 设置预览帧率上限；--no-mirror显示摄像头原始方向。
// 当前云台安装方向：串口x=width-1-原始x，y=height-1-原始y，独立于预览镜像开关。
// 自适应ROI逐帧重新检测人脸，丢失后扩大并回退全图；--capture-fps可调整采集请求。
// --fixed-fps尝试关闭摄像头的曝光优先变帧率，仍保留自动曝光；摄像头需支持此控制。
// 默认: /dev/ttyS1 /dev/video0；XML可放在当前目录或OpenCV标准数据目录。
// 检测的是人脸位置，不进行人员身份识别；关联使用位置和大小，遮挡/交叉时不能保证身份。
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <stdexcept>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;
static constexpr int FRAME_WIDTH = 640;
static constexpr int FRAME_HEIGHT = 480;
static constexpr int CAPTURE_FPS = 30;
static constexpr int CAPTURE_BUFFERS = 2;
static constexpr int MAX_RESULT_AGE_MS = 150;
static volatile std::sig_atomic_t quit = 0;
static void stop(int) { quit = 1; }

static uint16_t crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static std::array<uint8_t, 16> packet(uint8_t seq, bool face,
                                     uint16_t x, uint16_t y,
                                     uint16_t width, uint16_t height)
{
    std::array<uint8_t, 16> p{};
    p[0] = 0xA3; p[1] = 0xB3; p[2] = 1; p[3] = seq; p[4] = face ? 1 : 0;
    auto put = [&](int offset, uint16_t v) {
        p[offset] = static_cast<uint8_t>(v);
        p[offset + 1] = static_cast<uint8_t>(v >> 8);
    };
    put(6, face ? x : 0); put(8, face ? y : 0);
    put(10, width); put(12, height); put(14, crc16(p.data() + 2, 12));
    return p;
}

static int openSerial(const char* device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;
    termios tty{};
    if (tcgetattr(fd, &tty) == 0) {
        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
        tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | CLOCAL | CREAD;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &tty) == 0 && tcflush(fd, TCIOFLUSH) == 0)
            return fd;
    }
    const int error = errno;
    close(fd); errno = error;
    return -1;
}

// 不积压发送队列；单帧限时20ms，失败后退出，绝不在半帧中插入下一帧。
static bool sendPacket(int fd, const std::array<uint8_t, 16>& p)
{
    const auto deadline = Clock::now() + std::chrono::milliseconds(20);
    size_t offset = 0;
    while (offset < p.size()) {
        if (Clock::now() >= deadline) return false;
        const ssize_t n = write(fd, p.data() + offset, p.size() - offset);
        if (n > 0) { offset += static_cast<size_t>(n); continue; }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        pollfd item{fd, POLLOUT, 0};
        int result = poll(&item, 1, 2);
        if (result < 0 && errno != EINTR) return false;
        if (result > 0 && (item.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    }
    return true;
}

static int cameraIoctl(int fd, unsigned long request, void* argument)
{
    int result;
    do { result = ioctl(fd, request, argument); } while (result < 0 && errno == EINTR);
    return result;
}

static void reportCameraControls(const char* device, bool fixedFps)
{
    const int fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        std::cerr << "Camera controls unavailable: " << std::strerror(errno) << '\n';
        return;
    }
    if (fixedFps) {
        v4l2_control control{};
        control.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
        control.value = 0;
        if (cameraIoctl(fd, VIDIOC_S_CTRL, &control) == 0)
            std::cout << "Fixed-FPS request: exposure_auto_priority=0 (auto exposure mode unchanged)\n";
        else
            std::cerr << "Fixed-FPS control unsupported/failed: " << std::strerror(errno) << '\n';
    }
    const uint32_t ids[] = {V4L2_CID_EXPOSURE_AUTO, V4L2_CID_EXPOSURE_ABSOLUTE, V4L2_CID_EXPOSURE_AUTO_PRIORITY};
    const char* names[] = {"exposure_auto", "exposure_absolute", "exposure_auto_priority"};
    for (size_t i = 0; i < 3; ++i) {
        v4l2_queryctrl query{};
        query.id = ids[i];
        if (cameraIoctl(fd, VIDIOC_QUERYCTRL, &query) != 0 || (query.flags & V4L2_CTRL_FLAG_DISABLED)) continue;
        v4l2_control control{};
        control.id = ids[i];
        if (cameraIoctl(fd, VIDIOC_G_CTRL, &control) == 0) {
            std::cout << "Camera control: " << names[i] << '=' << control.value
                      << " range=" << query.minimum << ".." << query.maximum;
            if (ids[i] == V4L2_CID_EXPOSURE_ABSOLUTE) std::cout << " (100us units)";
            std::cout << '\n';
        }
    }
    close(fd);
}

struct DetectionOverlay {
    cv::Mat image;
    std::vector<cv::Rect> faces;
    int selected = -1;
    std::string reason = "WAIT_DETECTION";
    Clock::time_point acquired{};
    uint64_t generation = 0;
    int64_t resultAgeMs = -1; // 主线程决定是否可发送时的结果年龄，预览不重新判定。
    double detectFps = 0.0;
    double validTxFps = 0.0;
    bool localMode = false;
    double captureFps = 0.0;
};

struct FrameRates {
    Clock::time_point since = Clock::now();
    unsigned detected = 0, validSent = 0;
    double detectFps = 0.0, validTxFps = 0.0;

    void record(bool sentValidFace) { ++detected; if (sentValidFace) ++validSent; }
    void update(Clock::time_point now)
    {
        const double elapsed = std::chrono::duration<double>(now - since).count();
        if (elapsed < 1.0) return;
        detectFps = detected / elapsed;
        validTxFps = validSent / elapsed;
        detected = validSent = 0;
        since = now;
    }
};

struct LatestFrame {
    std::mutex mutex;
    cv::Mat image;
    Clock::time_point acquired{};
    uint64_t generation = 0;
    DetectionOverlay detection; // 与检测框对应的那张原始图像，受mutex保护。
    double captureFps = 0.0; // 按成功read的帧数/真实经过时间统计，非驱动声明的FPS。
    double grabMs = 0.0, decodeMs = 0.0, decodeCpuMs = 0.0;
    std::atomic<bool> failed{false};
    std::atomic<bool> stopping{false};
};

static int64_t threadCpuNs()
{
    timespec value{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return 0;
    return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

static cv::Rect previewRect(const cv::Rect& box, int width, bool mirror)
{
    return cv::Rect(mirror ? width - box.x - box.width : box.x,
                    box.y, box.width, box.height);
}

static cv::Point previewPoint(const cv::Point& point, int width, bool mirror)
{
    return cv::Point(mirror ? width - 1 - point.x : point.x, point.y);
}

static cv::Point faceCoordinate(const cv::Rect& box, int width, int height)
{
    return cv::Point(std::clamp(cvRound(box.x + box.width * 0.5), 0, width - 1),
                     std::clamp(cvRound(box.y + box.height * 0.5), 0, height - 1));
}

// 适配用户实物的两轴转向；在串口发送前仅反转一次，预览开关不参与此计算。
static cv::Point serialPoint(const cv::Point& point, int width, int height)
{
    return cv::Point(width - 1 - point.x, height - 1 - point.y);
}

static std::string fpsText(double fps)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(1) << fps;
    return text.str();
}

// 图像、框和质心一起镜像；TX字段显示实际发送给STM32的坐标。
static cv::Mat drawPreview(const DetectionOverlay& detection, bool mirror,
                           double fps, int targetFps)
{
    cv::Mat image;
    if (mirror) cv::flip(detection.image, image, 1);
    else image = detection.image.clone();
    const auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - detection.acquired).count();
    // FACE/HOLD描述这张图像的检测/发送结果；显示延迟只显示在view_age中。
    const int selected = detection.selected;
    const std::string& reason = detection.reason;
    for (size_t i = 0; i < detection.faces.size(); ++i) {
        const cv::Rect box = previewRect(detection.faces[i], image.cols, mirror);
        const cv::Scalar color = static_cast<int>(i) == selected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
        cv::rectangle(image, box, color, 2);
        cv::putText(image, "FACE" + std::to_string(i + 1), cv::Point(box.x, std::max(94, box.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
        if (static_cast<int>(i) == selected) {
            const cv::Point point = faceCoordinate(detection.faces[i], image.cols, image.rows);
            cv::circle(image, previewPoint(point, image.cols, mirror), 4, color, -1);
        }
    }
    cv::drawMarker(image, cv::Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2),
                   cv::Scalar(255, 180, 0), cv::MARKER_CROSS, 18, 1);
    const std::string state = selected >= 0 ? "FACE" + std::to_string(selected + 1) :
                              reason == "NO_FACE" ? "NOFACE" : "HOLD: " + reason;
    cv::rectangle(image, cv::Rect(0, 0, image.cols, 80), cv::Scalar(0, 0, 0), -1);
    std::string status = state + "   faces=" + std::to_string(detection.faces.size());
    if (selected >= 0) {
        const cv::Point tx = serialPoint(faceCoordinate(detection.faces[selected], image.cols, image.rows), image.cols, image.rows);
        status += "   TX=(" + std::to_string(tx.x) + "," + std::to_string(tx.y) + ")";
    }
    cv::putText(image, status, cv::Point(8, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, selected >= 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 160, 255), 2);
    std::string rates = "CAP " + fpsText(detection.captureFps) + "   DET " + fpsText(detection.detectFps) +
                        "   VALID_TX " + fpsText(detection.validTxFps);
    if (targetFps > 0) rates = "VIEW " + fpsText(fps) + "/" + std::to_string(targetFps) + "   " + rates;
    rates += detection.localMode ? "   ROI" : "   GLOBAL";
    cv::putText(image, rates, cv::Point(8, 46), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    std::string info = "view_age=" + std::to_string(ageMs) + "ms";
    if (detection.resultAgeMs >= 0) info = "result_age=" + std::to_string(detection.resultAgeMs) + "ms  " + info;
    info += mirror ? "   MIRROR" : "   ORIGINAL";
    cv::putText(image, info, cv::Point(8, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    return image;
}

// 预览线程按需编码最新图像，慢客户端不会阻塞采集/串口，不缓存历史图像。
class MjpegPreview {
public:
    MjpegPreview(LatestFrame& latest, int port, int fps = 15, bool mirror = true)
        : latest_(latest), targetFps_(fps), mirror_(mirror)
    {
        listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (listener_ < 0) throw std::runtime_error("Cannot create preview socket");
        int reuse = 1;
        setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(port));
        if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
            listen(listener_, 4) != 0) {
            const std::string message = "Cannot start preview on port " + std::to_string(port) + ": " + std::strerror(errno);
            close(listener_); listener_ = -1;
            throw std::runtime_error(message);
        }
        try { worker_ = std::thread(&MjpegPreview::run, this); }
        catch (...) { close(listener_); listener_ = -1; throw; }
        std::cout << "Browser preview: http://<board-IP>:" << port
                  << "/ (640x480, target " << targetFps_ << " FPS, "
                  << (mirror_ ? "mirror" : "original") << ", one viewer)" << std::endl;
    }

    ~MjpegPreview()
    {
        stopping_ = true;
        if (worker_.joinable()) worker_.join();
        if (listener_ >= 0) close(listener_);
    }

private:
    bool running() const { return !stopping_ && !latest_.stopping && !latest_.failed && !quit; }

    bool sendAll(int fd, const void* data, size_t length, Clock::time_point deadline)
    {
        const auto* bytes = static_cast<const uint8_t*>(data);
        size_t offset = 0;
        while (running() && offset < length && Clock::now() < deadline) {
            const ssize_t count = send(fd, bytes + offset, length - offset, MSG_NOSIGNAL);
            if (count > 0) { offset += static_cast<size_t>(count); continue; }
            if (count < 0 && errno == EINTR) continue;
            if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) return false;
            pollfd item{fd, POLLOUT, 0};
            const int result = poll(&item, 1, 10);
            if (result < 0 && errno != EINTR) return false;
            if (result > 0 && (item.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
        }
        return offset == length;
    }

    void serve(int fd)
    {
        // 限制请求大小/等待时间，避免半个HTTP请求长期占住预览线程。
        std::string request;
        const auto requestDeadline = Clock::now() + std::chrono::seconds(1);
        while (running() && request.size() < 4096 && Clock::now() < requestDeadline) {
            char buffer[512];
            const ssize_t count = recv(fd, buffer, sizeof(buffer), 0);
            if (count > 0) {
                request.append(buffer, static_cast<size_t>(count));
                if (request.find("\r\n\r\n") != std::string::npos) break;
            } else if (count == 0) return;
            else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return;
            else {
                pollfd item{fd, POLLIN, 0};
                poll(&item, 1, 20);
            }
        }
        if (request.find("\r\n\r\n") == std::string::npos) return;
        if (request.compare(0, 6, "GET / ") != 0 && request.compare(0, 12, "GET /stream ") != 0) {
            const std::string response = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            sendAll(fd, response.data(), response.size(), Clock::now() + std::chrono::milliseconds(200));
            return;
        }
        const std::string header = "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-store, no-cache\r\nConnection: close\r\n\r\n";
        if (!sendAll(fd, header.data(), header.size(), Clock::now() + std::chrono::milliseconds(200))) return;
        uint64_t used = 0;
        auto nextFrame = Clock::now();
        auto fpsStart = Clock::now();
        unsigned sentFrames = 0;
        double actualFps = 0.0;
        const auto frameInterval = std::chrono::microseconds(1000000 / targetFps_);
        while (running()) {
            // 即使摄像头暂时无新帧，也检测观看端关闭，允许重新连接。
            char byte;
            const ssize_t peek = recv(fd, &byte, 1, MSG_PEEK);
            if (peek == 0 || (peek < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return;
            DetectionOverlay overlay;
            if (Clock::now() >= nextFrame) {
                std::lock_guard<std::mutex> guard(latest_.mutex);
                // 框和对应的检测图像一起取出，避免把旧框画到更新的另一帧上。
                const bool haveDetection = latest_.detection.generation != 0 &&
                    Clock::now() - latest_.detection.acquired <= std::chrono::milliseconds(300);
                const uint64_t generation = haveDetection ? latest_.detection.generation : latest_.generation;
                const auto acquired = haveDetection ? latest_.detection.acquired : latest_.acquired;
                const uint64_t key = (generation << 1) | (haveDetection ? 0 : 1);
                if (generation != 0 && key != used && Clock::now() - acquired <= std::chrono::milliseconds(300)) {
                    used = key;
                    if (haveDetection) overlay = latest_.detection;
                    else {
                        overlay.image = latest_.image;
                        overlay.acquired = acquired;
                        overlay.generation = generation;
                        overlay.captureFps = latest_.captureFps;
                    }
                }
            }
            if (overlay.image.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const auto frameStarted = Clock::now();
            const int64_t cpuStarted = threadCpuNs();
            const cv::Mat image = drawPreview(overlay, mirror_, actualFps, targetFps_);
            std::vector<uint8_t> jpeg;
            if (!cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 60})) return;
            const std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     std::to_string(jpeg.size()) + "\r\n\r\n";
            const auto deadline = Clock::now() + std::chrono::milliseconds(200);
            if (!sendAll(fd, part.data(), part.size(), deadline) ||
                !sendAll(fd, jpeg.data(), jpeg.size(), deadline) ||
                !sendAll(fd, "\r\n", 2, deadline)) return;
            // 预览按自身CPU耗时自动降帧，约15%的软预算；不靠放宽检测有效期维持帧率。
            // 使用线程CPU时间，网络等待不会被误认为CPU开销。
            const int64_t cpuUsed = std::max<int64_t>(0, threadCpuNs() - cpuStarted);
            const auto budgetInterval = std::chrono::nanoseconds(cpuUsed * 100 / 15);
            nextFrame = frameStarted + std::max(std::chrono::duration_cast<Clock::duration>(frameInterval),
                                                std::chrono::duration_cast<Clock::duration>(budgetInterval));
            ++sentFrames;
            const double elapsed = std::chrono::duration<double>(Clock::now() - fpsStart).count();
            if (elapsed >= 1.0) {
                actualFps = sentFrames / elapsed;
                sentFrames = 0;
                fpsStart = Clock::now();
            }
        }
    }

    void run()
    {
        // Linux线程nice值独立，降低预览优先级，让采集/检测/串口先运行。
        if (setpriority(PRIO_PROCESS, 0, 5) != 0)
            std::cerr << "Cannot lower preview priority: " << std::strerror(errno) << '\n';
        while (running()) {
            pollfd item{listener_, POLLIN, 0};
            const int ready = poll(&item, 1, 100);
            if (ready < 0 && errno == EINTR) continue;
            if (ready < 0 || (ready > 0 && (item.revents & (POLLERR | POLLHUP | POLLNVAL)))) break;
            if (ready == 0) continue;
            const int client = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (client < 0) continue;
            // 限制TCP缓冲，发送超时即断开慢观看端，不积累旧画面。
            int sendBuffer = 32 * 1024;
            setsockopt(client, SOL_SOCKET, SO_SNDBUF, &sendBuffer, sizeof(sendBuffer));
            try { serve(client); }
            catch (const std::exception& e) { std::cerr << "Preview client error: " << e.what() << '\n'; }
            close(client);
        }
    }

    LatestFrame& latest_;
    int targetFps_;
    bool mirror_;
    int listener_ = -1;
    std::atomic<bool> stopping_{false};
    std::thread worker_;
};

static cv::Point2d center(const cv::Rect& box)
{
    return {box.x + box.width * 0.5, box.y + box.height * 0.5};
}

// 首次选择最大脸；有目标时按重叠/距离/大小关联，丢失600ms后允许重新选人。
static int selectFace(const std::vector<cv::Rect>& faces, bool locked, const cv::Rect& previous)
{
    int selected = -1;
    double best = locked ? 1e9 : -1.0;
    for (size_t i = 0; i < faces.size(); ++i) {
        const cv::Rect& box = faces[i];
        if (!locked) {
            if (box.area() > best) { best = box.area(); selected = static_cast<int>(i); }
            continue;
        }
        double ratio = double(box.area()) / previous.area();
        double distance = cv::norm(center(box) - center(previous));
        double scale = std::max(previous.width, previous.height);
        double intersection = (box & previous).area();
        double iou = intersection / (box.area() + previous.area() - intersection);
        if (ratio < 0.4 || ratio > 2.5 || (iou < 0.1 && distance > scale * 0.8)) continue;
        double score = distance / scale + std::abs(std::log(ratio)) * 0.5 - iou;
        if (score < best) { best = score; selected = static_cast<int>(i); }
    }
    return selected;
}

struct DetectionPlan {
    cv::Rect region;
    cv::Size resized;
    cv::Size minFace, maxFace;
    double scaleFactor = 1.2;
    bool local = false;
};

static DetectionPlan planDetection(cv::Size frame, const cv::Rect& previous,
                                   bool locked, unsigned misses, cv::Size baseWindow)
{
    DetectionPlan global;
    global.region = cv::Rect(0, 0, frame.width, frame.height);
    global.resized = cv::Size(std::min(240, frame.width),
        std::max(1, cvRound(double(frame.height) * std::min(240, frame.width) / frame.width)));
    global.minFace = baseWindow;
    if (!locked || previous.area() <= 0 || misses >= 3) return global;

    // 漏检逐步扩大范围；一张图只扫描一次，连续3次漏检后下一张图再全局搜索。
    const double expansion = misses == 0 ? 1.8 : misses == 1 ? 2.5 : 3.2;
    const int roiWidth = cvRound(previous.width * expansion);
    const int roiHeight = cvRound(previous.height * expansion);
    DetectionPlan local;
    local.region = cv::Rect(cvRound(previous.x + previous.width * 0.5 - roiWidth * 0.5),
                            cvRound(previous.y + previous.height * 0.5 - roiHeight * 0.5),
                            roiWidth, roiHeight) & global.region;
    if (local.region.width < baseWindow.width || local.region.height < baseWindow.height) return global;
    // 把目标脸约缩至48像素，并限定扫描尺寸在上一张脸的0.75..1.35倍。
    const double scale = std::min({1.0, 48.0 / std::max(previous.width, previous.height),
                                  192.0 / std::max(local.region.width, local.region.height)});
    local.resized = cv::Size(std::max(1, cvRound(local.region.width * scale)),
                             std::max(1, cvRound(local.region.height * scale)));
    local.minFace = cv::Size(std::max(baseWindow.width, cvRound(previous.width * scale * 0.75)),
                             std::max(baseWindow.height, cvRound(previous.height * scale * 0.75)));
    local.maxFace = cv::Size(std::min(local.resized.width, cvRound(previous.width * scale * 1.35)),
                             std::min(local.resized.height, cvRound(previous.height * scale * 1.35)));
    if (local.minFace.width > local.maxFace.width || local.minFace.height > local.maxFace.height) return global;
    local.scaleFactor = 1.12;
    local.local = true;
    return local;
}

static cv::Rect mapDetection(const cv::Rect& detected, const DetectionPlan& plan, cv::Size frame)
{
    const cv::Rect box = detected & cv::Rect(0, 0, plan.resized.width, plan.resized.height);
    if (box.area() <= 0) return cv::Rect();
    const double sx = double(plan.region.width) / plan.resized.width;
    const double sy = double(plan.region.height) / plan.resized.height;
    const int left = plan.region.x + cvRound(box.x * sx);
    const int top = plan.region.y + cvRound(box.y * sy);
    const int right = plan.region.x + cvRound((box.x + box.width) * sx);
    const int bottom = plan.region.y + cvRound((box.y + box.height) * sy);
    return cv::Rect(left, top, right - left, bottom - top) & cv::Rect(0, 0, frame.width, frame.height);
}

struct TargetAssociation {
    bool locked = false;
    cv::Rect previous;
    unsigned misses = 0;
    Clock::time_point lastSeen{};

    void expire(Clock::time_point now)
    {
        if (locked && now - lastSeen > std::chrono::milliseconds(600)) { locked = false; misses = 0; }
    }

    int accept(const std::vector<cv::Rect>& faces, Clock::time_point acquired,
               Clock::time_point now, bool local)
    {
        expire(now);
        const int candidate = selectFace(faces, locked, previous);
        if (candidate >= 0 && now - acquired <= std::chrono::milliseconds(MAX_RESULT_AGE_MS)) {
            previous = faces[candidate]; locked = true; misses = 0; lastSeen = acquired;
            return candidate;
        }
        // 全图搜索慢时，旧框只用来定位下一帧的搜索区域，绝不发送该旧坐标。
        if (!local && !locked && candidate >= 0 && now - acquired <= std::chrono::milliseconds(600)) {
            previous = faces[candidate]; locked = true; misses = 0; lastSeen = acquired;
        } else if (locked && candidate < 0) ++misses;
        return -1;
    }
};

int main(int argc, char** argv)
{
    std::vector<std::string> arguments;
    bool previewEnabled = false;
    int previewPort = 8080;
    int previewFps = 15;
    int captureFps = CAPTURE_FPS;
    bool mirrorPreview = true;
    bool fixedFps = false;
    auto usage = [&] {
        std::cout << "Usage: " << argv[0]
                  << " [serial=/dev/ttyS1] [camera=/dev/video0] [cascade.xml] [debug.jpg]"
                     " [--preview] [--preview-port PORT] [--preview-fps 1..30] [--no-mirror] [--capture-fps 1..30] [--fixed-fps]\n";
    };
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string value = argv[i];
            if (value == "--help") { usage(); return 0; }
            if (value == "--preview") previewEnabled = true;
            else if (value == "--no-mirror") mirrorPreview = false;
            else if (value == "--fixed-fps") fixedFps = true;
            else if (value == "--capture-fps") {
                if (++i >= argc) throw std::runtime_error("Missing capture FPS");
                const std::string fps = argv[i];
                size_t consumed = 0;
                captureFps = std::stoi(fps, &consumed);
                if (consumed != fps.size() || captureFps < 1 || captureFps > 30)
                    throw std::runtime_error("Capture FPS must be 1..30");
            }
            else if (value == "--preview-fps") {
                if (++i >= argc) throw std::runtime_error("Missing preview FPS");
                const std::string fps = argv[i];
                size_t consumed = 0;
                previewFps = std::stoi(fps, &consumed);
                if (consumed != fps.size() || previewFps < 1 || previewFps > 30)
                    throw std::runtime_error("Preview FPS must be 1..30");
                previewEnabled = true;
            }
            else if (value == "--preview-port") {
                if (++i >= argc) throw std::runtime_error("Missing preview port");
                const std::string port = argv[i];
                size_t consumed = 0;
                previewPort = std::stoi(port, &consumed);
                if (consumed != port.size() || previewPort < 1 || previewPort > 65535)
                    throw std::runtime_error("Preview port must be 1..65535");
                previewEnabled = true;
            } else if (value.compare(0, 2, "--") == 0) throw std::runtime_error("Unknown option: " + value);
            else arguments.push_back(value);
        }
        if (arguments.size() > 4) throw std::runtime_error("Too many positional arguments");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; usage(); return 1;
    }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    const char* serial = arguments.size() > 0 ? arguments[0].c_str() : "/dev/ttyS1";
    const char* device = arguments.size() > 1 ? arguments[1].c_str() : "/dev/video0";
    std::string debugImage = arguments.size() > 3 ? arguments[3] : "";
    // 2K0300计算资源有限，避免OpenCV内部并行与采集线程争抢CPU。
    cv::setNumThreads(1);
    cv::CascadeClassifier detector;
    std::vector<std::string> paths;
    if (arguments.size() > 2) paths.emplace_back(arguments[2]);
    else {
        const std::string name = "haarcascade_frontalface_default.xml";
        paths = {name, "/usr/share/opencv4/haarcascades/" + name,
                 "/usr/share/opencv/haarcascades/" + name,
                 "/usr/local/share/opencv4/haarcascades/" + name};
    }
    try {
        for (const auto& path : paths)
            if (access(path.c_str(), R_OK) == 0 && detector.load(path)) break;
    } catch (const cv::Exception& e) {
        std::cerr << "Cannot load face model: " << e.what() << '\n'; return 1;
    }
    if (detector.empty()) {
        std::cerr << "Missing face model: supply haarcascade_frontalface_default.xml as argument 3.\n";
        return 1;
    }
    int fd = openSerial(serial);
    if (fd < 0) { std::cerr << "Cannot open " << serial << ": " << std::strerror(errno) << '\n'; return 2; }
    const cv::Size baseWindow = detector.getOriginalWindowSize();
    std::cout << "Face tracker v13: serial=" << serial << " 115200 8N1, camera=" << device
              << ", image=640x480, image_center=(320,240), detect_width<=240, max_age_ms="
              << MAX_RESULT_AGE_MS << ", capture_fps_request=" << captureFps << ", adaptive_ROI=on"
              << ", serial_x=639-raw_x, serial_y=479-raw_y" << std::endl;

    LatestFrame latest;
    std::unique_ptr<MjpegPreview> preview;
    // 摄像头线程持续采集，只保留一张最新图像；检测慢时直接覆盖旧图像。
    std::thread capture;
    uint8_t sequence = 0;
    int width = FRAME_WIDTH, height = FRAME_HEIGHT;
    int result = 0;
    auto transmit = [&](bool face, int x = 0, int y = 0) {
        return sendPacket(fd, packet(sequence++, face, x, y, width, height));
    };
    try {
        if (previewEnabled) preview.reset(new MjpegPreview(latest, previewPort, previewFps, mirrorPreview));
        capture = std::thread([&] {
            try {
                cv::VideoCapture camera(device, cv::CAP_V4L2);
                if (!camera.isOpened()) { latest.failed = true; return; }
                // 原始采集/串口/调试图片统一640x480，只有内部检测图缩小。
                camera.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
                camera.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
                // 摄像头仍为640x480；请求的FPS不代表检测速度，以日志中的实测值为准。
                camera.set(cv::CAP_PROP_FPS, captureFps);
                // V4L2后端在retrieve颜色转换完成后才归还采集缓冲。
                // 至少两个缓冲，让设备在转换上一帧时仍能接收下一帧；单缓冲可能丢帧。
                // 这里是驱动缓冲数量，应用层仍持续读取且只保留一张最新图像。
                if (!camera.set(cv::CAP_PROP_BUFFERSIZE, CAPTURE_BUFFERS))
                    std::cerr << "Camera rejected capture buffer request: " << CAPTURE_BUFFERS << '\n';
                const uint32_t fourcc = static_cast<uint32_t>(camera.get(cv::CAP_PROP_FOURCC));
                std::string format(4, '?');
                for (int i = 0; i < 4; ++i) {
                    const unsigned char ch = static_cast<unsigned char>(fourcc >> (i * 8));
                    if (ch >= 32 && ch <= 126) format[i] = static_cast<char>(ch);
                }
                std::cout << "Camera configured: " << camera.get(cv::CAP_PROP_FRAME_WIDTH) << 'x'
                          << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << " reported_fps="
                          << camera.get(cv::CAP_PROP_FPS) << " format=" << format
                          << " reported_buffers=" << camera.get(cv::CAP_PROP_BUFFERSIZE) << std::endl;
                reportCameraControls(device, fixedFps);
                auto captureRateStart = Clock::now();
                unsigned captureCount = 0;
                double actualCaptureFps = 0.0;
                double grabSum = 0.0, decodeSum = 0.0, decodeCpuSum = 0.0;
                double averageGrab = 0.0, averageDecode = 0.0, averageDecodeCpu = 0.0;
                while (!latest.stopping && !quit) {
                    cv::Mat image;
                    // read等价于grab+retrieve；拆开后区分设备/驱动等待和解码/颜色转换。
                    const auto grabStart = Clock::now();
                    if (!camera.grab()) { latest.failed = true; return; }
                    const auto grabbed = Clock::now();
                    const int64_t decodeCpuStart = threadCpuNs();
                    if (!camera.retrieve(image) || image.empty()) { latest.failed = true; return; }
                    const auto acquired = Clock::now();
                    grabSum += std::chrono::duration<double, std::milli>(grabbed - grabStart).count();
                    decodeSum += std::chrono::duration<double, std::milli>(acquired - grabbed).count();
                    decodeCpuSum += std::max<int64_t>(0, threadCpuNs() - decodeCpuStart) / 1000000.0;
                    if (image.cols != FRAME_WIDTH || image.rows != FRAME_HEIGHT) {
                        std::cerr << "Camera returned " << image.cols << 'x' << image.rows
                                  << "; 640x480 is required. Check the camera video mode.\n";
                        latest.failed = true;
                        return;
                    }
                    ++captureCount;
                    const double elapsed = std::chrono::duration<double>(acquired - captureRateStart).count();
                    if (elapsed >= 1.0) {
                        actualCaptureFps = captureCount / elapsed;
                        averageGrab = grabSum / captureCount;
                        averageDecode = decodeSum / captureCount;
                        averageDecodeCpu = decodeCpuSum / captureCount;
                        grabSum = decodeSum = decodeCpuSum = 0.0;
                        captureCount = 0;
                        captureRateStart = acquired;
                    }
                    std::lock_guard<std::mutex> guard(latest.mutex);
                    latest.image = image;
                    latest.acquired = acquired;
                    ++latest.generation;
                    latest.captureFps = actualCaptureFps;
                    latest.grabMs = averageGrab;
                    latest.decodeMs = averageDecode;
                    latest.decodeCpuMs = averageDecodeCpu;
                }
            } catch (const std::exception& e) {
                std::cerr << "Camera error: " << e.what() << '\n'; latest.failed = true;
            }
        });
        uint64_t used = 0;
        TargetAssociation target;
        bool lastWasFace = false;
        auto lastSend = Clock::now() - std::chrono::seconds(1);
        auto lastLog = Clock::now();
        auto lastSnapshot = Clock::now();
        unsigned processed = 0, accepted = 0, expired = 0;
        FrameRates rates;
        cv::Mat small, gray;
        std::vector<cv::Rect> faces;
        while (!quit) {
            cv::Mat image;
            double actualCaptureFps = 0.0;
            double grabMs = 0.0, decodeMs = 0.0, decodeCpuMs = 0.0;
            Clock::time_point acquired;
            {
                std::lock_guard<std::mutex> guard(latest.mutex);
                actualCaptureFps = latest.captureFps;
                grabMs = latest.grabMs;
                decodeMs = latest.decodeMs;
                decodeCpuMs = latest.decodeCpuMs;
                if (latest.generation != used) {
                    used = latest.generation; image = latest.image; acquired = latest.acquired;
                }
            }
            auto now = Clock::now();
            rates.update(now);
            if (latest.failed) { std::cerr << "Camera disconnected or capture failed.\n"; result = 3; break; }
            if (image.empty() || now - acquired > std::chrono::milliseconds(MAX_RESULT_AGE_MS)) {
                // 正常等下一张图时不立即发无人脸，否则会在相邻有效帧间插入HOLD。
                if ((lastWasFace && !image.empty()) || now - lastSend >= std::chrono::milliseconds(100)) {
                    if (!transmit(false)) { result = 4; break; }
                    lastSend = now;
                    lastWasFace = false;
                }
                if (now - lastLog >= std::chrono::seconds(1)) {
                    // 正常等下一张图不等于已发送HOLD；按最近一次串口状态显示。
                    std::cout << (lastWasFace ? "FACE" : "HOLD")
                              << " reason=" << (image.empty() ? "WAIT_FRAME" : "CAPTURE_STALE")
                              << " capture_fps=" << fpsText(actualCaptureFps)
                              << " grab_ms=" << fpsText(grabMs) << " decode_ms=" << fpsText(decodeMs)
                              << " decode_cpu_ms=" << fpsText(decodeCpuMs)
                              << " detect_fps=" << fpsText(rates.detectFps) << " valid_tx_fps=" << fpsText(rates.validTxFps)
                              << " processed=" << processed << " accepted=" << accepted
                              << " expired=" << expired << std::endl;
                    lastLog = now;
                    processed = accepted = expired = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            if (image.cols < 16 || image.cols > 4096 || image.rows < 16 || image.rows > 4096)
                throw std::runtime_error("Camera dimensions outside protocol range 16..4096");
            if (width != image.cols || height != image.rows) target = TargetAssociation{};
            width = image.cols; height = image.rows;
            const auto detectStart = Clock::now();
            const int64_t detectCpuStart = threadCpuNs();
            target.expire(detectStart);
            const DetectionPlan plan = planDetection(image.size(), target.previous, target.locked, target.misses, baseWindow);
            // 只处理本次扫描区域，并复用输出内存；结果仍换算到640x480原图。
            cv::resize(image(plan.region), small, plan.resized, 0, 0, cv::INTER_AREA);
            cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(gray, gray);
            faces.clear();
            detector.detectMultiScale(gray, faces, plan.scaleFactor, 3, 0, plan.minFace, plan.maxFace);
            for (auto& box : faces) box = mapDetection(box, plan, image.size());
            faces.erase(std::remove_if(faces.begin(), faces.end(), [](const cv::Rect& r) { return r.area() <= 0; }), faces.end());
            now = Clock::now();
            const int64_t detectCpuMs = std::max<int64_t>(0, threadCpuNs() - detectCpuStart) / 1000000;
            // 150ms预算包含取图后的等待和检测；超过预算不发送有效坐标。
            // 此时间不是传感器曝光时间，真实端到端延迟仍需在板上测量。
            const bool stale = now - acquired > std::chrono::milliseconds(MAX_RESULT_AGE_MS);
            int index = target.accept(faces, acquired, now, plan.local);
            bool found = index >= 0;
            ++processed;
            if (stale) ++expired;
            if (found) ++accepted;
            const char* reason = found ? "OK" : faces.empty() ? "NO_FACE" : stale ? "STALE_RESULT" : "TARGET_MISMATCH";
            int x = 0, y = 0, rawX = 0, rawY = 0;
            if (found) {
                const cv::Point raw = faceCoordinate(faces[index], width, height);
                const cv::Point tx = serialPoint(raw, width, height);
                rawX = raw.x;
                rawY = raw.y;
                x = tx.x;
                y = tx.y;
            }
            // 有脸每张新图发送；无脸立即通知，随后最多10Hz，不重发旧坐标。
            if (found || lastWasFace || now - lastSend >= std::chrono::milliseconds(100)) {
                if (!transmit(found, x, y)) { result = 4; break; }
                lastSend = now;
                lastWasFace = found;
            }
            const int64_t resultAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - acquired).count();
            rates.record(found);
            rates.update(Clock::now());
            if (previewEnabled) {
                std::lock_guard<std::mutex> guard(latest.mutex);
                latest.detection = DetectionOverlay{image, faces, index, reason, acquired, used, resultAgeMs, rates.detectFps, rates.validTxFps, plan.local, actualCaptureFps};
            }
            if (now - lastLog >= std::chrono::seconds(1)) {
                std::cout << (found ? "FACE" : "HOLD") << " x=" << x << " y=" << y
                          << " raw_x=" << rawX << " raw_y=" << rawY
                          << " image=" << width << 'x' << height << " faces=" << faces.size()
                          << " mode=" << (plan.local ? "ROI" : "GLOBAL") << " scan=" << plan.resized.width << 'x' << plan.resized.height
                          << " reason=" << reason << " stale=" << stale << " age_ms="
                          << std::chrono::duration_cast<std::chrono::milliseconds>(now - acquired).count()
                          << " detect_ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(now - detectStart).count()
                          << " detect_cpu_ms=" << detectCpuMs
                          << " capture_fps=" << fpsText(actualCaptureFps)
                          << " grab_ms=" << fpsText(grabMs) << " decode_ms=" << fpsText(decodeMs)
                          << " decode_cpu_ms=" << fpsText(decodeCpuMs)
                          << " detect_fps=" << fpsText(rates.detectFps) << " valid_tx_fps=" << fpsText(rates.validTxFps)
                          << " processed=" << processed << " accepted=" << accepted << " expired=" << expired << std::endl;
                lastLog = now;
                processed = accepted = expired = 0;
            }
            // 只有指定第4个参数时才保存画面，约2秒更新一次；写盘在串口发送之后。
            // 调试图片保持原始方向；浏览器镜像选项只影响实时预览。
            if (!debugImage.empty() && now - lastSnapshot >= std::chrono::seconds(2)) {
                try {
                    const cv::Mat preview = drawPreview(DetectionOverlay{image, faces, index, reason, acquired, used, resultAgeMs, rates.detectFps, rates.validTxFps, plan.local, actualCaptureFps}, false, 0, 0);
                    if (!cv::imwrite(debugImage, preview)) throw std::runtime_error("imwrite returned false");
                    std::cout << "Debug image: " << debugImage << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Cannot save debug image: " << e.what() << '\n';
                    debugImage.clear();
                }
                lastSnapshot = Clock::now();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Tracking error: " << e.what() << '\n'; result = 5;
    }
    // 串口失败时不再接续可能残缺的帧；其余退出路径通知下位机保持。
    if (result != 4 && !transmit(false)) result = 4;
    latest.stopping = true;
    close(fd);
    preview.reset();
    // 某些V4L2驱动断线会阻塞read；下位机已收到无人脸或将由200ms超时保持。
    if (capture.joinable()) capture.join();
    if (result == 4) std::cerr << "Serial write failed/timed out.\n";
    return result;
}
