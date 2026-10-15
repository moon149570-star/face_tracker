# 久久派 2K0300 + STM32 人脸追踪云台

使用久久派 2K0300 采集 USB 摄像头画面，通过 OpenCV 检测并关联人脸，将人脸框中心坐标经串口发送给 STM32，由 STM32 控制双轴舵机跟随目标。

当前上位机版本：**Face tracker v13**。实测硬件：久久派 2K0300、OV9726 USB 摄像头、STM32F103。关闭预览、稳定锁定人脸时，实测采集、检测和有效坐标发送均约30次/秒。

**当前仓库包含Linux上位机、人脸模型，以及 `servo/` 中的STM32下位机工程。** 本文集中说明两端环境准备、路径修改、部署、协议和排查方法。人脸检测用于定位，不识别人员身份。

## 1. 系统分工与项目结构

```text
USB摄像头 → 久久派采集/检测/目标关联 → TTL串口 → STM32双轴控制 → 舵机云台
                    ↓
              局域网HTTP预览
```

上位机只发送新鲜的人脸坐标，不发送舵机角度。下位机负责坐标滤波、PID、速度与加速度限制、角度限位，以及无人脸或通信超时后的保持。

```text
face_tracker/
├── src/main.cpp                         # 上位机全部业务代码
├── servo/                               # STM32下位机，Windows/Keil构建
│   ├── Core/                            # 初始化与主循环
│   ├── BSP/                             # 协议、跟踪、PID、舵机、OLED、按键
│   ├── Drivers/                         # HAL和CMSIS，构建必需
│   ├── MDK-ARM/servo.uvprojx             # Keil工程入口
│   ├── servo.ioc                        # CubeMX配置
│   └── .gitignore                       # 下位机编译产物和本地状态忽略规则
├── haarcascade_frontalface_default.xml   # 正面人脸检测模型
├── CMakeLists.txt                       # C++17、OpenCV和pthread链接配置
├── toolchain-loongarch64.cmake           # 交叉编译器/sysroot/库搜索路径
├── CMakePresets.json                    # 原开发环境的Debug配置
├── .gitignore                           # 排除编译产物和旧代码备份
└── README.md                            # 本项目的统一使用说明
```

`build/camera_probe` 是本地编译产物。`build/`、`out/` 和旧代码备份不会随源码提交到GitHub。交叉编译工具链及OpenCV库也不随本仓库提供，需要自行准备，或者使用维护者另行提供的兼容可执行文件。

## 2. 使用前先选择部署方式

| 方式 | 适用情况 | 需要准备 |
| --- | --- | --- |
| A：直接部署可执行文件 | 已有针对本板卡系统编译的 `camera_probe`，暂时不修改代码 | 可执行文件、XML模型、板上匹配的运行库 |
| B：自行交叉编译 | 修改了源码，或现有可执行文件不兼容板上系统 | 开发电脑上的完整交叉编译环境，以及板上匹配的运行库 |

两种方式最终都在久久派上运行程序。**方式A不需要在开发电脑安装交叉编译器；方式B必须先配好环境，再执行CMake命令。** 仅下载源码无法直接运行，也不能把普通电脑编译出的x86可执行文件放到久久派运行。

### 路径与设备名分别属于哪台机器

| 项目 | 所在机器 | 如何确定 |
| --- | --- | --- |
| 项目根目录 | 开发电脑 | 自己下载/克隆项目的位置，不要求与作者相同 |
| `TOOLCHAIN_ROOT` | 开发电脑 | 自己安装的LoongArch64交叉工具链根目录 |
| `CMAKE_SYSROOT` | 开发电脑 | 与板上系统兼容的目标头文件、库和链接环境所在目录 |
| OpenCV安装目录 / `OpenCV_DIR` | 开发电脑 | 为目标架构编译的OpenCV目录；后者指包含 `OpenCVConfig.cmake` 的目录 |
| 程序和XML路径 | 久久派 | 自己复制文件后的存放位置 |
| `/dev/ttyS1` | 久久派 | 本次使用的串口节点，其他接线或系统需自行核对 |
| `/dev/video0` | 久久派 | 本次使用的USB摄像头节点，多摄像头或重插后可能不同 |
| 预览IP与端口 | 久久派/局域网 | 当前板子IP和启动时指定的端口 |

`/opt/...` 是下面构建示例所用的开发环境路径；`/dev/...` 是板上的设备节点。两者都不是要求所有使用者照搬的固定值。

## 3. 方式A：直接复制可执行文件到久久派

### 3.1 准备文件和运行环境

需要两个文件：

```text
camera_probe
haarcascade_frontalface_default.xml
```

本次开发环境已经生成 `build/camera_probe`。从GitHub获取源码的使用者，需要自行编译，或取得维护者单独提供的可执行文件；源码仓库不自动包含被忽略的 `build/` 内容。目前不假定仓库已经发布预编译下载包。

可执行文件必须匹配板子的LoongArch架构、ABI、系统动态加载器及库版本。程序动态链接OpenCV等库，**复制可执行文件不等于把运行环境也复制过去**。若板上尚无匹配的运行库，需要先部署与该产物配套的库。

### 3.2 复制并运行

例如在久久派上创建目录：

```bash
mkdir -p ~/face_tracker
```

通过U盘、SCP或现有文件传输方式，将两个文件放进该目录。目录名可自行调整，只需运行时使用正确路径。

确认第5节中的串口和摄像头节点后，在**久久派终端**运行：

```bash
cd ~/face_tracker
chmod +x camera_probe
./camera_probe /dev/ttyS1 /dev/video0 ./haarcascade_frontalface_default.xml \
  --capture-fps 30 --fixed-fps
```

这里的 `/dev/ttyS1`、`/dev/video0` 必须换成自己的实际节点。`./haarcascade_frontalface_default.xml` 表示当前工作目录中的模型，而非自动表示可执行文件所在目录。模型在其他位置时传入其绝对路径。

本次设备启动输出示例：

```text
Face tracker v13: ...
Camera configured: 640x480 reported_fps=30 format=YUYV reported_buffers=2
```

其他摄像头的协商格式或控制支持情况可能不同。按 `Ctrl+C` 退出。正常退出会尝试通知下位机无人脸；串口写入失败时停止发送，下位机依靠通信超时保持。

## 4. 方式B：配置环境并交叉编译

以下步骤在**Linux开发电脑**执行。Windows使用者可在合适的Linux开发环境中完成构建，再将产物复制到板子。

### 4.1 先准备交叉编译环境

在构建本项目之前，应已有：

1. CMake及可用的构建工具，如Make。
2. 支持C++17、面向板子系统的LoongArch64交叉编译工具链。
3. 与板上系统兼容的sysroot，包含目标系统头文件和链接库。
4. 使用兼容工具链为LoongArch目标构建的OpenCV，至少包含 `core`、`videoio`、`imgproc`、`imgcodecs`、`objdetect`，且支持V4L2采集和JPEG编解码。
5. 板上能够部署与编译结果匹配的OpenCV、C++等动态运行库。

安装开发电脑原生的x86版OpenCV，不能替代目标架构的OpenCV。建议先按板卡SDK或配套工具链的说明完成环境配置，再编译本项目。

本次已验证的开发环境为GCC 8.3工具链及OpenCV 4.10.0，目录如下，仅供参考：

```text
工具链：/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1
OpenCV：/opt/opencv_4_10_build
```

### 4.2 修改工具链文件中的路径

打开 `toolchain-loongarch64.cmake`，按实际环境检查以下配置：

```cmake
set(TOOLCHAIN_ROOT
    "/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1"
)
set(CMAKE_C_COMPILER
    "${TOOLCHAIN_ROOT}/bin/loongarch64-linux-gnu-gcc"
)
set(CMAKE_CXX_COMPILER
    "${TOOLCHAIN_ROOT}/bin/loongarch64-linux-gnu-g++"
)
set(CMAKE_SYSROOT
    "${TOOLCHAIN_ROOT}/loongarch64-linux-gnu/sysroot"
)
set(CMAKE_FIND_ROOT_PATH
    "${CMAKE_SYSROOT}"
    "/opt/opencv_4_10_build"
)
```

工具链根目录、编译器文件名、sysroot层级和OpenCV目录都需与实际安装一致。不同工具链不一定使用上面的目录结构。不能只改 `TOOLCHAIN_ROOT`，而忽略仍然指向旧目录的OpenCV搜索路径。

`CMAKE_SYSTEM_NAME`、`CMAKE_SYSTEM_PROCESSOR` 等目标配置已经在文件中。工具链配置原理可参考 [CMake官方说明](https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html)。

### 4.3 配置、构建并检查产物

进入自己电脑上的项目根目录，再运行下列命令。`OpenCV_DIR` 应指向**包含 `OpenCVConfig.cmake` 的目录**，不是模型目录或OpenCV头文件目录：

```bash
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=toolchain-loongarch64.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/opt/opencv_4_10_build/lib/cmake/opencv4
cmake --build build -j4
file build/camera_probe
```

将示例中的OpenCV路径替换为自己的路径。构建成功后，产物为 `build/camera_probe`，`file` 应显示LoongArch架构。它在久久派运行，不是直接在普通x86开发电脑运行。

只改 `src/main.cpp` 时，再执行 `cmake --build build -j4` 即可。更换编译器、sysroot或OpenCV环境时，建议换一个新的构建目录重新配置，例如将两条命令中的 `build` 都改为 `build-loongarch`，避免沿用旧缓存。

使用 `Release` 进行性能测试。现有 `CMakePresets.json` 含原开发环境的绝对路径和Debug设置，如需使用也要自行修改；以上手工命令不依赖该preset。

构建完成后按第3节复制可执行文件和模型，并检查板上运行库。

## 5. 根据实际接线选择串口和摄像头

以下检查在**久久派上**执行：

```bash
ls -l /dev/ttyS*
ls -l /dev/video*
```

串口使用USB转接器时，也可能出现 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。节点是否存在、对应哪个物理接口、是否被系统控制台占用，需要结合板卡文档和实际接线确认。不能仅凭接口标注“串口1”推断所有系统都叫 `/dev/ttyS1`。

摄像头也不一定为 `/dev/video0`；多个视频节点可能属于同一个摄像头的不同功能。系统安装了 `v4l2-ctl` 时，可用 `v4l2-ctl --list-devices` 辅助查看设备对应关系。选择可采集640×480画面的节点，当前程序遇到其他分辨率会报错退出。

程序位置参数依次为：

```text
./camera_probe [串口设备] [摄像头设备] [模型路径] [可选调试图片路径] [选项]
```

例如实际串口为 `/dev/ttyS2`、摄像头为 `/dev/video2`，模型放在 `/opt/face_tracker` 时：

```bash
./camera_probe /dev/ttyS2 /dev/video2 /opt/face_tracker/haarcascade_frontalface_default.xml \
  --capture-fps 30 --fixed-fps
```

修改这些运行参数即可，不需要为了更换设备节点修改代码或重新编译。确保当前运行用户对设备节点有访问权限，也不要让其他程序同时占用该串口或摄像头。

## 6. STM32接线、坐标方向与通信协议

### 6.1 本次接线配置

| 久久派端 | STM32F103端 | 说明 |
| --- | --- | --- |
| TXD1，本次对应 `/dev/ttyS1` | PA3 / USART2_RX | 必需，发送坐标 |
| GND | GND | 必需，共地 |
| RXD1 | PA2 / USART2_TX | 可选，当前协议无返回帧 |

串口为 **115200、8N1、无软硬件流控**，使用兼容3.3V逻辑的TTL接口。数据是原始二进制，不能发送ASCII坐标、十六进制文本或额外换行。

当前下位机使用USART2，PA8/PA9分别用于舵机PWM。久久派的“串口1”与STM32的“USART1”不是必须同号的关系；如果自行修改下位机串口，应同时调整接线和固件配置。

舵机使用独立且合适的电源，并与控制系统共地。板卡、摄像头和STM32也需要稳定供电；供电下降时先排查电源和线缆。

### 6.2 坐标方向

原始图像为640×480，原点在左上角，x向右增加、y向下增加，人脸坐标取检测框中心并限制在图像范围内。

**当前程序已按实测云台安装方向，在发送前反转两轴：**

```text
发送x = 639 - raw_x
发送y = 479 - raw_y
```

该转换位于 `serialPoint()`。更换摄像头朝向、舵机安装方式或下位机方向配置后，应重新检查闭环方向；避免上下位机同时重复反转同一轴。

预览默认只做水平镜像，画面、框和质心一起变换。`--no-mirror` 只改变预览方向，不改变串口坐标。日志 `raw_x/raw_y` 为原始坐标，`x/y` 和预览的 `TX=(x,y)` 为实际发送坐标。

图像中心按 `(320,240)` 理解。像素索引反转后该点为 `(319,239)`，是1像素取整差异。无人脸占位坐标固定为 `(0,0)`，不对占位值进行反转。

### 6.3 固定16字节协议v1

| 字节偏移 | 字段 | 内容 |
| --- | --- | --- |
| 0 | head1 | `0xA3` |
| 1 | head2 | `0xB3` |
| 2 | version | `0x01` |
| 3 | sequence | 0～255循环序号 |
| 4 | flags | bit0：1有人脸，0无人脸；其余位为0 |
| 5 | reserved | `0x00` |
| 6～7 | x | uint16，小端 |
| 8～9 | y | uint16，小端 |
| 10～11 | width | uint16，小端，当前640 |
| 12～13 | height | uint16，小端，当前480 |
| 14～15 | crc | uint16，小端 |

无帧尾。CRC为 `CRC-16/CCITT-FALSE`：poly=`0x1021`、init=`0xFFFF`、refin=false、refout=false、xorout=`0x0000`，只覆盖偏移2～13的12字节。标准校验向量 `123456789` 对应 `0x29B1`。

有人脸时必须满足 `0≤x<width`、`0≤y<height`；无人脸时标志为0、x和y均为0，宽高仍有效。旧7字节 `A3 B3 xL xH yL yH C3` 协议与当前版本不兼容。

每次发送新测量或状态时递增序号。有效坐标来自新图像，不通过反复发送旧坐标维持发送频率。无人脸首次立即通知，后续约10Hz；正常等待新图不会立即插入保持帧，约100ms没有新发送时会通知保持。单帧写入预算20ms，处理部分写入，失败后退出。

下位机收到无人脸，或超过200ms没有新的有效测量，应保持当前角度，不自动扫动或回零。协议无ACK，`valid_tx_fps` 表示上位机有效帧成功写入串口，不代表已经确认STM32解析成功。

## 7. 人脸模型

项目根目录已包含 `haarcascade_frontalface_default.xml`，部署时与可执行文件一起复制即可，无需在久久派再次下载。

这是OpenCV的正面人脸Haar级联模型，不是摄像头配置。侧脸、遮挡、暗光、运动模糊或人脸过小可能导致漏检。程序每帧重新检测，并根据位置、大小和框重叠关联目标；多人交叉或遮挡时不能保证同一人员身份。

模型来源：[OpenCV官方模型目录](https://github.com/opencv/opencv/tree/4.x/data/haarcascades)。如需重新取得模型，可在能联网的开发电脑下载后传入板子。XML文件内已有作者、版权和许可声明，分发时保留。

## 8. 局域网浏览器预览

在久久派运行：

```bash
./camera_probe /dev/ttyS1 /dev/video0 ./haarcascade_frontalface_default.xml \
  --capture-fps 30 --fixed-fps --preview --preview-fps 15
```

设备名和模型路径仍按第5节修改。程序保持运行，电脑与久久派需能够通过局域网IP互通。串口终端连接不会自动建立视频网络连接。

在板上执行 `ip addr`，找到当前联网接口的IPv4地址，例如 `192.168.1.100`，再在电脑浏览器打开：

```text
http://192.168.1.100:8080/
```

该地址仅为示例，必须替换成自己的板子IP。使用 **HTTP**，也可访问 `/stream`。端口默认8080，可用 `--preview-port 8081` 等参数更改，并同步修改浏览器地址。

画面包含人脸框、质心、中心十字、`FACE1/NOFACE/HOLD` 状态、实际发送坐标，以及 `VIEW/CAP/DET/VALID_TX` 帧率信息。框和对应检测图像一起输出，避免把旧框画到新图上。

预览只支持一个观看连接，无人观看时不编码。观看时按约15%的线程CPU软预算自动降帧，并降低预览线程优先级。`--preview-fps 30` 只是上限，不保证显示30帧；页面只有几帧时，实际检测可能仍然更高，应查看 `DET` 或日志。

## 9. 参数与实测性能

### 参数速查

| 参数 | 默认与作用 |
| --- | --- |
| 第1位置参数：串口 | `/dev/ttyS1`，按实际接线和设备节点更改 |
| 第2位置参数：摄像头 | `/dev/video0`，按实际可采集节点更改 |
| 第3位置参数：模型 | 建议显式传入；省略时搜索当前工作目录及OpenCV标准数据目录 |
| 第4位置参数：调试图片 | 可选，约每2秒保存一次带标识的原始方向图片；路径需可写 |
| `--capture-fps 1..30` | 默认30，请求采集帧率，以实测日志为准 |
| `--fixed-fps` | 尝试关闭曝光优先变帧率，保留自动曝光模式；设备可能不支持 |
| `--preview` | 开启预览，默认关闭 |
| `--preview-port PORT` | 默认8080，指定后同时开启预览 |
| `--preview-fps 1..30` | 默认15，指定后同时开启预览 |
| `--no-mirror` | 预览显示原始方向，不改变串口坐标 |
| `--help` | 打印用法 |

### 性能记录

记录日期：2026-10-15。条件：久久派2K0300、OV9726 USB模块、Release构建、640×480 YUYV、请求30帧、`--fixed-fps`、关闭预览且不保存调试图片。

| 场景 | 实测结果 |
| --- | --- |
| 稳定锁定，ROI模式 | 采集约30帧、检测约30帧、有效坐标发送约30次/秒 |
| 常见局部检测耗时 | 约17～22ms |
| 丢脸后的全图搜索 | 常见检测约130～165ms、约7～9帧；采集仍约30帧 |
| 重新锁定 | 切回ROI后可恢复约30帧 |

稳定日志示例：

```text
capture_fps=30.0 grab_ms=22.5 decode_ms=10.8 detect_fps=30.0 valid_tx_fps=30.0 processed=30 accepted=30 expired=0
```

v13将V4L2采集缓冲由1改为2，解决了本次采集只有15帧的瓶颈。保持 `CAPTURE_BUFFERS=2` 可让接收与转换重叠，应用层仍只保留最新一张图像。全图搜索缩至240×180，锁定后局部扫描常见约86×86；漏检逐步扩大ROI，连续3次未匹配后回到全图搜索。

结果年龄超过150ms时不发送有效坐标；较旧全图候选只可用于定位下一张新图的ROI，经过新图确认后才能发送。`age_ms` 从retrieve完成后计时，不包含此前传感器曝光和传输，因此不是完整端到端延迟。

以上为指定条件下实测，不承诺快速移动、转头、遮挡、不同光照或开启预览时始终30帧。后续优化重点是减少移动漏检和加快全图重捕获。

## 10. 常见问题与排查

| 现象 | 优先检查 |
| --- | --- |
| `Exec format error` | 可执行文件架构是否为目标LoongArch，是否误用了电脑端产物 |
| 文件存在却提示无法找到/执行 | 执行权限、动态加载器路径、架构和系统兼容性 |
| 提示找不到 `.so`，或 `GLIBC/GLIBCXX` 版本不符 | 在板上用 `ldd ./camera_probe` 检查；部署配套目标库，或使用匹配sysroot重新编译 |
| CMake找不到OpenCV | `OpenCV_DIR` 是否包含 `OpenCVConfig.cmake`，是否安装了目标架构所需模块 |
| 无法打开串口 | 第1参数的设备节点、访问权限、是否被控制台或其他程序占用 |
| 摄像头打开/采集失败 | 第2参数的节点、640×480模式支持、USB连接、权限、占用及供电 |
| 模型加载失败 | 第3参数是否指向真实XML，当前工作目录是否正确，文件是否可读 |
| STM32收不到坐标 | TX→RX、共地、115200 8N1、两端协议是否均为16字节v1 |
| 一直 `HOLD / NO_FACE` | 模型、实际画面、光照、角度、目标大小及模糊；`faces=1` 也需看有效性原因 |
| `STALE_RESULT` | 结果超过150ms，检查GLOBAL耗时、CPU竞争和图像积压 |
| `FACE reason=WAIT_FRAME` | 最近状态为有效人脸，正常等待下一张图，不代表已经发送HOLD |
| 预览超时或502 | 板子IP、HTTP协议、是否开启预览且程序仍运行、端口、设备互通及电脑代理 |
| 页面帧率低 | 区分预览与检测帧率；检查CPU编码预算和网络，不能仅凭页面判断检测速度 |
| 跟随方向相反 | 检查 `serialPoint()`、机械安装和下位机方向参数，不能只改预览镜像 |

日志中 `capture_fps` 为实测采集，`reported_fps` 为驱动报告；`detect_fps` 为所有完成检测次数，`valid_tx_fps` 为有效坐标成功写入次数。`processed/accepted/expired` 是上次打印以来的处理、有效接受和过期计数。启动初期帧率尚未统计完成时可能显示0。

`detect_ms` 为本次检测墙钟耗时，`detect_cpu_ms` 为检测线程CPU耗时；`grab_ms/decode_ms` 为采集等待与转换阶段的平均墙钟耗时。先辨别采集、检测、有效发送、预览哪一环节变慢，再调整对应环节。

## 11. STM32下位机编译、烧录与参数

下位机位于 `servo/`，与上位机 `src/main.cpp` 属于同一个仓库。**Linux中的CMake命令只构建上位机，下位机在Windows中使用Keil单独构建。** 不需要把目录改名为 `stm32/`。

### Windows环境与构建入口

1. 在Windows取得完整仓库，保留 `servo/Core`、`BSP`、`Drivers`、`MDK-ARM` 的相对位置。
2. 配置Keil μVision、ARM Compiler 5.06 update 6，以及工程指定的 `Keil.STM32F1xx_DFP.2.4.1` 设备包。当前工程设备为 `STM32F103C8`，使用C99；ARM Compiler 6不能仅凭已安装就当作当前工程的等价环境。
3. 打开 `servo/MDK-ARM/servo.uvprojx`，选择 `servo` 目标，执行Rebuild。
4. 生成的固件为 `servo/MDK-ARM/servo/servo.hex`，调试产物为同目录的 `servo.axf`。整个生成目录已忽略，不随源码提交。
5. 连接ST-LINK，确认目标供电及调试器配置后执行Download，并检查下载与校验结果。编译成功不等于已经烧录成功。

工程引用的源码与头文件目录采用相对路径，整体克隆到其他盘符或目录即可保留这些引用。Keil、编译器、设备包及调试器驱动仍需在自己的电脑安装；它们不包含在仓库里。

### 当前下位机硬件配置

| 功能 | 引脚/配置 |
| --- | --- |
| 水平舵机 | PA8 / TIM1_CH1，启动命令90° |
| 垂直舵机 | PA9 / TIM1_CH2，启动命令45° |
| 接收上位机数据 | PA3 / USART2_RX，115200 8N1 |
| OLED | PB8 / I2C1_SCL、PB9 / I2C1_SDA，I2C1重映射，100kHz |
| OLED地址 | 驱动地址0x78，对应7位地址0x3C |
| 按键 | PA4、PA6、PB1、PB11，低电平按下 |
| 状态LED | PC13，FACE时低电平，其他状态高电平 |
| 时钟/PWM | 8MHz外部晶振、系统72MHz；TIM1输出50HzPWM |

更换晶振或系统时钟后需检查定时器计数频率，否则脉宽与角度的映射也会改变。角度是软件命令值，实际机械角度需按舵盘安装和脉宽标定确认。

### 参数位置与默认值

控制参数集中在 `servo/BSP/Inc/tracking_config.h`：

| 参数 | 水平轴 | 垂直轴 |
| --- | --- | --- |
| 初始角度 | 90° | 45° |
| 软件角度范围 | 10～170° | 10～135° |
| 方向符号 | +1 | +1 |
| Kp / Ki / Kd | 100 / 3 / 1.5 | 80 / 2 / 1.2 |
| 最大角速度 | 120°/s | 90°/s |
| 最大加速度 | 600°/s² | 450°/s² |
| 0～180°默认脉宽映射 | 500～2500μs | 500～2500μs |

公共默认值：控制目标周期20ms、启动等待1000ms、有效测量超时200ms、坐标滤波时间常数40ms、微分滤波时间常数60ms。中心死区归一化阈值为0.015进入、0.030退出。按默认映射，90°为1500μs，45°为1000μs。

PID输出为角速度，按实际经过时间更新角度，结合加速度限制与软件限位。默认参数是调试起点，不代表适合所有机械安装。先核对两轴方向，再按需要从Ki=Kd=0开始调Kp，随后逐步加入Kd和Ki；不要用提高增益补偿电源或机械问题。

### 状态与维护

启动阶段为HOME，约1秒后按新测量进入FACE；收到无人脸或超过200ms无新有效测量时进入HOLD，保持当前命令角度。恢复有效测量后继续跟踪，复位才重新回到启动姿态。同序号重传不刷新有效测量超时。

串口中断只缓存字节及接收时间，主循环解析CRC和字段、取得最新完整帧并执行控制。OLED显示状态、命令角度和坐标，I2C故障时停止刷新；按键当前只扫描状态，没有在主循环分配手动调角或归位动作。

Keil调试时可观察 `tracker.state`、`tracker.pan/tilt.angle` 及 `g_serial_stats` 中的 `good_frames`、`bad_frames`、`uart_errors`、`ring_overflows`。保留CubeMX的USER CODE区域，重新生成后检查BSP文件仍在工程中，避免在串口中断中加入显示、打印或PID计算。

上下位机均采用第6节的协议v1。更改线格式、坐标方向或失效行为时同步更新本README，并成对验证两个程序。本次仅检查了Keil工程引用文件是否存在，尚未在此Linux环境执行Keil构建或重新烧录。

## 12. 联调验收

1. 正确部署程序、XML及运行库，启动显示v13和640×480，设备节点与实物一致。
2. 关闭预览及调试写盘，正面人脸稳定锁定后检查 `CAP/DET/VALID_TX`。
3. 分别上下左右移动人脸，确认云台动作使目标逐步接近中心。
4. 人脸离开画面或停止有效数据后，STM32保持当前角度。
5. 人脸重新出现后能重捕获，观察GLOBAL恢复为ROI和帧率恢复。
6. 开启预览检查框、质心、状态及TX字段，单独记录开启预览后的性能。

v13已通过当前工具链的Release交叉编译及警告检查，真实帧率来自实物日志。快速运动、长时间运行和机械闭环稳定性仍需在自己的硬件上验收。

## 13. 第三方许可

`haarcascade_frontalface_default.xml` 保留OpenCV模型原始作者、版权和许可声明。项目自有代码的许可证尚未指定，可由维护者另行添加 `LICENSE`；不能用项目许可证替换第三方模型内的声明。

`servo/Drivers/` 内HAL与CMSIS组件的原始许可文件也应随对应源码保留。
