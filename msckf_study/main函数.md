##1、读取数据文件和配置文件路径，结果为：
>data_dir：数据文件路径
>config_file：配置文件路径
    std::string data_dir;
    if (argc == 1) {
        data_dir = std::string("C:/Users/XuXin/Desktop/MH_04_difficult/mav0");
    }
    else {
        data_dir = std::string(argv[1]);
    }

    std::string config_file;
    if (argc < 3) {
        config_file = std::string(
            PROJECT_DIR
            "/msckf_vio/entry/config/"
            "camchain-imucam-euroc.yaml");
    }
    else {
        config_file = std::string(argv[2]);
    }
####2、定义一个Stereo_camera_config_t结构体变量stereo_param
>
    typedef struct {
    std::string cam0_distortion_model;//畸变模型
    int32_t cam0_resolution[2];//分辨率
    double cam0_intrinsics[4];//内部物理学
    double cam0_distortion_coeffs[4];//畸变系数
    double R_cam0_imu[9];//cam0旋转矩阵
    double t_cam0_imu[3];//cam0平移向量

    std::string cam1_distortion_model;
    int32_t cam1_resolution[2];
    double cam1_intrinsics[4];
    double cam1_distortion_coeffs[4];
    double R_cam1_imu[9];
    double t_cam1_imu[3];

    double R_imu_body[9];
    double t_imu_body[3];
    } Stereo_camera_config_t;//立体图像配置
####3、把config_file路径下的的yaml文件中参数读取到上述Stereo_camera_config_t结构体中
    loadCaliParameters(config_file, stereo_param);
loadCaliParameters解析如下：
>参数：为yaml配置文件路径和Stereo_camera_config_t结构体变量。
>作用：将yaml配置文件中一些参数赋值给Stereo_camera_config_t结构体。

①使用opencv中FileStorage类读取配置文件。cv::FileStorage对象代表一个YAML或XML格式的文件。
	//1.读取文件指针
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);//从yaml(calib_file)文件中读取数据

	//2.判断文件是否打开成功
    if (!fs.isOpened()) {
        std::cout
            << "can't open calibration file: "
            << calib_file << std::endl;
        return false;
    }
ymal配置文件如下所示

![ymal.png][1]

一旦文件被打开，接下来便可对其中的数据进行读取，首先需要确定你要访问的数据名，即FileStorage最顶层的mapping中的关键字，可以通过重载操作符cv::FileStorage::operator[]获取自己需要的数据。然而，该操作符的返回值并不是你想要的数据，而是一个FileNode对象。下面创建一个FileNode对象接收cam0下的数据
    cv::FileNode node = fs["cam0"];//把cam0数据赋值给node

下面将cam0下的"distortion_model"、"resolution"、"intrinsics"、"distortion_coeffs"、"T_cam_imu"赋值给相应vector数组
    if (node.empty()) {
        stereo_param.cam0_distortion_model = std::string("radtan");
    }
    else {
        node["distortion_model"] >> stereo_param.cam0_distortion_model;
    }

    std::vector<int32_t> cam_resolution_temp(2);
    node["resolution"] >> cam_resolution_temp;


    memcpy(stereo_param.cam0_resolution,
        &cam_resolution_temp[0], sizeof(double)* 2);
    std::vector<double> cam_intrinsics_temp(4);
    node["intrinsics"] >> cam_intrinsics_temp;
    memcpy(stereo_param.cam0_intrinsics,
        &cam_intrinsics_temp[0], sizeof(double)* 4);

    std::vector<double> cam_distortion_coeffs_temp(4);;
    node["distortion_coeffs"] >> cam_distortion_coeffs_temp;
    memcpy(stereo_param.cam0_distortion_coeffs,
        &cam_distortion_coeffs_temp[0], sizeof(double)* 4);

    std::vector<double> T_imu_cam0_temp;
    node["T_cam_imu"] >> T_imu_cam0_temp;
其中memcpy函数为内存拷贝函数
>函数原型
>* void *memcpy(void *destin, void *source, unsigned n);

>参数
>* destin-- 指向用于存储复制内容的目标数组，类型强制转换为 void* 指针。
>* source-- 指向要复制的数据源，类型强制转换为 void* 指针。
>* n-- 要被复制的字节数。

>返回值
>* 该函数返回一个指向目标存储区destin的指针。

>功能
>* 从源source所指的内存地址的起始位置开始拷贝n个字节到目标destin所指的内存地址的起始位置中。

>所需头文件

>* C++：#include<cstring>

下面是矩阵赋值，从齐次坐标系矩阵中，分解出旋转矩阵和平移向量，然后旋转矩阵转置，化成imu系到cam系的旋转矩阵，平移向量转换，不单单是加负号的问题，先用旋转矩阵转换坐标系后再加负号。
```
    cv::Mat T_imu_cam0(4, 4, CV_64F, &T_imu_cam0_temp[0]);
    cv::Mat R_imu_cam0 = T_imu_cam0(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t_imu_cam0 = T_imu_cam0(cv::Rect(3, 0, 1, 3)).clone();
    cv::Mat R_cam0_imu(R_imu_cam0.t());
    cv::Mat t_cam0_imu(-R_cam0_imu * t_imu_cam0);
    memcpy(stereo_param.R_cam0_imu, R_cam0_imu.data, sizeof(double)* 9);
    memcpy(stereo_param.t_cam0_imu, t_cam0_imu.data, sizeof(double)* 3);
```
关于cv::Rect(0, 0, 3, 3)函数
```
参数1和参数2：左上角点坐标；参数3和参数4：宽和高
```
下面就是对cam1相机的参数赋值，跟cam0一样
```
// Camera1 calibration parameters
    node = fs["cam1"];
    if (node.empty()) {
        stereo_param.cam1_distortion_model = std::string("radtan");
    }
    else {
        node["distortion_model"] >> stereo_param.cam1_distortion_model;
    }

    node["resolution"] >> cam_resolution_temp;
    memcpy(stereo_param.cam1_resolution,
        &cam_resolution_temp[0], sizeof(double)* 2);

    node["intrinsics"] >> cam_intrinsics_temp;
    memcpy(stereo_param.cam1_intrinsics,
        &cam_intrinsics_temp[0], sizeof(double)* 4);

    node["distortion_coeffs"] >> cam_distortion_coeffs_temp;
    memcpy(stereo_param.cam1_distortion_coeffs,
        &cam_distortion_coeffs_temp[0], sizeof(double)* 4);

    std::vector<double> T_imu_cam1_temp;
    node["T_cam_imu"] >> T_imu_cam1_temp;
    cv::Mat T_imu_cam1(4, 4, CV_64F, &T_imu_cam1_temp[0]);
    cv::Mat R_imu_cam1 = T_imu_cam1(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t_imu_cam1 = T_imu_cam1(cv::Rect(3, 0, 1, 3)).clone();
    cv::Mat R_cam1_imu(R_imu_cam1.t());
    cv::Mat t_cam1_imu(-R_cam1_imu * t_imu_cam1);
    memcpy(stereo_param.R_cam1_imu, R_cam1_imu.data, sizeof(double)* 9);
    memcpy(stereo_param.t_cam1_imu, t_cam1_imu.data, sizeof(double)* 3);
```
下面是T_imu_body的获取,如果是空的，R矩阵赋值为三阶单位阵，T向量为0三维零向量。
```
    if (node.empty()) {
        R_imu_body = cv::Mat::eye(3, 3, CV_64F);
        t_imu_body = cv::Mat::zeros(3, 1, CV_64F);
    }
    else {
        std::vector<double> T_imu_body_temp;
        node >> T_imu_body_temp;
        cv::Mat T_imu_body(4, 4, CV_64F, &T_imu_body_temp[0]);
        R_imu_body = T_imu_body(cv::Rect(0, 0, 3, 3)).clone();
        t_imu_body = T_imu_body(cv::Rect(3, 0, 1, 3)).clone();
    }
    memcpy(stereo_param.R_imu_body, R_imu_body.data, sizeof(double)* 9);
    memcpy(stereo_param.t_imu_body, t_imu_body.data, sizeof(double)* 3);
```
loadCaliParameters函数结束，已将config_file配置文件赋值给Stereo_camera_config_t结构体
####4、读取imu数据
创建Sensor_imu_t结构体的vector数组，使用read_imu_data函数读取。
①Sensor_imu_t结构体如下：
```
typedef struct {
    uint64_t stamp;//时间戳
    Vector3f_t angular_velocity;//三维角速度向量
    Vector3f_t linear_acceleration;//三维加速度向量
} Sensor_imu_t;
```
②read_imu_data解析如下：
参数为文件路径和Sensor_imu_t结构体数组。
先判断文件是否打开成功
```
    std::ifstream filestream(file);
    if (!filestream.is_open()) {
        std::cout
            << "can't imu data: "
            << file << std::endl;
        return false;
    }
```
开始按行读取(跳过第一行)文件赋值给Sensor_imu_t结构体，每一行就是一个结构体，整个文件组成Sensor_imu_t结构体数组。
```
    std::string line;
    Sensor_imu_t imu;
    /* skip first line */
    getline(filestream, line);
    double vel[3], acc[3];
    while (getline(filestream, line)) {
        sscanf(line.c_str(), "%lld,%lf,%lf,%lf,%lf,%lf,%lf\n",
            &imu.stamp, &vel[0], &vel[1], &vel[2],
            &acc[0], &acc[1], &acc[2]);
        imu.angular_velocity.x = (float)vel[0];
        imu.angular_velocity.y = (float)vel[1];
        imu.angular_velocity.z = (float)vel[2];
        imu.linear_acceleration.x = (float)acc[0];
        imu.linear_acceleration.y = (float)acc[1];
        imu.linear_acceleration.z = (float)acc[2];
        imu_data.push_back(imu);
    }
    filestream.close();
```
其中sscanf函数从字符串读取格式化输入。
>声明
>* int sscanf(const char *str, const char *format, ...)

>参数
>* str -- 这是 C 字符串，是函数检索数据的源。
>* format -- 这是 C 字符串，包含了以下各项中的一个或多个：空格字符、非空格字符 和 format 说明符。format 说明符形式为 [=%[*][width][modifiers]type=]
>* 附加参数 -- 这个函数接受一系列的指针作为附加参数，每一个指针都指向一个对象，对象类型由 format 字符串中相应的 % 标签指定，参数与 % 标签的顺序相同。针对检索数据的 format 字符串中的每个 format 说明符，应指定一个附加参数。如果您想要把 sscanf 操作的结果存储在一个普通的变量中，您应该在标识符前放置引用运算符（&）

>返回值
>* 如果成功，该函数返回成功匹配和赋值的个数。如果到达文件末尾或发生读错误，则返回 EOF。
####5、读取图像数据
创建一个时间戳数组和一个字符串数组变量，使用 read_image_list函数读取时间戳和图像名称。
```
    std::vector<std::string> image_list;
    std::vector<uint64_t> image_stamp;
    read_image_list(
        data_dir + "/cam0/data.csv",
        image_stamp,
        image_list);
```
其中read_image_list函数解析如下：
>声明：
>* bool read_image_list(
std::string file,
std::vector<uint64_t>& timestamp,
std::vector<std::string>& image_list)

>参数:
>* std::string file————文件路径
>* std::vector<uint64_t>& timestamp————时间戳数组
>* vector<std::string>& image_list————字符串数组存储图像名

判断是否打开文件
```
    std::ifstream filestream(file);
    if (!filestream.is_open()) {
        return false;
    }
```

先定义四个变量uint64_t stamp（存储时间戳）、std::string filename（存储文件名）、std::string line（存储一行的内容）、char temp（存储分隔符“，”）。
```
    uint64_t stamp;
    std::string filename;
    std::string line;
    char temp;
```

进入while循环，按行读取文件。
>通过识别#号跳过第一行
```
    if(line.find("#") != std::string::npos) {
        continue;
    }
```
>getline函数读取的csv文件两列之间用逗号隔开，所以使用stringstream类型将一行变量line分割开。
```
    std::stringstream ss(line);
    ss >> stamp >> temp >> filename;
```
>图像名称的命名方式就是时间戳+.png,如果时间戳和图像名称对应不上就会报异常
```
    if(0 != filename.compare(std::to_string(stamp) + ".png")) {
        std::cout << "Read image error in : "<< line << std::endl;
        continue;
    }
```
最后给timestamp、image_list赋值，循环结束。
####6、创建三个线程
```
    MsckfSystem *pSystem = new MsckfSystem(stereo_param);
    pSystem->reset();

    std::thread *pThreadFeature =
        new std::thread(&MsckfSystem::imageHandleLoop, pSystem);

    std::thread *pThreadFusion =
        new std::thread(&MsckfSystem::fusionHandleLoop, pSystem);

    std::thread *pThreadViewer =
        new std::thread(&MsckfSystem::showResult, pSystem);
```
  [1]: https://xin.zyong.vip/usr/uploads/2022/02/1601087183.png
####7、for循环读取每一帧图像并解算
在循环之前定义两个相机矩阵cam0和cam1，Sensor_imu_t结构体组。
```
 cv::Mat cam0, cam1;
    std::vector<Sensor_imu_t> imu_interval;
```

下面进入for循环
通过while循环控制，指针不忙碌继续往下走。
```
    while (pSystem->isImageLoopBusy()
            ||pSystem->isFusionLoopBusy());
```
通过两个 read_stereo_image()函数读取双目相机的图片存到矩阵中。在read_stereo_image()中使用opencv的imread()函数读取。
```
    read_stereo_image(
            cam0,
            data_dir + "/cam0/data/" + image_list[i]);//读取cam0图像第一张存放在cam0矩阵

    read_stereo_image(
            cam1,
            data_dir + "/cam1/data/" + image_list[i]);//读取cam1图像第一张存放在cam1矩阵
```
使用 get_imus()函数获取两帧图像之间的imu数据，下面解析此函数。
>声明：
>* bool get_imus(
  std::vector<Sensor_imu_t>& imu,
  uint64_t timestamp0,
  uint64_t timestamp1,
  std::vector<Sensor_imu_t>& imu_interval)

>参数:
>* std::vector<Sensor_imu_t>& imu————imu结构体数组，存储全部的imu数据。
>* uint64_t timestamp0————t0时刻的时间戳
>* uint64_t timestamp1————t1时刻的时间戳
>* std::vector<Sensor_imu_t>& imu_interval————imu结构体数组，存储t0和t1时刻之间的imu数据

第一个for循环
第二个for循环
此解析暂时未看，待补==========================================

get_imus()函数结束之后，添加imu数据和相机数据
```
pSystem->insertImu(imu_interval);//添加imu数据

        pSystem->insertStereoCam(
            cam0.data,
            cam1.data,
            stereo_param.cam1_resolution[0],
            stereo_param.cam1_resolution[1],
            image_stamp[i]);//添加相机数据
```
这里的cam0.data是uchar类型的指针，指向Mat数据矩阵的首地址。
下面进入pSystem->insertStereoCam()函数，开始前端的内容。
