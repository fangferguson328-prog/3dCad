#include "rscamera.h"
#include <QDebug>
#include "xcamera.h"
#include "enumerate.h"
#include <fstream>
#include <iostream>

using namespace CAMERA;

//从KWSC读取点云

void loadPointCloudFromKWSC(QVector<Vertex>& glPoint, bool& isSuccess, GLuint& textureID)
{
    /*****************************************************************************************************/
    int ret_code = 0;

    //创建相机
    XCamera* p_camera = (XCamera*)createXCamera();

    //连接相机 
    
     ret_code = p_camera->connect("192.168.1.100");
    
     if (ret_code == -1) {
         isSuccess = false;
         return;
    }

    int width = 0, height = 0;
    int channels = 1;

    if (0 == ret_code)
    {
        //必须连接相机成功后，才可获取相机分辨率
        ret_code = p_camera->getCameraResolution(&width, &height);
        std::cout << "Width: " << width << "    Height: " << height << std::endl;

        ret_code = p_camera->getCameraChannels(&channels);
        std::cout << "channels: " << channels << std::endl;
    }
    else
    {
        std::cout << "Connect Camera Error!";
        isSuccess = false;
        return;
    }

    //分配内存保存采集结果
    float* point_cloud_data = (float*)malloc(sizeof(float) * width * height * 3);
    memset(point_cloud_data, 0, sizeof(float) * width * height * 3);

    float* height_map_data = (float*)malloc(sizeof(float) * width * height);
    memset(height_map_data, 0, sizeof(float) * width * height);

    float* depth_data = (float*)malloc(sizeof(float) * width * height);
    memset(depth_data, 0, sizeof(float) * width * height);

    char* timestamp_data = (char*)malloc(sizeof(char) * 30);
    memset(timestamp_data, 0, sizeof(char) * 30);

    unsigned char* brightness_data = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
    memset(brightness_data, 0, sizeof(unsigned char) * width * height);

    unsigned char* color_brightness_data = (unsigned char*)malloc(sizeof(unsigned char) * width * height * 3);
    memset(color_brightness_data, 0, sizeof(unsigned char) * width * height * 3);

    int capture_num = 0;

    if (0 == ret_code)
    {
        //set patam json
        char status_json[20480];
        char config_json[20480];
        int num = 0;
        p_camera->readJson(config_json, "0414_config.json");
        p_camera->setParamJson(config_json, status_json, num);

        ret_code = p_camera->captureData(num, timestamp_data);

        if (0 == ret_code)
        {
            glPoint.clear();
            if (1 == channels)
            {
                
                //获取亮度图数据并保存
                ret_code = p_camera->getBrightnessData(brightness_data);
                if (0 == ret_code)
                {
                    cv::Mat bright = cv::Mat(height, width, CV_8U, brightness_data);
                    //cv::imwrite("bright.bmp", bright);
                
                 
                }
				
				
				
                //获取点云数据并保存（2选1）
                ret_code = p_camera->getPointcloudData(point_cloud_data);
                if (0 == ret_code)
                {
                    std::cout << "Get Pointcloud!" << std::endl;
                    
                }
            }
            else if (3 == channels)
            {
                //获取亮度图数据并保存
                ret_code = p_camera->getColorBrightnessData(color_brightness_data, Color::Rgb);
                if (0 == ret_code)
                {
                    cv::Mat bright = cv::Mat(height, width, CV_8UC3, color_brightness_data);
                    //cv::imwrite("bright.bmp", bright);
                    std::cout << "Get color Brightness!" << std::endl;
                }

                //获取点云数据并保存（2选1）
                ret_code = p_camera->getPointcloudData(point_cloud_data);
                if (0 == ret_code)
                {
                    std::cout << "Get Pointcloud!" << std::endl;
                    //const char* pcd = "point_cloud.pcd";
                    //const char* ply = "point_cloud.ply";
                    //p_camera->savePointcloudToPcd(point_cloud_data, color_brightness_data, channels, pcd);
                    //p_camera->savePointcloudToPly(point_cloud_data, color_brightness_data, channels, ply);
                   
                }
            }
            capture_num++;

            std::cout << "Capture num: " << capture_num << std::endl;
            // 获取OpenGL函数
            QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

            // 初始化纹理
            f->glGenTextures(1, &textureID);
            f->glBindTexture(GL_TEXTURE_2D, textureID);

            // 更新纹理数据
            f->glBindTexture(GL_TEXTURE_2D, textureID);
            if(channels == 3)
                f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, color_brightness_data);
            else
				f->glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, brightness_data);

            //设置格式
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
            glPoint.resize(width * height);
			//获取颜色帧的数据
            for (int i = 0; i < width * height; ++i) {
                glPoint[i].position[0] = static_cast<float>(point_cloud_data[i * 3]) / 1000;
                glPoint[i].position[1] = static_cast<float>(point_cloud_data[i * 3 + 1]) / 1000;
                glPoint[i].position[2] = static_cast<float>(point_cloud_data[i * 3 + 2]) / 1000;
                if(channels == 3)
                {
                    glPoint[i].color[0] = static_cast<float>(color_brightness_data[i * 3]) / 255;
                    glPoint[i].color[1] = static_cast<float>(color_brightness_data[i * 3 + 1]) / 255;
                    glPoint[i].color[2] = static_cast<float>(color_brightness_data[i * 3 + 2]) / 255;
                   
                }
                else
                {
					glPoint[i].color[0] = static_cast<float>(brightness_data[i]) / 255;
					glPoint[i].color[1] = static_cast<float>(brightness_data[i]) / 255;
					glPoint[i].color[2] = static_cast<float>(brightness_data[i]) / 255;
                }
                glPoint[i].texCoord[0] = static_cast<float>(i % width) / width;
                glPoint[i].texCoord[1] = static_cast<float>(i / width) / height;
            }
			glPoint.resize(width * height);
			std::cout << "Get Pointcloud!" << std::endl;
			isSuccess = true;
        }
        else
        {
            std::cout << "Capture Data Error!" << std::endl;
			isSuccess = false;
        }

    }


    free(brightness_data);
    free(depth_data);
    free(point_cloud_data);
    free(height_map_data);
    free(timestamp_data);

    p_camera->disconnect("192.168.1.100");

    destroyXCamera(p_camera);

}


// 从Intel RealSense读取点云数据
void loadPointCloudFromRealSense(std::vector<Vertex> &glPoint, bool &isSuccess, GLuint &textureID)
{
    try {
        rs2::context ctx;
        auto list = ctx.query_devices();
        if (list.size() == 0) {
            qDebug() << "无相机设备连接.";
            isSuccess = false;
            return;
        } else {
            rs2::device device = list.front();
            qDebug() << "设备: " << device.get_info(RS2_CAMERA_INFO_NAME);
//            device.hardware_reset();
        }

        rs2::pipeline pipe;
        rs2::config cfg;
        //cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30); // 使用更高的分辨率
        //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30); // 使用更高的分辨率


        qDebug() << "启动流\n";
        rs2::pipeline_profile selection = pipe.start(cfg);
        rs2::sensor selected_sensor = selection.get_device().query_sensors()[1];

        // 禁用自动曝光
        //selected_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

        // 设置曝光时间（单位为微秒，值可以根据需要调整）
        //selected_sensor.set_option(RS2_OPTION_EXPOSURE, 9000);

        qDebug() << "开始获取帧";
        rs2::frameset frames = pipe.wait_for_frames();

//        qDebug() << "已获取到帧";
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        if (!depth || !color) {

//            qDebug() << "未获取到深度帧或颜色帧";
            isSuccess = false;
            pipe.stop();
            return;
        }

        rs2::pointcloud pc;
        pc.map_to(color); // 将点云映射到颜色帧
        auto points_data = pc.calculate(depth);



        auto vertices = points_data.get_vertices();
        auto tex_coords = points_data.get_texture_coordinates();
        // 获取OpenGL函数
        QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

        // 初始化纹理
        f->glGenTextures(1, &textureID);
        f->glBindTexture(GL_TEXTURE_2D, textureID);

        // 更新纹理数据
        f->glBindTexture(GL_TEXTURE_2D, textureID);
        f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, color.get_width(), color.get_height(), 0, GL_RGB, GL_UNSIGNED_BYTE, color.get_data());

        //设置格式
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        glPoint.clear();

        // 获取颜色帧的数据
        const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
        int width = color.get_width();
        int height = color.get_height();

        for (size_t i = 0; i < points_data.size(); ++i) {
            if (vertices[i].z != 0) {  // 过滤掉无效点
                Vertex vertex;
                vertex.position = QVector3D(-vertices[i].x, vertices[i].y,  vertices[i].z);

                float u = tex_coords[i].u;
                float v = tex_coords[i].v;

                // 存储纹理坐标
                vertex.texCoord = QVector2D(u, v);
                // 将纹理坐标转换为像素坐标
                int x = std::min(std::max(int(u * width + 0.5f), 0), width - 1);
                int y = std::min(std::max(int(v * height + 0.5f), 0), height - 1);

                int idx = (y * width + x) * 3; // RGB格式

                // 获取颜色
                uint8_t red = color_data[idx];
                uint8_t green = color_data[idx + 1];
                uint8_t blue = color_data[idx + 2];

                // 存储颜色
                vertex.color = QVector3D(red / 255.0f, green / 255.0f, blue / 255.0f);

                glPoint.emplace_back(vertex);
            }
        }
        isSuccess = true;

        pipe.stop();
        qDebug() << "停止管道";

    } catch (const rs2::error &e) {
        qDebug() << "RealSense error: " << e.what();
        isSuccess = false;
    } catch (const std::exception &e) {
        qDebug() << "Exception: " << e.what();
        isSuccess = false;
    } catch (...) {
        qDebug() << "Unknown error";
        isSuccess = false;
    }
}

void loadPointCloudFromRealSense(QVector<Vertex> &glPoint, bool &isSuccess, GLuint &textureID)
{
    try {
        rs2::context ctx;
        auto list = ctx.query_devices();
        if (list.size() == 0) {
            qDebug() << "无相机设备连接.";
            isSuccess = false;
            return;
        } else {
            rs2::device device = list.front();
            qDebug() << "设备: " << device.get_info(RS2_CAMERA_INFO_NAME);
//            device.hardware_reset();
        }

        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH,848, 480, RS2_FORMAT_Z16, 30); // 使用更高的分辨率
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30); // 使用更高的分辨率


        qDebug() << "启动流\n";
        rs2::pipeline_profile selection = pipe.start(cfg);
        rs2::sensor selected_sensor = selection.get_device().query_sensors()[1];
        //selected_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        // 禁用自动曝光
       // selected_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

        // 设置曝光时间（单位为微秒，值可以根据需要调整）
        //selected_sensor.set_option(RS2_OPTION_EXPOSURE, 156     );

        qDebug() << "开始获取帧";
        rs2::frameset frames = pipe.wait_for_frames();

        //qDebug() << "已获取到帧";
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        if (!depth || !color) {

//            qDebug() << "未获取到深度帧或颜色帧";
            isSuccess = false;
            pipe.stop();
            return;
        }

        rs2::pointcloud pc;
        pc.map_to(color); // 将点云映射到颜色帧
        auto points_data = pc.calculate(depth);



        auto vertices = points_data.get_vertices();
        auto tex_coords = points_data.get_texture_coordinates();
        // 获取OpenGL函数
        QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

        // 初始化纹理
        f->glGenTextures(1, &textureID);
        f->glBindTexture(GL_TEXTURE_2D, textureID);

        // 更新纹理数据
        f->glBindTexture(GL_TEXTURE_2D, textureID);
        f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, color.get_width(), color.get_height(), 0, GL_RGB, GL_UNSIGNED_BYTE, color.get_data());

        //设置格式
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        glPoint.clear();

        // 获取颜色帧的数据
        const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
        int width = color.get_width();
        int height = color.get_height();

        for (size_t i = 0; i < points_data.size(); ++i) {
            if (vertices[i].z != 0) {  // 过滤掉无效点
                Vertex vertex;
                vertex.position = QVector3D(vertices[i].x, vertices[i].y, vertices[i].z);

                float u = tex_coords[i].u;
                float v = tex_coords[i].v;

                // 存储纹理坐标
                vertex.texCoord = QVector2D(u, v);
                // 将纹理坐标转换为像素坐标
                int x = std::min(std::max(int(u * width + 0.5f), 0), width - 1);
                int y = std::min(std::max(int(v * height + 0.5f), 0), height - 1);

                int idx = (y * width + x) * 3; // RGB格式

                // 获取颜色
                uint8_t red = color_data[idx];
                uint8_t green = color_data[idx + 1];
                uint8_t blue = color_data[idx + 2];

                // 存储颜色
                vertex.color = QVector3D(red / 255.0f, green / 255.0f, blue / 255.0f);

                glPoint.push_back(vertex);
            }
        }
        isSuccess = true;

        pipe.stop();
        qDebug() << "停止管道";

    } catch (const rs2::error &e) {
        qDebug() << "RealSense error: " << e.what();
        isSuccess = false;
    } catch (const std::exception &e) {
        qDebug() << "Exception: " << e.what();
        isSuccess = false;
    } catch (...) {
        qDebug() << "Unknown error";
        isSuccess = false;
    }
}

void loadPointCloudFromRealSense(rs2::points &points, bool &isSuccess)
{
    try {
        rs2::context ctx;
        auto list = ctx.query_devices();
        if (list.size() == 0) {
            qDebug() << "无相机设备连接.";
            isSuccess = false;
            return;
        } else {
            rs2::device device = list.front();
            qDebug() << "设备: " << device.get_info(RS2_CAMERA_INFO_NAME);
//            device.hardware_reset();
        }

        rs2::pipeline pipe;
        rs2::pointcloud pc;

//        qDebug() << "启动流";
        pipe.start();

        qDebug() << "开始获取帧";
        rs2::frameset frames = pipe.wait_for_frames();
//        qDebug() << "已获取到帧";

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        if (!depth || !color) {

//            qDebug() << "未获取到深度帧或颜色帧";
            isSuccess = false;
            pipe.stop();
            return;
        }

        pc.map_to(color); // 将点云映射到颜色帧
        points = pc.calculate(depth);

    } catch (const rs2::error &e) {
        qDebug() << "RealSense error: " << e.what();
        isSuccess = false;
    } catch (const std::exception &e) {
        qDebug() << "Exception: " << e.what();
        isSuccess = false;
    } catch (...) {
        qDebug() << "Unknown error";
        isSuccess = false;
    }

}


