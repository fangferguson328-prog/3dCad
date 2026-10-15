#ifndef XPLGENERATOR_H
#define XPLGENERATOR_H

#include <tinyxml2.h>
#include <string>
#include <vector>

// 使用 tinyxml2 命名空间
using namespace tinyxml2;

// XPL 焊接程序生成器类
class XPLGenerator {
public:
    // 构造函数，允许指定焊枪名称，默认 "tool1"
    explicit XPLGenerator(const std::string& tool_name = "tool1");
    // 创建 Program 节点
    void addJointMotion(const std::vector<double>& joint_angles, const std::string& speed, const std::string& zone);

    //修改关节运动指令
	void modifyJointMotion(const std::vector<double>& joint_angles,
		const std::string& speed = "v100",
		const std::string& zone = "fine");

    // 添加直线运动指令
    void addLinearMotion(const std::vector<double>& position,
        const std::vector<double>& euler_angles,
        const std::string& speed = "v100",
        const std::string& zone = "fine");

    // 添加圆弧运动指令
    void addCircularMotion(const std::vector<double>& aux_position,
        const std::vector<double>& aux_euler,
        const std::vector<double>& target_position,
        const std::vector<double>& target_euler,
        const std::string& speed = "v100",
        const std::string& zone = "fine");

    // 添加焊接起弧指令
    void addArcStart(int weld_file_num = 0);

    // 添加焊接收弧指令
    void addArcEnd();

    // 添加打开摆焊指令
    void addWeaveStart(int weave_file_num = 0);

    // 添加关闭摆焊指令
    void addWeaveEnd();

    // 批量生成焊接路径程序
    void generateWeldingPath(const std::vector<std::vector<double>>& path_points);

    // 将生成的 XPL 文件保存到磁盘
    bool saveToFile(const std::string& filename);

    XMLElement* createMotionElement(const char* type, const std::vector<double>& jointAngles);

    // 初始化 XML 文档结构
    void initializeDocument();

    // 工具名称（如焊枪名）
    std::string tool_name_;



private:
    // XML 文档对象
    tinyxml2::XMLDocument doc_;
   

    

    // 创建 HostEnvironment 节点
    XMLElement* createHostEnvironment();

    // 创建 XplEnvironment 节点
    XMLElement* createXplEnvironment();

	

    // 创建运动指令元素（直线/圆弧）
    XMLElement* createMotionElement(const char* type,
        const std::vector<double>& pos,
        const std::vector<double>& euler);

    XMLElement* createPointElement(const char* type, const std::vector<double>& jointAngles);

    // 创建点元素（辅助点/目标点）
    XMLElement* createPointElement(const char* type,
        const std::vector<double>& pos,
        const std::vector<double>& euler);

    std::string formatPointString(const std::vector<double>& jointAngles) const;

    // 将坐标和欧拉角格式化成字符串
    std::string formatPointString(const std::vector<double>& pos,
        const std::vector<double>& euler) const;

    // 添加速度、区域、工具参数到运动元素中
    void addMotionParameters(XMLElement* elem,
        const std::string& speed,
        const std::string& zone);

    // 获取程序 Body 节点
    XMLElement* getBodyElement();

    // 创建简单文本元素
    XMLElement* createTextElement(const char* name, const char* value);

    // 创建 CDATA 节点
    XMLElement* createCDATASection(const char* name,
        const char* value,
        int lcid);
};

#endif // XPLGENERATOR_H
