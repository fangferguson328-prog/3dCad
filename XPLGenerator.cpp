#include "XPLGenerator.h"

// 构造函数，初始化 XML 文档结构
XPLGenerator::XPLGenerator(const std::string& tool_name)
    : doc_(), tool_name_(tool_name)
{
    initializeDocument();
}

// 初始化 XML 文档结构
void XPLGenerator::initializeDocument() {
    doc_.InsertEndChild(doc_.NewDeclaration());

    XMLElement* root = doc_.NewElement("Xpl-source");
    doc_.InsertEndChild(root);

    root->InsertEndChild(createHostEnvironment());
    root->InsertEndChild(createXplEnvironment());
}

// 创建 HostEnvironment 节点
XMLElement* XPLGenerator::createHostEnvironment() {
    XMLElement* host = doc_.NewElement("HostEnvironment");
    XMLElement* config = doc_.NewElement("Config");

    config->InsertEndChild(createTextElement("FileId", "0x2fb23202"));
    config->InsertEndChild(createTextElement("Target", "Robox Motion Control"));
    config->InsertEndChild(createTextElement("XplType", "RoboticProgramLanguage"));

    host->InsertEndChild(config);
    return host;
}

// 创建 XplEnvironment 节点
XMLElement* XPLGenerator::createXplEnvironment() {
    XMLElement* xplEnv = doc_.NewElement("XplEnvironment");
    XMLElement* info = doc_.NewElement("Info");

    info->InsertEndChild(createCDATASection("Title", "XPL program", 1033));
    info->InsertEndChild(createCDATASection("Description", "XPL program edited from teachgun", 1033));
    info->InsertEndChild(createTextElement("Version", "1.0.0"));
    info->InsertEndChild(createTextElement("Author", "Robox SpA"));

    xplEnv->InsertEndChild(info);

    XMLElement* pous = doc_.NewElement("Pous");
    XMLElement* pou = doc_.NewElement("Pou");
    pou->InsertEndChild(createTextElement("Name", "Main"));
    pou->InsertEndChild(createTextElement("Type", "Program"));
    pou->InsertEndChild(doc_.NewElement("Body"));
    pous->InsertEndChild(pou);

    xplEnv->InsertEndChild(pous);
    return xplEnv;
}

// 添加关节运动指令
void XPLGenerator::addJointMotion(const std::vector<double>& joint_angles,
    const std::string& speed,
    const std::string& zone) {
    // 创建 <mjoint> 元素
    XMLElement* mjoint = createMotionElement("mjoint", joint_angles);
    addMotionParameters(mjoint, speed, zone);
    getBodyElement()->InsertEndChild(mjoint);
}

// 修改关节运动指令
void XPLGenerator::modifyJointMotion(const std::vector<double>& joint_angles, const std::string& speed, const std::string& zone)
{
    // 查找 <Body> 节点
    XMLElement* body = getBodyElement();
    if (!body) { 
		
        return;
    }

    // 查找现有的 <mjoint> 元素
    XMLElement* existingMjoint = body->FirstChildElement("mjoint");
    if (existingMjoint) {
        // 修改目标点
        XMLElement* target = existingMjoint->FirstChildElement("target");
        if (target) {
            target->SetText(formatPointString(joint_angles).c_str());
        }
        else {
            // 如果没有 <target> 子元素，则创建并插入
            existingMjoint->InsertEndChild(createPointElement("target", joint_angles));
        }

        // 修改速度
        XMLElement* speedElem = existingMjoint->FirstChildElement("speed");
        if (speedElem) {
            speedElem->SetText(speed.c_str());
        }
        else {
            existingMjoint->InsertEndChild(createTextElement("speed", speed.c_str()));
        }

        // 修改区域
        XMLElement* zoneElem = existingMjoint->FirstChildElement("zone");
        if (zoneElem) {
            zoneElem->SetText(zone.c_str());
        }
        else {
            existingMjoint->InsertEndChild(createTextElement("zone", zone.c_str()));
        }
    }
    else {
        // 如果没有现有的 <mjoint> 元素，则创建并插入
        XMLElement* mjoint = createMotionElement("mjoint", joint_angles);
        addMotionParameters(mjoint, speed, zone);
        body->InsertEndChild(mjoint);
    }
}

// 添加直线运动指令
void XPLGenerator::addLinearMotion(const std::vector<double>& position,
    const std::vector<double>& euler_angles,
    const std::string& speed,
    const std::string& zone)
{
    XMLElement* mlin = createMotionElement("mlin", position, euler_angles);
    addMotionParameters(mlin, speed, zone);
    getBodyElement()->InsertEndChild(mlin);
}

// 添加圆弧运动指令
void XPLGenerator::addCircularMotion(const std::vector<double>& aux_position,
    const std::vector<double>& aux_euler,
    const std::vector<double>& target_position,
    const std::vector<double>& target_euler,
    const std::string& speed,
    const std::string& zone)
{
    XMLElement* mcirc = doc_.NewElement("mcirc");
    mcirc->InsertEndChild(createPointElement("aux", aux_position, aux_euler));
    mcirc->InsertEndChild(createPointElement("target", target_position, target_euler));
    addMotionParameters(mcirc, speed, zone);
    getBodyElement()->InsertEndChild(mcirc);
}

// 添加焊接起弧指令（方案 B 结构）
void XPLGenerator::addArcStart(int weld_file_num) {
    XMLElement* call = doc_.NewElement("call");

    XMLElement* sub = doc_.NewElement("subroutine");

    // <name>arcweld.ArcOn</name>
    XMLElement* nameElem = doc_.NewElement("name");
    nameElem->SetText("arcweld.ArcOn");
    sub->InsertEndChild(nameElem);

    // <args><arg>0</arg></args>
    XMLElement* argsElem = doc_.NewElement("args");
    XMLElement* arg = doc_.NewElement("arg");
    arg->SetText(weld_file_num);
    argsElem->InsertEndChild(arg);
    sub->InsertEndChild(argsElem);

    call->InsertEndChild(sub);
    getBodyElement()->InsertEndChild(call);
}

// 添加焊接收弧指令（方案 B 结构）
void XPLGenerator::addArcEnd() {
    XMLElement* call = doc_.NewElement("call");

    XMLElement* sub = doc_.NewElement("subroutine");

    // <name>arcweld.ArcOff</name>
    XMLElement* nameElem = doc_.NewElement("name");
    nameElem->SetText("arcweld.ArcOff");
    sub->InsertEndChild(nameElem);

    // 没有参数就不用加 <args>，如果控制器要求空 args，可选
    call->InsertEndChild(sub);
    getBodyElement()->InsertEndChild(call);
}
// 添加焊接起弧指令（方案 B 结构）
void XPLGenerator::addWeaveStart(int weave_file_num) {
    XMLElement* call = doc_.NewElement("call");

    XMLElement* sub = doc_.NewElement("subroutine");

    // <name>arcweld.ArcOn</name>
    XMLElement* nameElem = doc_.NewElement("name");
    nameElem->SetText("arcweld.WeaveOn");
    sub->InsertEndChild(nameElem);

    // <args><arg>0</arg></args>
    XMLElement* argsElem = doc_.NewElement("args");
    XMLElement* arg = doc_.NewElement("arg");
    arg->SetText(weave_file_num);
    argsElem->InsertEndChild(arg);
    sub->InsertEndChild(argsElem);

    call->InsertEndChild(sub);
    getBodyElement()->InsertEndChild(call);
}

// 添加焊接收弧指令（方案 B 结构）
void XPLGenerator::addWeaveEnd() {
    XMLElement* call = doc_.NewElement("call");

    XMLElement* sub = doc_.NewElement("subroutine");

    // <name>arcweld.ArcOff</name>
    XMLElement* nameElem = doc_.NewElement("name");
    nameElem->SetText("arcweld.WeaveOff");
    sub->InsertEndChild(nameElem);

    // 没有参数就不用加 <args>，如果控制器要求空 args，可选
    call->InsertEndChild(sub);
    getBodyElement()->InsertEndChild(call);
}


// 批量生成焊接路径程序
void XPLGenerator::generateWeldingPath(const std::vector<std::vector<double>>& path_points) {
    if (path_points.empty()) return;

    addLinearMotion({ path_points[0][0], path_points[0][1], path_points[0][2] },
        { path_points[0][3], path_points[0][4], path_points[0][5] }, "v20");
    addArcStart();

    for (size_t i = 1; i < path_points.size() - 1; ++i) {
        addLinearMotion({ path_points[i][0], path_points[i][1], path_points[i][2] },
            { path_points[i][3], path_points[i][4], path_points[i][5] }, "v5");
    }

    addLinearMotion({ path_points.back()[0], path_points.back()[1], path_points.back()[2] },
        { path_points.back()[3], path_points.back()[4], path_points.back()[5] }, "v20");
    addArcEnd();
}

// 保存 XML 文件
bool XPLGenerator::saveToFile(const std::string& filename) {
    return doc_.SaveFile(filename.c_str()) == XML_SUCCESS;
}

// 创建运动元素
XMLElement* XPLGenerator::createMotionElement(const char* type,
    const std::vector<double>& jointAngles)
{
    XMLElement* elem = doc_.NewElement(type);
    elem->InsertEndChild(createPointElement("target", jointAngles));
    return elem;
}

// 创建运动元素
XMLElement* XPLGenerator::createMotionElement(const char* type,
    const std::vector<double>& pos,
    const std::vector<double>& euler)
{
    XMLElement* elem = doc_.NewElement(type);
    elem->InsertEndChild(createPointElement("target", pos, euler));
    return elem;
}

// 创建角度元素
XMLElement* XPLGenerator::createPointElement(const char* type,
    const std::vector<double>& jointAngles)
{
    XMLElement* point = doc_.NewElement(type);
    point->SetText(formatPointString(jointAngles).c_str());
    return point;
}

// 创建点元素
XMLElement* XPLGenerator::createPointElement(const char* type,
    const std::vector<double>& pos,
    const std::vector<double>& euler)
{
    XMLElement* point = doc_.NewElement(type);
    point->SetText(formatPointString(pos, euler).c_str());
    return point;
}

// jointAngles格式化字符串
std::string XPLGenerator::formatPointString(const std::vector<double>& jointAngles) const
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "POINTJ(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5]);
    return buffer;
}


// 坐标+欧拉角格式化字符串
std::string XPLGenerator::formatPointString(const std::vector<double>& pos,
    const std::vector<double>& euler) const
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "POINTC(%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,\"%s\",0,0,0)",
        pos[0], pos[1], pos[2], euler[0], euler[1], euler[2], "CFG0");
    return buffer;
}

// 添加运动参数
void XPLGenerator::addMotionParameters(XMLElement* elem,
    const std::string& speed,
    const std::string& zone)
{
    elem->InsertEndChild(createTextElement("speed", speed.c_str()));
    elem->InsertEndChild(createTextElement("zone", zone.c_str()));
    elem->InsertEndChild(createTextElement("tool", tool_name_.c_str()));
}

// 获取 Body 节点
XMLElement* XPLGenerator::getBodyElement() {
    return doc_.FirstChildElement("Xpl-source")
        ->FirstChildElement("XplEnvironment")
        ->FirstChildElement("Pous")
        ->FirstChildElement("Pou")
        ->FirstChildElement("Body");
}

// 创建简单文本元素
XMLElement* XPLGenerator::createTextElement(const char* name, const char* value) {
    XMLElement* elem = doc_.NewElement(name);
    elem->SetText(value);
    return elem;
}

// 创建 CDATA 节点
XMLElement* XPLGenerator::createCDATASection(const char* name,
    const char* value,
    int lcid)
{
    XMLElement* elem = doc_.NewElement(name);
    elem->SetAttribute("LcId", lcid);
    XMLText* text = doc_.NewText(value);
    text->SetCData(true);
    elem->InsertEndChild(text);
    return elem;
}
