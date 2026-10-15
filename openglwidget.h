#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QOpenGLWidget>
//#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <QVector>
#include <QMouseEvent>
#include <QWheelEvent>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <qgl.h>
#include "rscamera.h"
#include "common.h"

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    // 构造函数和析构函数
    OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget();

    // 加载PCD文件
    bool loadPCDFile(const QString &fileName);

    // 加载OBJ文件
    bool loadOBJFile(const QString &fileName);

    //保存pcd文件
    void savePcdFile(const std::string &filename);

    //设置选择点云标志
    void setSelectPointFlag(bool flagValue);

    //设置角度标志
    void setAngleFlag(bool flagValue);


    //画圆弧
    
    //求圆弧上的点
    void findArcPoints(const std::vector<QVector3D> &controlPoints, std::vector<glm::vec3> &arcPoints, glm::vec3 &center);

    //判断圆弧的方向
    void findArcDirecton(const std::vector<glm::vec3> &arcPoints, const glm::vec3 &center ,std::vector<QVector3D> &directon);

    //画圆弧
    //三个点画
    void drawArc(const std::vector<QVector3D> &controlPoints);

    void drawArc(const std::vector<QVector3D>& controlPoints, const QVector3D& color);

    void drawArc(const std::vector<glm::vec3> arcPoints, const QVector3D& color);

    //通过圆弧点画
    void drawArc(const std::vector<glm::vec3> arcPoints);

    void drawArc(const Point &begin, const Point &pocess, const Point &end, const QVector3D& color);

    //画直线
    void drawLine(const std::vector<QVector3D> &controlPoints);

    void drawLine(const Point& begin, const Point& end);

    void drawLine(const Point &begin, const Point &end, const QVector3D& color);

    void drawLine(const QVector3D& begin, const QVector3D& end, const QVector3D& color);

    //画曲线
    void drawCurve(const std::vector<QVector3D> &controlPoints);

    //画控制点
    void drawArcControlPoint(const std::vector<QVector3D> arcPoints);

    //复位view
    void initialViewPb();

    //设置轨迹拟合标志
    void setTrackFitFlag(bool flagValue, const std::vector<ShapeData> &mShape);

    //三维重建
    bool PointCloudFromRealSense();

    bool PointCloudFromKWSC();

    //画点云
    void draw_pointcloud(QVector<Vertex> pointsCloud);

    int setWeldTrackDis(const double& dis, const GraphicType& weldTrackType, const bool& continueFlag, const bool& continueEndFlag);

    //设置焊接轨迹平移距离
    int setWeldTrackDis(const double &dis,  const GraphicType &weldTrackType, const bool &continueFlag);

    int setPingHanWeldTrackDis(const double& dis, const GraphicType& weldTrackType, const bool& continueFlag, const bool& continueEndFlag, QVector3D zDirection, QVector3D xDirection);

    //计算怕平焊平面
    void calculatePinghanPlane(const int& calAngleFlag, QVector3D &zDirection);

    //设置平焊焊接轨迹平移距离
	int setPingHanWeldTrackDis(const double& dis, const GraphicType& weldTrackType, const bool& continueFlag,  QVector3D zDirection, QVector3D xDirection);

    //计算角度
    void calculateAngle(const int &calAngleFlag, const bool& quashFlag);

    //焊接圆弧轨迹
    void calculateweldArc(const int &weldArcFlag, const bool& quashFlag);

    //标准化设备坐标到相机坐标
    void ndcXYZToCameraXYZ(const QVector3D &ndcXYZ, QVector3D &CameraXYZ);

    // ====== 3D点云直接绘制 ======
    enum Draw3DMode { D3_NONE = 0, D3_LINE, D3_ARC, D3_ELLIPSE, D3_SPLINE };
    void setDraw3DMode(int mode);          // 进入/退出某绘制模式
    void undo3DPoint();                    // 撤销最后一个控制点
    bool finish3DShape();                  // 完成当前图元并生成焊接轨迹
    void clear3DDrawing();                 // 清空当前绘制状态
    int  draw3DMode() const { return m_draw3DMode; }
    int  draw3DControlCount() const { return (int)m_draw3DCtrlPoints.size(); }
    bool last3DFailureWasReported() const { return m_last3DFailureReported; }


public:
    bool selectPointFlag;   //选择点云标志位
    bool angleFlag;   //选择点云标志位
    QVector3D selectedPoint; //被选中的点
    bool needTrackFit;      //轨迹拟合标志位
    std::vector<ShapeData> shapes; //OpenGL需要的shapes
    QVector<Vertex> points; // 点云数据
    QVector<Vertex> pointData; // 点云数据
    std::vector<Point> anglePoints; //储存用于计算角度的点
    std::vector<QVector3D> ArcPoints; //储存用于焊接轨迹为圆弧的原始点
    std::vector<glm::vec3> inArcPoints; //储存圆上的拟合点
    std::vector<QVector3D> arcPointsDirecton; //圆上点的方向
	std::vector<QVector3D> drawAnglePoints; //画角度直线的点
	std::vector<QVector3D> drawPinghanPlanePoints; //画平焊平面的点
	QVector3D pinghanZDirectionInOPengl; //平焊Z方向
	std::vector<QVector3D> planeSelectPoints; //平面选择的点（用于2D绘制）

    // 定义焊接轨迹结构
    struct WeldTrack {
        GraphicType type;
        Point begin, pocess, end;
        QVector3D direction;    //机械臂末端的朝向在相机坐标系下的向量
        QVector3D xDirection;    //机械臂末端的X轴朝向在相机坐标系下的向量
        bool continueFlag;       //是否为连续轨迹
        bool continueEndFlag;    //是否为连续轨迹终点轨迹
		bool baihanFlag; //是否是摆焊
        std::vector<QVector3D> ArcPoints; //储存用于焊接轨迹为圆弧的原始点
        std::vector<QVector3D> arcPointsDirecton; //圆上点的方向
        std::vector<QVector3D> arcPointsXDirecton; //圆上x轴的方向
    };

    std::vector<WeldTrack> weldTrackPoints;  //存储所有焊接轨迹




protected:

    // OpenGL初始化
    void initializeGL() override;
    // OpenGL视口大小变化处理
    void resizeGL(int w, int h) override;
    // OpenGL绘制
    void paintGL() override;
    // 处理鼠标按下事件
    void mousePressEvent(QMouseEvent *event) override;

    // 处理鼠标释放事件
    void mouseReleaseEvent(QMouseEvent* event) override;
    // 处理鼠标移动事件
    void mouseMoveEvent(QMouseEvent *event) override;
    // 处理鼠标滚轮事件
    void wheelEvent(QWheelEvent *event) override;



private:

    QOpenGLBuffer vbo; // 顶点缓冲对象
    QOpenGLShaderProgram shaderProgram; // 着色器程序

    GLuint primitiveVAO = 0;
    GLuint primitiveVBO = 0;


    float zoomFactor; // 缩放因子
    float xOffset; // X轴偏移
    float yOffset; // Y轴偏移
    float xRot; // 绕X轴旋转角度
    float yRot; // 绕Y轴旋转角度

   
 


    QPoint lastMousePosition; // 上次鼠标位置

    glm::mat4 projectionMatrix;
    glm::mat4 viewMatrix;
    glm::mat4 modelMatrix;
    glm::mat4 inverseMVP;

    int selectedPointIndex; // 选中的点的索引

    // ====== 3D点云直接绘制 内部状态/辅助 ======
    bool pickSurfacePoint(const QPoint& pos, QVector3D& outPt, int& outIdx); // 深度优先拾取
    QVector3D surfaceNormalAt(const QVector3D& p);   // 朝相机的单位法线
    bool nearestSurfaceDistance(const QVector3D& p, float& distance);
    bool surfacePathSample(const QVector3D& p, float& distance, float& tolerance);
    bool validateSurfacePath(const std::vector<QVector3D>& path);
    void showSurfaceTrackError();
    void buildKdTree();                              // 为法线估计构建KD-Tree
    void drawDraw3DPreview();                        // 绘制实时预览
    void appendLineTrack(const QVector3D& a, const QVector3D& b, bool cont, bool contEnd);
    void appendArcTrack(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, bool cont, bool contEnd, bool fixedOrientation = false);
    bool buildEllipseTracks();                       // 椭圆弧->多段圆弧轨迹

    int m_draw3DMode = D3_NONE;
    bool m_last3DFailureReported = false;
    std::vector<QVector3D> m_draw3DCtrlPoints;       // 已确认的控制点(已吸附点云)
    QVector3D m_draw3DHoverPoint;                    // 实时hover点
    bool m_draw3DHoverValid = false;
    PointCloudAdapter* m_cloudAdapter = nullptr;     // KD-Tree点云适配器
    my_kd_tree_t* m_kdTree = nullptr;                // 法线估计用KD-Tree

    bool needDrawPoints;     //是否画点云
    bool m_needDrawAngleLine;       //是否画角度直线
    bool m_needDrawWeldArc;   //画焊接圆弧
    bool m_needDrawWeldArcPlane;   //画焊接圆弧
    bool m_needDrawWeldArcControlPoint; //画控制点
	bool m_needDrawPinghanPlane; //画平焊平面

    rs2::points pointsCloud; //点云数据


    GLuint textureID; // 声明 textureID 变量

private:
    // 渲染点云
    void renderPointCloud();

    // 设置着色器
    void setupShaders();

    // 选择点云中的点
    void selectPointCloudPoint(const QPoint &pos);

    //将屏幕坐标转换为世界坐标
    QVector3D unproject(const QPoint &screenPos);

    void findTrackFitPoints(const std::vector<QVector3D> &);

    //轨迹拟合
    void trackFit();

    //焊接轨迹
    void drawWeldTrack();

    //画平焊直线
    void drawPinghanPlane();

    //画角度直线
    void drawAngleLine();

    //画焊接圆弧
    void drawWeldARC();

    //画圆弧所在平面的线
    void drawWeldArcPlane();

    //初始化视图
    void initialView();

};
#endif
