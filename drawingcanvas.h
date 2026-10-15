#ifndef DRAWINGCANVAS_H
#define DRAWINGCANVAS_H

#include <QDialog>
#include <QToolBar>
#include <QAction>
#include <QActionGroup>
#include <QVBoxLayout>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <vector>
#include "planeselection.h"

// 2D绘图区域
class DrawingArea : public QWidget {
    Q_OBJECT
public:
    enum Tool { TOOL_LINE, TOOL_POLYLINE, TOOL_RECTANGLE, TOOL_ELLIPSE, TOOL_POLYGON, TOOL_ARC };

    explicit DrawingArea(const PlaneSelection& plane, QWidget* parent = nullptr);
    void setCurrentTool(Tool tool);
    void undoLastTrack();
    std::vector<DrawingTrack2D> getTracks() const { return m_tracks; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    PlaneSelection m_plane;
    Tool m_currentTool;

    // 视口变换
    double m_scale;
    QPointF m_offset;       // 视图中心在2D坐标系中的位置
    QPoint m_lastMousePos;
    bool m_panning;

    // 绘制状态
    std::vector<QPointF> m_currentPoints;  // 正在绘制的控制点
    bool m_isDrawing;
    QPointF m_mousePos2D;   // 鼠标当前的2D坐标

    // 已完成的轨迹
    std::vector<DrawingTrack2D> m_tracks;

    // 坐标转换
    QPointF screenTo2D(const QPoint& screenPos) const;
    QPoint toScreen(const QPointF& pos2D) const;
    void fitToView();
    void finishCurrentTrack();

    // 绘制辅助
    void drawTrack(QPainter& painter, const DrawingTrack2D& track, const QPen& pen);
    void drawCurrentDrawing(QPainter& painter);
};

// 2D轨迹绘制对话框
class DrawingCanvas : public QDialog {
    Q_OBJECT
public:
    explicit DrawingCanvas(const PlaneSelection& plane, QWidget* parent = nullptr);

signals:
    void tracksExported(const std::vector<DrawingTrack2D>& tracks);

private:
    DrawingArea* m_drawingArea;
    void onExport();
};

#endif // DRAWINGCANVAS_H
