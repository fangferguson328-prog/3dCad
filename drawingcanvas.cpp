#include "drawingcanvas.h"
#include <QPushButton>
#include <QLabel>
#include <QMessageBox>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==================== DrawingArea ====================

DrawingArea::DrawingArea(const PlaneSelection& plane, QWidget* parent)
    : QWidget(parent), m_plane(plane), m_currentTool(TOOL_LINE),
      m_scale(1.0), m_panning(false), m_isDrawing(false)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    m_offset = m_plane.bounds2D.center();
    setMinimumSize(600, 400);
}

void DrawingArea::setCurrentTool(Tool tool)
{
    if (m_isDrawing) {
        m_currentPoints.clear();
        m_isDrawing = false;
    }
    m_currentTool = tool;
    update();
}

void DrawingArea::undoLastTrack()
{
    if (m_isDrawing) {
        m_currentPoints.clear();
        m_isDrawing = false;
    } else if (!m_tracks.empty()) {
        m_tracks.pop_back();
    }
    update();
}

QPointF DrawingArea::screenTo2D(const QPoint& screenPos) const
{
    double x2d = (screenPos.x() - width() / 2.0) / m_scale + m_offset.x();
    double y2d = (height() / 2.0 - screenPos.y()) / m_scale + m_offset.y();
    return QPointF(x2d, y2d);
}

QPoint DrawingArea::toScreen(const QPointF& pos2D) const
{
    int sx = (int)((pos2D.x() - m_offset.x()) * m_scale + width() / 2.0);
    int sy = (int)(height() / 2.0 - (pos2D.y() - m_offset.y()) * m_scale);
    return QPoint(sx, sy);
}

void DrawingArea::fitToView()
{
    if (!m_plane.isValid) return;
    double boundsW = m_plane.bounds2D.width();
    double boundsH = m_plane.bounds2D.height();
    if (boundsW < 1e-9 || boundsH < 1e-9) return;

    double scaleX = width() / boundsW;
    double scaleY = height() / boundsH;
    m_scale = std::min(scaleX, scaleY) * 0.85;
    m_offset = m_plane.bounds2D.center();
}

void DrawingArea::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    fitToView();
}

void DrawingArea::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor(30, 30, 30));

    // 绘制网格
    QPen gridPen(QColor(60, 60, 60), 1);
    painter.setPen(gridPen);

    double rawStep = 50.0 / m_scale;
    double orderOfMag = std::pow(10.0, std::floor(std::log10(rawStep)));
    double gridStep = orderOfMag;
    if (rawStep / orderOfMag > 5) gridStep = orderOfMag * 5;
    else if (rawStep / orderOfMag > 2) gridStep = orderOfMag * 2;

    QPointF topLeft2D = screenTo2D(QPoint(0, 0));
    QPointF bottomRight2D = screenTo2D(QPoint(width(), height()));
    double minX = std::min(topLeft2D.x(), bottomRight2D.x());
    double maxX = std::max(topLeft2D.x(), bottomRight2D.x());
    double minY = std::min(topLeft2D.y(), bottomRight2D.y());
    double maxY = std::max(topLeft2D.y(), bottomRight2D.y());

    double startX = std::floor(minX / gridStep) * gridStep;
    double startY = std::floor(minY / gridStep) * gridStep;

    for (double x = startX; x <= maxX; x += gridStep) {
        QPoint s1 = toScreen(QPointF(x, minY));
        QPoint s2 = toScreen(QPointF(x, maxY));
        painter.drawLine(s1, s2);
    }
    for (double y = startY; y <= maxY; y += gridStep) {
        QPoint s1 = toScreen(QPointF(minX, y));
        QPoint s2 = toScreen(QPointF(maxX, y));
        painter.drawLine(s1, s2);
    }

    // 绘制边界矩形
    QPen boundaryPen(QColor(0, 255, 0), 2, Qt::DashLine);
    painter.setPen(boundaryPen);
    QPoint c0 = toScreen(QPointF(m_plane.bounds2D.left(), m_plane.bounds2D.top()));
    QPoint c1 = toScreen(QPointF(m_plane.bounds2D.right(), m_plane.bounds2D.top()));
    QPoint c2 = toScreen(QPointF(m_plane.bounds2D.right(), m_plane.bounds2D.bottom()));
    QPoint c3 = toScreen(QPointF(m_plane.bounds2D.left(), m_plane.bounds2D.bottom()));
    painter.drawLine(c0, c1);
    painter.drawLine(c1, c2);
    painter.drawLine(c2, c3);
    painter.drawLine(c3, c0);

    // 绘制四角标记
    QPen cornerPen(QColor(255, 255, 0), 1);
    painter.setPen(cornerPen);
    QBrush cornerBrush(QColor(255, 255, 0));
    painter.setBrush(cornerBrush);
    for (int i = 0; i < 4; i++) {
        QPointF cp2d = projectTo2D(m_plane.corners[i], m_plane);
        QPoint sp = toScreen(cp2d);
        painter.drawEllipse(sp, 5, 5);
        painter.drawText(sp + QPoint(8, -8), QString("P%1").arg(i));
    }
    painter.setBrush(Qt::NoBrush);

    // 绘制已完成轨迹
    QPen trackPen(QColor(0, 180, 255), 2);
    for (const auto& track : m_tracks) {
        drawTrack(painter, track, trackPen);
    }

    // 绘制正在绘制的轨迹
    drawCurrentDrawing(painter);

    // 绘制十字光标
    QPen crossPen(QColor(200, 200, 200, 100), 1, Qt::DotLine);
    painter.setPen(crossPen);
    QPoint mouseScreen = mapFromGlobal(QCursor::pos());
    if (rect().contains(mouseScreen)) {
        painter.drawLine(QPoint(mouseScreen.x(), 0), QPoint(mouseScreen.x(), height()));
        painter.drawLine(QPoint(0, mouseScreen.y()), QPoint(width(), mouseScreen.y()));
    }

    // 绘制坐标信息
    painter.setPen(QColor(200, 200, 200));
    QPointF pos2d = screenTo2D(mouseScreen);
    painter.drawText(10, height() - 10,
        QString("X: %1  Y: %2").arg(pos2d.x(), 0, 'f', 4).arg(pos2d.y(), 0, 'f', 4));

    // 绘制当前工具提示
    QString toolNames[] = {
        QString::fromLocal8Bit("直线"),
        QString::fromLocal8Bit("多段线"),
        QString::fromLocal8Bit("矩形"),
        QString::fromLocal8Bit("椭圆"),
        QString::fromLocal8Bit("多边形"),
        QString::fromLocal8Bit("圆弧(3点)")
    };
    painter.drawText(10, 20,
        QString::fromLocal8Bit("当前工具: %1 | 轨迹数: %2")
            .arg(toolNames[m_currentTool]).arg(m_tracks.size()));
}

void DrawingArea::drawTrack(QPainter& painter, const DrawingTrack2D& track, const QPen& pen)
{
    painter.setPen(pen);
    const auto& pts = track.points;
    if (pts.empty()) return;

    switch (track.type) {
    case DrawingTrack2D::DT_LINE:
        if (pts.size() >= 2)
            painter.drawLine(toScreen(pts[0]), toScreen(pts[1]));
        break;
    case DrawingTrack2D::DT_POLYLINE:
        for (size_t i = 0; i + 1 < pts.size(); i++)
            painter.drawLine(toScreen(pts[i]), toScreen(pts[i + 1]));
        break;
    case DrawingTrack2D::DT_RECTANGLE:
        if (pts.size() >= 2) {
            QPoint s0 = toScreen(pts[0]);
            QPoint s1 = toScreen(QPointF(pts[1].x(), pts[0].y()));
            QPoint s2 = toScreen(pts[1]);
            QPoint s3 = toScreen(QPointF(pts[0].x(), pts[1].y()));
            painter.drawLine(s0, s1);
            painter.drawLine(s1, s2);
            painter.drawLine(s2, s3);
            painter.drawLine(s3, s0);
        }
        break;
    case DrawingTrack2D::DT_ELLIPSE:
        if (pts.size() >= 2) {
            QPointF center = pts[0];
            double rx = std::abs(pts[1].x() - pts[0].x());
            double ry = std::abs(pts[1].y() - pts[0].y());
            int segments = 64;
            QPoint prev = toScreen(QPointF(center.x() + rx, center.y()));
            for (int i = 1; i <= segments; i++) {
                double angle = 2.0 * M_PI * i / segments;
                QPointF ep(center.x() + rx * std::cos(angle),
                           center.y() + ry * std::sin(angle));
                QPoint cur = toScreen(ep);
                painter.drawLine(prev, cur);
                prev = cur;
            }
        }
        break;
    case DrawingTrack2D::DT_POLYGON:
        for (size_t i = 0; i + 1 < pts.size(); i++)
            painter.drawLine(toScreen(pts[i]), toScreen(pts[i + 1]));
        if (pts.size() >= 3)
            painter.drawLine(toScreen(pts.back()), toScreen(pts.front()));
        break;
    case DrawingTrack2D::DT_ARC:
        if (pts.size() >= 3) {
            int arcSegs = 32;
            QPointF p0 = pts[0], p1 = pts[1], p2 = pts[2];
            double ax = p0.x(), ay = p0.y(), bx = p1.x(), by = p1.y(), cx = p2.x(), cy = p2.y();
            double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
            if (std::abs(D) > 1e-12) {
                double ux = ((ax*ax + ay*ay) * (by - cy) + (bx*bx + by*by) * (cy - ay) + (cx*cx + cy*cy) * (ay - by)) / D;
                double uy = ((ax*ax + ay*ay) * (cx - bx) + (bx*bx + by*by) * (ax - cx) + (cx*cx + cy*cy) * (bx - ax)) / D;
                double r = std::sqrt((ax - ux)*(ax - ux) + (ay - uy)*(ay - uy));
                double ang0 = std::atan2(ay - uy, ax - ux);
                double ang1 = std::atan2(by - uy, bx - ux);
                double ang2 = std::atan2(cy - uy, cx - ux);
                double sweep = ang2 - ang0;
                double mid = ang1 - ang0;
                if (mid < 0) mid += 2*M_PI; if (sweep < 0) sweep += 2*M_PI;
                if (mid > sweep) { sweep -= 2*M_PI; }
                QPoint prev = toScreen(pts[0]);
                for (int si = 1; si <= arcSegs; si++) {
                    double a = ang0 + sweep * si / arcSegs;
                    QPoint cur = toScreen(QPointF(ux + r*std::cos(a), uy + r*std::sin(a)));
                    painter.drawLine(prev, cur); prev = cur;
                }
            } else {
                painter.drawLine(toScreen(pts[0]), toScreen(pts[2]));
            }
        }
        break;
    }
}

void DrawingArea::drawCurrentDrawing(QPainter& painter)
{
    if (!m_isDrawing || m_currentPoints.empty()) return;

    QPen drawPen(QColor(255, 100, 100), 2, Qt::DashLine);
    painter.setPen(drawPen);

    // 绘制控制点标记
    QBrush pointBrush(QColor(255, 100, 100));
    for (const auto& pt : m_currentPoints) {
        painter.setBrush(pointBrush);
        painter.drawEllipse(toScreen(pt), 3, 3);
    }
    painter.setBrush(Qt::NoBrush);
    painter.setPen(drawPen);

    switch (m_currentTool) {
    case TOOL_LINE:
        if (m_currentPoints.size() == 1)
            painter.drawLine(toScreen(m_currentPoints[0]), toScreen(m_mousePos2D));
        break;
    case TOOL_POLYLINE:
        for (size_t i = 0; i + 1 < m_currentPoints.size(); i++)
            painter.drawLine(toScreen(m_currentPoints[i]), toScreen(m_currentPoints[i + 1]));
        if (!m_currentPoints.empty())
            painter.drawLine(toScreen(m_currentPoints.back()), toScreen(m_mousePos2D));
        break;
    case TOOL_RECTANGLE:
        if (m_currentPoints.size() == 1) {
            QPoint s0 = toScreen(m_currentPoints[0]);
            QPoint s1 = toScreen(QPointF(m_mousePos2D.x(), m_currentPoints[0].y()));
            QPoint s2 = toScreen(m_mousePos2D);
            QPoint s3 = toScreen(QPointF(m_currentPoints[0].x(), m_mousePos2D.y()));
            painter.drawLine(s0, s1);
            painter.drawLine(s1, s2);
            painter.drawLine(s2, s3);
            painter.drawLine(s3, s0);
        }
        break;
    case TOOL_ELLIPSE:
        if (m_currentPoints.size() == 1) {
            QPointF center = m_currentPoints[0];
            double rx = std::abs(m_mousePos2D.x() - center.x());
            double ry = std::abs(m_mousePos2D.y() - center.y());
            int segments = 64;
            QPoint prev = toScreen(QPointF(center.x() + rx, center.y()));
            for (int i = 1; i <= segments; i++) {
                double angle = 2.0 * M_PI * i / segments;
                QPointF ep(center.x() + rx * std::cos(angle),
                           center.y() + ry * std::sin(angle));
                QPoint cur = toScreen(ep);
                painter.drawLine(prev, cur);
                prev = cur;
            }
        }
        break;
    case TOOL_POLYGON:
        for (size_t i = 0; i + 1 < m_currentPoints.size(); i++)
            painter.drawLine(toScreen(m_currentPoints[i]), toScreen(m_currentPoints[i + 1]));
        if (!m_currentPoints.empty())
            painter.drawLine(toScreen(m_currentPoints.back()), toScreen(m_mousePos2D));
        break;
    case TOOL_ARC:
        for (const auto& pt : m_currentPoints) {
            painter.setBrush(pointBrush);
            painter.drawEllipse(toScreen(pt), 4, 4);
        }
        painter.setBrush(Qt::NoBrush);
        if (m_currentPoints.size() == 1)
            painter.drawLine(toScreen(m_currentPoints[0]), toScreen(m_mousePos2D));
        else if (m_currentPoints.size() == 2) {
            painter.drawLine(toScreen(m_currentPoints[0]), toScreen(m_currentPoints[1]));
            painter.drawLine(toScreen(m_currentPoints[1]), toScreen(m_mousePos2D));
        }
        break;
    }
}

void DrawingArea::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        m_panning = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        return;
    }

    if (event->button() != Qt::LeftButton) return;

    QPointF pos2D = screenTo2D(event->pos());

    switch (m_currentTool) {
    case TOOL_LINE:
        if (!m_isDrawing) {
            m_currentPoints.clear();
            m_currentPoints.push_back(pos2D);
            m_isDrawing = true;
        } else {
            m_currentPoints.push_back(pos2D);
            finishCurrentTrack();
        }
        break;
    case TOOL_POLYLINE:
    case TOOL_POLYGON:
        if (!m_isDrawing) {
            m_currentPoints.clear();
            m_isDrawing = true;
        }
        m_currentPoints.push_back(pos2D);
        break;
    case TOOL_RECTANGLE:
    case TOOL_ELLIPSE:
        if (!m_isDrawing) {
            m_currentPoints.clear();
            m_currentPoints.push_back(pos2D);
            m_isDrawing = true;
        } else {
            m_currentPoints.push_back(pos2D);
            finishCurrentTrack();
        }
        break;
    case TOOL_ARC:
        if (!m_isDrawing) {
            m_currentPoints.clear();
            m_currentPoints.push_back(pos2D);
            m_isDrawing = true;
        } else {
            m_currentPoints.push_back(pos2D);
            if (m_currentPoints.size() >= 3)
                finishCurrentTrack();
        }
        break;
    }
    update();
}

void DrawingArea::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton && m_panning) {
        m_panning = false;
        setCursor(Qt::ArrowCursor);
    }
}

void DrawingArea::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) return;

    if (m_currentTool == TOOL_POLYLINE || m_currentTool == TOOL_POLYGON) {
        if (m_isDrawing && m_currentPoints.size() >= 2) {
            finishCurrentTrack();
        }
    }
    update();
}

void DrawingArea::mouseMoveEvent(QMouseEvent* event)
{
    m_mousePos2D = screenTo2D(event->pos());

    if (m_panning) {
        QPoint delta = event->pos() - m_lastMousePos;
        m_offset.rx() -= delta.x() / m_scale;
        m_offset.ry() += delta.y() / m_scale;
        m_lastMousePos = event->pos();
    }

    update();
}

void DrawingArea::wheelEvent(QWheelEvent* event)
{
    double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    m_scale *= factor;
    update();
}

void DrawingArea::finishCurrentTrack()
{
    DrawingTrack2D track;
    switch (m_currentTool) {
    case TOOL_LINE:      track.type = DrawingTrack2D::DT_LINE;      break;
    case TOOL_POLYLINE:  track.type = DrawingTrack2D::DT_POLYLINE;  break;
    case TOOL_RECTANGLE: track.type = DrawingTrack2D::DT_RECTANGLE; break;
    case TOOL_ELLIPSE:   track.type = DrawingTrack2D::DT_ELLIPSE;   break;
    case TOOL_POLYGON:   track.type = DrawingTrack2D::DT_POLYGON;   break;
    case TOOL_ARC:       track.type = DrawingTrack2D::DT_ARC;       break;
    }
    track.points = m_currentPoints;
    m_tracks.push_back(track);
    m_currentPoints.clear();
    m_isDrawing = false;
}

// ==================== DrawingCanvas ====================

DrawingCanvas::DrawingCanvas(const PlaneSelection& plane, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QString::fromLocal8Bit("2D轨迹绘制"));
    resize(900, 700);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // 工具栏
    QToolBar* toolbar = new QToolBar(this);
    QActionGroup* toolGroup = new QActionGroup(this);

    QAction* actLine = toolbar->addAction(QString::fromLocal8Bit("直线"));
    actLine->setCheckable(true);
    actLine->setChecked(true);
    toolGroup->addAction(actLine);

    QAction* actPolyline = toolbar->addAction(QString::fromLocal8Bit("多段线"));
    actPolyline->setCheckable(true);
    toolGroup->addAction(actPolyline);

    QAction* actRect = toolbar->addAction(QString::fromLocal8Bit("矩形"));
    actRect->setCheckable(true);
    toolGroup->addAction(actRect);

    QAction* actEllipse = toolbar->addAction(QString::fromLocal8Bit("椭圆"));
    actEllipse->setCheckable(true);
    toolGroup->addAction(actEllipse);

    QAction* actPolygon = toolbar->addAction(QString::fromLocal8Bit("多边形"));
    actPolygon->setCheckable(true);
    toolGroup->addAction(actPolygon);

    QAction* actArc = toolbar->addAction(QString::fromLocal8Bit("圆弧(3点)"));
    actArc->setCheckable(true);
    toolGroup->addAction(actArc);

    toolbar->addSeparator();

    QAction* actUndo = toolbar->addAction(QString::fromLocal8Bit("撤销(Ctrl+Z)"));

    toolbar->addSeparator();

    QAction* actExport = toolbar->addAction(QString::fromLocal8Bit(">>> 导出轨迹 <<<"));

    mainLayout->addWidget(toolbar);

    // 绘图区域
    m_drawingArea = new DrawingArea(plane, this);
    mainLayout->addWidget(m_drawingArea);

    // 提示标签
    QLabel* tipLabel = new QLabel(QString::fromLocal8Bit(
        "操作: 左键绘制 | 右键拖动平移 | 滚轮缩放 | 双击结束多段线/多边形"), this);
    tipLabel->setStyleSheet("background: #333; color: #aaa; padding: 4px;");
    mainLayout->addWidget(tipLabel);

    // 连接工具按钮
    connect(actLine, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_LINE);
    });
    connect(actPolyline, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_POLYLINE);
    });
    connect(actRect, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_RECTANGLE);
    });
    connect(actEllipse, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_ELLIPSE);
    });
    connect(actPolygon, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_POLYGON);
    });
    connect(actArc, &QAction::triggered, [this]() {
        m_drawingArea->setCurrentTool(DrawingArea::TOOL_ARC);
    });
    connect(actUndo, &QAction::triggered, [this]() {
        m_drawingArea->undoLastTrack();
    });
    connect(actExport, &QAction::triggered, this, &DrawingCanvas::onExport);
}

void DrawingCanvas::onExport()
{
    auto tracks = m_drawingArea->getTracks();
    if (tracks.empty()) {
        QMessageBox::information(this,
            QString::fromLocal8Bit("提示"),
            QString::fromLocal8Bit("没有可导出的轨迹，请先绘制轨迹。"));
        return;
    }
    emit tracksExported(tracks);
    accept();
}
