#ifndef RSCAMERAL_H
#define RSCAMERAL_H
#include <librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include "common.h"

#include <QVector>
#include <QVector3D>

#include <QOpenGLFunctions>

void loadPointCloudFromKWSC(QVector<Vertex>& glPoint, bool& isSuccess, GLuint& textureID);

void loadPointCloudFromRealSense(std::vector<Vertex> &glPoint, bool &isSuccess, GLuint &textureID);

void loadPointCloudFromRealSense(QVector<Vertex> &glPoint, bool &isSuccess, GLuint &textureID);

void loadPointCloudFromRealSense(rs2::points& points, bool &isSuccess);

#endif // RSCAMERAL_H
