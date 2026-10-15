#!/usr/bin/env python3
"""Compile production surface-path validation with synthetic clouds, without Qt.

The harness extracts the actual sampling namespace, validation methods,
PointCloudAdapter, KD-tree build, and common.cpp fitLocalPlane. It uses the real
nanoflann.hpp. QVector3D/QVector/Vertex are lightweight boundary substitutes;
cv::eigen is replaced by a separate 3x3 symmetric Jacobi eigensolver because this
environment cannot link the application's Windows OpenCV libraries. This checks
geometry and acceptance decisions, not Qt/OpenCV ABI, camera acquisition, robot
motion, or the distribution of noise in an actual captured point cloud.
The actual buildEllipseTracks assembly also runs; only its appendArcTrack sink
and error dialog are replaced, to count appended segments and reported failures.

Run: python3 diagnostics/test_surface_path.py
Use --baseline to run only the two regressions while documenting old behavior.
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from test_fixed_arc_orientation import SHIM, definition, extract_braced, mask_noncode

BOUNDARIES = r'''
#include <array>
#include <numeric>
#include "nanoflann.hpp"
template<class T> class QVector: public std::vector<T> {
public:
    using std::vector<T>::vector;
    bool isEmpty() const { return this->empty(); }
};
struct Vertex { QVector3D position; };
// Only the OpenCV matrix/eigendecomposition boundary is substituted. The
// production fitLocalPlane covariance construction and frame selection run below.
namespace cv {
struct Mat {
    std::array<double,9> data{{0,0,0,0,0,0,0,0,0}};
    template<class T> T at(int row,int col) const { return T(data[row*3+col]); }
};
struct Initializer {
    Mat matrix;
    int next=1;
    explicit Initializer(double first) { matrix.data[0]=first; }
    Initializer& operator,(double value) { matrix.data[next++]=value; return *this; }
    operator Mat() const { return matrix; }
};
template<class T> struct Mat_ {
    Mat_(int,int) {}
    Initializer operator<<(double first) { return Initializer(first); }
};
bool eigen(const Mat& input,Mat& values,Mat& vectors) {
    double a[3][3],v[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) a[i][j]=input.data[i*3+j];
    for (int iteration=0;iteration<60;++iteration) {
        int p=0,q=1;
        if (std::fabs(a[0][2])>std::fabs(a[p][q])) { p=0;q=2; }
        if (std::fabs(a[1][2])>std::fabs(a[p][q])) { p=1;q=2; }
        if (std::fabs(a[p][q])<1e-16) break;
        const double angle=0.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]);
        const double c=std::cos(angle),s=std::sin(angle);
        const double app=a[p][p],aqq=a[q][q],apq=a[p][q];
        a[p][p]=c*c*app-2*s*c*apq+s*s*aqq;
        a[q][q]=s*s*app+2*s*c*apq+c*c*aqq;
        a[p][q]=a[q][p]=0;
        for (int k=0;k<3;++k) {
            if (k!=p && k!=q) {
                double akp=a[k][p],akq=a[k][q];
                a[k][p]=a[p][k]=c*akp-s*akq;
                a[k][q]=a[q][k]=s*akp+c*akq;
            }
            double vkp=v[k][p],vkq=v[k][q];
            v[k][p]=c*vkp-s*vkq;
            v[k][q]=s*vkp+c*vkq;
        }
    }
    std::array<int,3> order{{0,1,2}};
    std::sort(order.begin(),order.end(),[&](int i,int j){ return a[i][i]>a[j][j]; });
    for (int i=0;i<3;++i) {
        values.data[i*3]=a[order[i]][order[i]];
        for (int j=0;j<3;++j) vectors.data[i*3+j]=v[j][order[i]];
    }
    return true;
}
}
'''

TESTS = r'''
void require(bool condition,const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void addPoint(OpenGLWidget& widget,float x,float y,float z) {
    widget.points.push_back(Vertex{QVector3D(x,y,z)});
}
void plane(OpenGLWidget& widget,float step,float halfWidth,
           const std::function<float(int,int)>& height) {
    const int n=(int)std::ceil(halfWidth/step);
    for (int i=-n;i<=n;++i) for (int j=-n;j<=n;++j)
        addPoint(widget,i*step,j*step,1+height(i,j));
}
void expect(OpenGLWidget& widget,const std::vector<QVector3D>& path,bool accepted) {
    bool actual=widget.validateSurfacePath(path);
    require(actual==accepted,std::string("expected ")+(accepted?"accept":"reject")+
            ", got "+(actual?"accept":"reject"));
}
void sparseFlat() {
    OpenGLWidget widget;
    plane(widget,.012f,.24f,[](int,int){return 0.f;});
    // Every sample lies 6 mm from the nearest grid row despite being on its plane.
    expect(widget,sampleLinePath({-.15f,.006f,1},{.15f,.006f,1}),true);
}
void shortPit() {
    OpenGLWidget widget;
    plane(widget,.002f,.16f,[](int i,int j){
        return (std::abs(i)<=3 && std::abs(j)<=3)?-.008f:0.f;
    });
    // A local 8 mm-deep scan defect must not invalidate a 240 mm surface stroke.
    expect(widget,sampleLinePath({-.12f,0,1},{.12f,0,1}),true);
}
void smoothFlat() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return 0.f;});
    expect(widget,sampleLinePath({-.15f,.001f,1},{.15f,.001f,1}),true);
}
void roughFlat() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int i,int j){
        // Reproducible full-amplitude noise; balanced around the workpiece plane.
        unsigned hash=uint32_t(i+200)*73856093u ^ uint32_t(j+200)*19349663u;
        return (int(hash%2001)-1000)*.000008f;
    });
    expect(widget,sampleLinePath({-.15f,.001f,1},{.15f,.001f,1}),true);
    // Picked endpoints themselves can be above the average surface.
    expect(widget,sampleLinePath({-.15f,.001f,1.008f},{.15f,.001f,1.008f}),true);
}
void sustainedDeviation() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return 0.f;});
    expect(widget,samplePolylinePath({{-.15f,0,1},{-.05f,0,1.020f},
                                    {.05f,0,1.020f},{.15f,0,1}}),false);
    // Clean data must keep a tight tolerance; the noise allowance is not global.
    expect(widget,sampleLinePath({-.15f,0,1.008f},{.15f,0,1.008f}),false);
}
void noSurfaceSupport() {
    OpenGLWidget widget;
    for (int i=-70;i<=70;++i) for (int j=-30;j<=30;++j)
        if (std::abs(i)>17) addPoint(widget,i*.003f,j*.003f,1);
    expect(widget,sampleLinePath({-.15f,0,1},{.15f,0,1}),false);
}
void tiltedRoughSurface() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int i,int j){
        unsigned hash=uint32_t(i+200)*73856093u ^ uint32_t(j+200)*19349663u;
        return (int(hash%2001)-1000)*.000008f;
    });
    const QVector3D u=QVector3D(1,2,0).normalized();
    const QVector3D v=QVector3D(-2,1,3).normalized();
    const QVector3D n=QVector3D::crossProduct(u,v).normalized();
    auto transform=[&](const QVector3D& p) {
        return QVector3D(.3f,-.2f,.7f)+p.x()*u+p.y()*v+(p.z()-1)*n;
    };
    for (auto& vertex:widget.points) vertex.position=transform(vertex.position);
    expect(widget,sampleLinePath(transform({-.15f,.001f,1}),transform({.15f,.001f,1})),true);
}
void parallelSheets() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return -.015f;});
    plane(widget,.003f,.2f,[](int,int){return .015f;});
    expect(widget,sampleLinePath({-.15f,0,1},{.15f,0,1}),false);
}
void cornerCut() {
    OpenGLWidget widget;
    for (int i=0;i<=60;++i) for (int j=-20;j<=20;++j) {
        addPoint(widget,i*.003f,j*.003f,1);
        addPoint(widget,0,j*.003f,1+i*.003f);
    }
    expect(widget,sampleLinePath({.12f,0,1},{0,0,1.12f}),false);
    expect(widget,samplePolylinePath({{.12f,0,1},{0,0,1},{0,0,1.12f}}),true);
}
void cylinder(bool chord,float radius=.1f) {
    OpenGLWidget widget;
    for (int i=0;i<360;++i) for (int j=-20;j<=20;++j) {
        float angle=i*M_PI/180;
        addPoint(widget,radius*std::cos(angle),j*.003f,1+radius*std::sin(angle));
    }
    const QVector3D a(radius,0,1),b(0,0,1+radius),c(-radius,0,1);
    expect(widget,chord?sampleLinePath(a,c):sampleArcPath(a,b,c),!chord);
}
void emptyCloud() {
    OpenGLWidget widget;
    expect(widget,sampleLinePath({0,0,1},{.1f,0,1}),false);
}
void isolatedPathNoise() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return 0.f;});
    auto path=sampleLinePath({-.15f,0,1},{.15f,0,1});
    path[path.size()/2]=path[path.size()/2]+QVector3D(0,0,.008f);
    expect(widget,path,true);
}
void longPathContinuousDefect() {
    OpenGLWidget widget;
    for (int i=-410;i<=410;++i) for (int j=-8;j<=8;++j)
        addPoint(widget,i*.005f,j*.005f,1);
    expect(widget,samplePolylinePath({{-2,0,1},{-.04f,0,1},
           {-.03f,0,1.008f},{.03f,0,1.008f},{.04f,0,1},{2,0,1}}),false);
}
void closedSeamDefect() {
    OpenGLWidget widget;
    plane(widget,.003f,.14f,[](int,int){return 0.f;});
    std::vector<QVector3D> path;
    for (int i=0;i<=200;++i) {
        const float a=i*2*M_PI/200;
        path.push_back({.08f*std::cos(a),.08f*std::sin(a),
                        1+((i<4 || i>196)?.008f:0.f)});
    }
    path.back()=path.front();
    expect(widget,path,false);
    // Move the same defect away from the storage seam: the decision must agree.
    path.pop_back();
    std::rotate(path.begin(),path.begin()+100,path.end());
    path.push_back(path.front());
    expect(widget,path,false);
}
void nonuniformResampling() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return 0.f;});
    for (bool narrow: {false,true}) {
        const float width=narrow?.003f:.04f;
        const auto path=samplePolylinePath({{-.15f,0,1},{-width,0,1},
                    {0,0,1.008f},{width,0,1},{.15f,0,1}});
        std::vector<QVector3D> dense;
        for (size_t i=0;i+1<path.size();++i) {
            const int subdivisions=std::fabs(path[i].x())<.05f?9:1;
            for (int j=0;j<subdivisions;++j) {
                float t=float(j)/subdivisions;
                dense.push_back((1-t)*path[i]+t*path[i+1]);
            }
        }
        dense.push_back(path.back());
        expect(widget,path,narrow);
        expect(widget,dense,narrow);
    }
}
void shortEntirelyOffSurface() {
    OpenGLWidget widget;
    plane(widget,.003f,.12f,[](int,int){return 0.f;});
    expect(widget,sampleLinePath({0,0,1.008f},{.004f,0,1.008f}),false);
}
void nonfiniteInput() {
    OpenGLWidget widget;
    plane(widget,.003f,.12f,[](int,int){return 0.f;});
    const float nan=std::numeric_limits<float>::quiet_NaN();
    const float inf=std::numeric_limits<float>::infinity();
    expect(widget,{{0,0,1},{nan,0,1},{.02f,0,1}},false);
    expect(widget,{{0,0,1},{0,inf,1}},false);
}
void boundaryExtrapolation() {
    OpenGLWidget widget;
    plane(widget,.012f,.12f,[](int,int){return 0.f;});
    // 18 mm outside the cloud, still within the 20 mm nearest-point reach.
    expect(widget,sampleLinePath({-.09f,.138f,1},{.09f,.138f,1}),false);
}
void isolatedCloudOutliers() {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int,int){return 0.f;});
    // A sparse one-dimensional chain of outliers is not a second physical face.
    for (int i=-15;i<=15;++i) addPoint(widget,i*.008f,0,1.018f);
    expect(widget,sampleLinePath({-.12f,0,1.018f},{.12f,0,1.018f}),false);
}
void mediumCloudGap() {
    OpenGLWidget widget;
    for (int i=-60;i<=60;++i) for (int j=-30;j<=30;++j)
        if (std::abs(i)>5) addPoint(widget,i*.003f,j*.003f,1);
    // 33 mm missing stripe cannot be established by interpolating two edges.
    expect(widget,sampleLinePath({-.12f,0,1},{.12f,0,1}),false);
}
void rotatedSparseClouds() {
    for (int iteration=0;iteration<12;++iteration) {
        OpenGLWidget widget;
        plane(widget,.012f,.18f,[](int,int){return 0.f;});
        const QVector3D axis=QVector3D(1+iteration*.13f,-.7f+iteration*.05f,.3f).normalized();
        const float angle=.23f+iteration*.49f;
        auto rotated=[&](const QVector3D& p) {
            const QVector3D q=p-QVector3D(0,0,1);
            return QVector3D(.1f,-.3f,.6f)+std::cos(angle)*q+
                   std::sin(angle)*QVector3D::crossProduct(axis,q)+
                   (1-std::cos(angle))*QVector3D::dotProduct(axis,q)*axis;
        };
        for (auto& vertex:widget.points) vertex.position=rotated(vertex.position);
        expect(widget,sampleLinePath(rotated({-.12f,.006f,1}),rotated({.12f,.006f,1})),true);
        expect(widget,sampleLinePath(rotated({-.12f,.006f,1.018f}),rotated({.12f,.006f,1.018f})),false);
    }
}
void ellipseIntegration(bool reject) {
    OpenGLWidget widget;
    plane(widget,.003f,.2f,[](int i,int j){
        unsigned hash=uint32_t(i+200)*73856093u ^ uint32_t(j+200)*19349663u;
        return (int(hash%2001)-1000)*.000008f;
    });
    float z=reject?1.020f:1;
    widget.m_draw3DCtrlPoints={{0,0,z},{.1f,0,z},{0,.07f,z}};
    require(widget.buildEllipseTracks()!=reject,"ellipse acceptance decision");
    require(widget.appendedArcs==(reject?0:8),"ellipse partial append or missing arc");
    require(widget.m_last3DFailureReported==reject,"ellipse failure reporting");
}
void eigenBoundary() {
    const QVector3D axisA=QVector3D(1,2,0).normalized();
    const QVector3D axisB=QVector3D(-2,1,3).normalized();
    const QVector3D target=QVector3D::crossProduct(axisA,axisB).normalized();
    std::vector<QVector3D> points;
    for (int i=-4;i<=4;++i) for (int j=-2;j<=2;++j)
        points.push_back(QVector3D(.3f,-.1f,1)+axisA*(i*.01f)+axisB*(j*.01f));
    QVector3D center,normal,u,v;
    require(fitLocalPlane(points,center,normal,u,v),"plane fit failed");
    require(std::fabs(QVector3D::dotProduct(target,normal))>.99999f,"Jacobi boundary normal incorrect");
}
int main(int argc,char** argv) {
    int failed=0,passed=0;
    auto run=[&](const char* name,const std::function<void()>& test) {
        try { test(); ++passed; std::cout<<"PASS "<<name<<"\n"; }
        catch (const std::exception& error) { ++failed; std::cout<<"FAIL "<<name<<": "<<error.what()<<"\n"; }
    };
    run("sparse 12 mm flat cloud between grid rows",sparseFlat);
    run("short 8 mm-deep point-cloud pit",shortPit);
    if (argc<2 || std::string(argv[1])!="--baseline") {
        run("Jacobi OpenCV boundary on tilted plane",eigenBoundary);
        run("smooth surface",smoothFlat);
        run("rough surface with +/-8 mm noise and 8 mm picked endpoint height",roughFlat);
        run("tilted rough surface",tiltedRoughSurface);
        run("sustained departures from clean surface at 20 mm and 8 mm",sustainedDeviation);
        run("large region without point-cloud support",noSurfaceSupport);
        run("midplane between two surfaces 30 mm apart",parallelSheets);
        run("90 degree corner: reject diagonal; accept surface polyline",cornerCut);
        run("cylinder surface arc",[]{ cylinder(false); });
        run("cylinder internal chord",[]{ cylinder(true); });
        run("empty point cloud",emptyCloud);
        run("single short 8 mm path noise spike",isolatedPathNoise);
        run("continuous defect on a 4 metre path",longPathContinuousDefect);
        run("closed curve joins unsupported run across storage seam",closedSeamDefect);
        run("nonuniform resampling keeps physical-length decisions",nonuniformResampling);
        run("short path entirely off surface",shortEntirelyOffSurface);
        run("NaN and infinity inputs",nonfiniteInput);
        run("sparse plane cannot extrapolate outside observed boundary",boundaryExtrapolation);
        run("isolated cloud outliers cannot fabricate a surface",isolatedCloudOutliers);
        run("33 mm missing point-cloud stripe",mediumCloudGap);
        run("sparse surface after twelve arbitrary rotations",rotatedSparseClouds);
        run("20 mm radius cylinder surface arc",[]{ cylinder(false,.02f); });
        run("20 mm radius cylinder interior chord",[]{ cylinder(true,.02f); });
        run("10 mm radius cylinder surface arc",[]{ cylinder(false,.01f); });
        run("10 mm radius cylinder interior chord",[]{ cylinder(true,.01f); });
        run("actual ellipse assembly on rough cloud",[]{ ellipseIntegration(false); });
        run("actual ellipse rejection appends no partial track",[]{ ellipseIntegration(true); });
    }
    std::cout<<passed<<" passed, "<<failed<<" failed\n";
    return failed?1:0;
}
'''


def extract_methods(source, header):
    """Follow production widget helper calls so newly introduced helpers run too."""
    masked = mask_noncode(source)
    available = set(re.findall(r"\bOpenGLWidget::(\w+)\s*\(", masked))
    needed = {"validateSurfacePath", "buildKdTree", "buildEllipseTracks"}
    boundaries = {"appendArcTrack", "showSurfaceTrackError"}
    if "nearestSurfaceDistance" in available:
        needed.add("nearestSurfaceDistance")
    bodies = {}
    while needed - bodies.keys():
        name = sorted(needed - bodies.keys())[0]
        body, line = definition(source, "OpenGLWidget::" + name)
        bodies[name] = (body, line)
        called = set(re.findall(r"\b(\w+)\s*\(", mask_noncode(body)))
        needed.update((called & available) - boundaries)
    declarations = []
    for name in sorted(bodies):
        pattern = r"^[ \t]*[^\n;{}]*\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*(?:const\s*)?;"
        match = re.search(pattern, header, re.MULTILINE)
        if not match:
            raise RuntimeError("Cannot find widget declaration: " + name)
        declarations.append(match[0])
    return bodies, declarations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--compiler", default="clang++")
    parser.add_argument("--baseline", action="store_true")
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    if not compiler:
        parser.error("C++ compiler not found: " + args.compiler)
    root = args.source_root.resolve()
    source_path = root / "openglwidget.cpp"
    common_path = root / "common.cpp"
    source = source_path.read_text(encoding="utf-8-sig")
    header = (root / "openglwidget.h").read_text(encoding="utf-8-sig")
    common_header = (root / "common.h").read_text(encoding="utf-8-sig")
    common = common_path.read_text(encoding="utf-8-sig")
    adapter, _ = extract_braced(common_header, r"\bstruct\s+PointCloudAdapter\b", "PointCloudAdapter")
    kd_type = re.search(r"typedef\s+nanoflann::KDTreeSingleIndexAdaptor[\s\S]*?my_kd_tree_t\s*;", common_header)[0]
    bodies, declarations = extract_methods(source, header)
    plane_body, plane_line = definition(common, "fitLocalPlane")
    chunks = [SHIM, BOUNDARIES, adapter + ";", kd_type,
              f"#line {plane_line} {json.dumps(str(common_path))}\n", plane_body,
              "class OpenGLWidget { public:", *declarations,
              "QVector<Vertex> points; PointCloudAdapter* m_cloudAdapter=nullptr; my_kd_tree_t* m_kdTree=nullptr;",
              "std::vector<QVector3D> m_draw3DCtrlPoints; bool m_last3DFailureReported=false; int appendedArcs=0;",
              "void showSurfaceTrackError() { m_last3DFailureReported=true; }",
              "void appendArcTrack(const QVector3D&,const QVector3D&,const QVector3D&,bool,bool,bool=false) { ++appendedArcs; }",
              "~OpenGLWidget() { delete m_kdTree; delete m_cloudAdapter; } };" ]
    helpers, line = extract_braced(source, r"^namespace\s*\{", "sampling and surface helpers")
    chunks.extend((f"#line {line} {json.dumps(str(source_path))}\n", helpers))
    if re.search(r"^namespace\s+SurfacePathValidation\s*\{", source, re.MULTILINE):
        helpers, line = extract_braced(source, r"^namespace\s+SurfacePathValidation\s*\{", "surface validation helpers")
        chunks.extend((f"#line {line} {json.dumps(str(source_path))}\n", helpers))
    for body, line in bodies.values():
        chunks.extend((f"#line {line} {json.dumps(str(source_path))}\n", body))
    chunks.extend(('#line 1 "surface_path_regression_harness"\n', TESTS))
    print("Compiling actual surface validation, sampling, plane fit, and nanoflann; synthetic clouds in metres.", flush=True)
    print("Boundary: Qt containers/vectors substituted; OpenCV eigen uses independent Jacobi, no devices or robot motion.", flush=True)
    print("Ellipse integration uses actual assembly and validation; track append and error UI are counters.", flush=True)
    with tempfile.TemporaryDirectory(prefix="3dcad-surface-path-test-") as work:
        cpp = Path(work) / "surface_path_test.cpp"
        executable = Path(work) / "surface_path_test"
        cpp.write_text("\n".join(chunks), encoding="utf-8")
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-I", str(root), str(cpp), "-o", str(executable)], check=True)
        return subprocess.run([str(executable), *(["--baseline"] if args.baseline else [])]).returncode


if __name__ == "__main__":
    raise SystemExit(main())
