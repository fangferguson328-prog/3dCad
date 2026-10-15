#!/usr/bin/env python3
"""Compile actual 3D arc code without Qt or a robot SDK.

The C++ test uses extracted production declarations, function bodies, and path
sampling helpers. Only QVector3D, surface-normal lookup, the point-cloud distance
check, and repainting are replaced. This does not validate Qt ABI, PCA quality,
robot inverse kinematics, controller interpolation, or existing tangent fallbacks.
The actual Euler conversion is also exercised after a non-identity rotation.
Run: python3 diagnostics/test_fixed_arc_orientation.py
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def mask_noncode(source):
    """Preserve offsets while hiding comments and ordinary C++ string literals."""
    pattern = r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    return re.sub(pattern, lambda m: "".join("\n" if c == "\n" else " " for c in m[0]), source)


def extract_braced(source, start_pattern, label):
    masked = mask_noncode(source)
    match = re.search(start_pattern, masked, re.MULTILINE)
    if not match:
        raise RuntimeError("Cannot locate " + label)
    opening = masked.index("{", match.start())
    depth = 0
    for i in range(opening, len(masked)):
        depth += (masked[i] == "{") - (masked[i] == "}")
        if depth == 0:
            return source[match.start():i + 1], source[:match.start()].count("\n") + 1
    raise RuntimeError("Unbalanced braces in " + label)


def definition(source, name):
    return extract_braced(source, r"^[^\n;{}]*\b" + re.escape(name) + r"\s*\(", name)


def declaration(header, name):
    match = re.search(r"\b(?:void|bool)\s+" + re.escape(name) + r"\s*\([^;{}]*\)\s*;", header)
    if not match:
        raise RuntimeError("Cannot locate declaration of " + name)
    return match[0]


SHIM = r'''
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
class QVector3D {
    float a_, b_, c_;
public:
    QVector3D(float a=0, float b=0, float c=0): a_(a), b_(b), c_(c) {}
    float x() const { return a_; }
    float y() const { return b_; }
    float z() const { return c_; }
    float length() const { return std::sqrt(dotProduct(*this,*this)); }
    QVector3D normalized() const { float n=length(); return n>0 ? *this/n : QVector3D(); }
    void normalize() { *this=normalized(); }
    QVector3D operator+(const QVector3D& v) const { return {a_+v.a_,b_+v.b_,c_+v.c_}; }
    QVector3D operator-(const QVector3D& v) const { return {a_-v.a_,b_-v.b_,c_-v.c_}; }
    QVector3D operator-() const { return {-a_,-b_,-c_}; }
    QVector3D operator*(float s) const { return {a_*s,b_*s,c_*s}; }
    QVector3D operator/(float s) const { return {a_/s,b_/s,c_/s}; }
    friend QVector3D operator*(float s,const QVector3D& v) { return v*s; }
    static float dotProduct(const QVector3D& a,const QVector3D& b) {
        return a.a_*b.a_+a.b_*b.b_+a.c_*b.c_;
    }
    static QVector3D crossProduct(const QVector3D& a,const QVector3D& b) {
        return {a.b_*b.c_-a.c_*b.b_,a.c_*b.a_-a.a_*b.c_,a.a_*b.b_-a.b_*b.a_};
    }
};
enum GraphicType { NUL=-1, LINE=0, ARC=1, CURVE=2 };
struct Point { double x,y,z; };
'''

CLASS_PREFIX = r'''
class OpenGLWidget {
public:
'''

CLASS_SUFFIX = r'''
    std::vector<WeldTrack> weldTrackPoints;
    int m_draw3DMode=D3_NONE;
    bool m_last3DFailureReported=false, m_draw3DHoverValid=false;
    std::vector<QVector3D> m_draw3DCtrlPoints;
    std::function<QVector3D(const QVector3D&)> normalProvider;
    bool acceptPath=true;
    int pathChecks=0, repaints=0;
    QVector3D surfaceNormalAt(const QVector3D& p) {
        return normalProvider ? normalProvider(p) : QVector3D(0,0,-1);
    }
    bool validateSurfacePath(const std::vector<QVector3D>&) { ++pathChecks; return acceptPath; }
    void showSurfaceTrackError() { m_last3DFailureReported=true; }
    void update() { ++repaints; }
};
'''

TESTS = r'''
void require(bool condition,const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
bool near(const QVector3D& a,const QVector3D& b,float eps=1e-5f) {
    return (a-b).length()<eps;
}
QVector3D vec(const Point& p) { return QVector3D(p.x,p.y,p.z); }
void checkFrame(const QVector3D& z,const QVector3D& x) {
    require(std::isfinite(x.x()) && std::isfinite(x.y()) && std::isfinite(x.z()) &&
            std::isfinite(z.x()) && std::isfinite(z.y()) && std::isfinite(z.z()),"non-finite frame");
    require(std::fabs(z.length()-1)<1e-5 && std::fabs(x.length()-1)<1e-5,"non-unit frame");
    require(std::fabs(QVector3D::dotProduct(z,x))<1e-5,"non-orthogonal frame");
}
// A proper, non-identity rotation: Rx(30 deg), then Rz(40 deg).
QVector3D rotated(const QVector3D& v) {
    const float a=M_PI/6, b=2*M_PI/9;
    float x=v.x(),y=std::cos(a)*v.y()-std::sin(a)*v.z(),z=std::sin(a)*v.y()+std::cos(a)*v.z();
    return {std::cos(b)*x-std::sin(b)*y,std::sin(b)*x+std::cos(b)*y,z};
}
void fixedCase(const std::vector<QVector3D>& p,const std::vector<QVector3D>& normals) {
    OpenGLWidget widget;
    widget.m_draw3DMode=OpenGLWidget::D3_ARC;
    widget.m_draw3DCtrlPoints=p;
    widget.normalProvider=[&](const QVector3D& q) {
        size_t best=0;
        for (size_t i=1;i<p.size();++i)
            if ((q-p[i]).length()<(q-p[best]).length()) best=i;
        return normals[best].normalized();
    };
    require(widget.finish3DShape(),"D3_ARC rejected valid input");
    require(widget.pathChecks==1,"D3_ARC must retain path validation");
    require(widget.weldTrackPoints.size()==1,"D3_ARC must append exactly one track");
    const auto& track=widget.weldTrackPoints.front();
    require(track.type==ARC,"track type changed");
    require(near(vec(track.begin),p[0]) && near(vec(track.pocess),p[1]) && near(vec(track.end),p[2]),"arc geometry changed");
    require(track.ArcPoints.size()==3 && track.arcPointsDirecton.size()==3 && track.arcPointsXDirecton.size()==3,"arc array sizes changed");
    const QVector3D n0=normals[0].normalized(), chord=p[1]-p[0];
    const QVector3D originalX=(chord-QVector3D::dotProduct(chord,n0)*n0).normalized();
    require(near(track.direction,-n0) && near(track.xDirection,originalX),"original first-point frame changed");
    QVector3D expectedAngles;
    calculateOrientationZX(rotated(track.direction),rotated(track.xDirection),expectedAngles);
    for (size_t i=0;i<3;++i) {
        require(near(track.ArcPoints[i],p[i]),"ArcPoints order or position changed");
        require(near(track.arcPointsDirecton[i],track.direction),"Z differs from first-point Z at index "+std::to_string(i));
        require(near(track.arcPointsXDirecton[i],track.xDirection),"X differs from first-point X at index "+std::to_string(i));
        checkFrame(track.arcPointsDirecton[i],track.arcPointsXDirecton[i]);
        require(near(rotated(track.arcPointsDirecton[i]),rotated(track.direction)) &&
                near(rotated(track.arcPointsXDirecton[i]),rotated(track.xDirection)),"rotation did not preserve identical frames");
        checkFrame(rotated(track.arcPointsDirecton[i]),rotated(track.arcPointsXDirecton[i]));
        QVector3D angles;
        calculateOrientationZX(rotated(track.arcPointsDirecton[i]),rotated(track.arcPointsXDirecton[i]),angles);
        require(std::isfinite(angles.x()) && std::isfinite(angles.y()) && std::isfinite(angles.z()),"non-finite Euler angles");
        require(near(angles,expectedAngles),"actual Euler conversion produced different point orientations");
    }
    require(!track.continueFlag && !track.continueEndFlag && !track.baihanFlag,"single-arc flags changed");
    require(widget.m_draw3DCtrlPoints.empty() && !widget.m_draw3DHoverValid,"successful completion did not clear preview");
}
void ellipseCase() {
    OpenGLWidget widget;
    widget.m_draw3DMode=OpenGLWidget::D3_ELLIPSE;
    widget.m_draw3DCtrlPoints={{0,0,1},{.12f,0,1},{0,.07f,1}};
    require(widget.finish3DShape(),"ellipse failed");
    require(widget.weldTrackPoints.size()==8,"ellipse segment count changed");
    require(widget.pathChecks==2,"ellipse must check the ideal and combined actual paths");
    for (size_t i=0;i<8;++i) {
        const auto& t=widget.weldTrackPoints[i];
        require(t.type==ARC && t.ArcPoints.size()==3 && t.arcPointsXDirecton.size()==3 && t.arcPointsDirecton.size()==3,"ellipse arc shape changed");
        require(near(vec(t.end),vec(widget.weldTrackPoints[(i+1)%8].begin)),"ellipse no longer closes or connects");
        require(t.continueFlag==(i<7) && t.continueEndFlag==(i==7) && !t.baihanFlag,"ellipse continuity flags changed");
        require(!near(t.arcPointsXDirecton[0],t.arcPointsXDirecton[1]),"ellipse unexpectedly uses fixed orientation");
        for (int j=0;j<3;++j) {
            const float angle=(i+j*.5f)*M_PI/4;
            require(near(t.ArcPoints[j],QVector3D(.12f*std::cos(angle),-.07f*std::sin(angle),1)),"ellipse point positions changed");
            require(near(t.arcPointsDirecton[j],QVector3D(0,0,1)),"ellipse Z direction changed");
        }
    }
    // Numeric snapshot of legacy first-segment chord directions for this ellipse.
    const QVector3D oldX[]={{-.322744763f,-.946486037f,0},{-.578965498f,-.815352042f,0},{-.753315392f,-.657659425f,0}};
    for (int i=0;i<3;++i)
        require(near(widget.weldTrackPoints[0].arcPointsXDirecton[i],oldX[i]),"legacy ellipse first-segment orientation changed");
}
void defaultCallCase() {
    OpenGLWidget widget;
    // Deliberately omit the new flag; the declaration comes from the real header.
    widget.appendArcTrack({.1f,0,1},{0,.1f,1},{-.1f,0,1},true,false);
    const auto& t=widget.weldTrackPoints.at(0);
    require(!near(t.arcPointsXDirecton[0],t.arcPointsXDirecton[1]),"omitted flag changed legacy default behavior");
    require(t.continueFlag && !t.continueEndFlag,"default call flags changed");
}
void rejectedPathCase() {
    OpenGLWidget widget;
    widget.m_draw3DMode=OpenGLWidget::D3_ARC;
    widget.m_draw3DCtrlPoints={{.1f,0,1},{0,.1f,1},{-.1f,0,1}};
    widget.acceptPath=false;
    require(!widget.finish3DShape(),"invalid surface path was accepted");
    require(widget.weldTrackPoints.empty(),"rejected path appended a track");
    require(widget.m_last3DFailureReported && widget.pathChecks==1,"rejection reporting changed");
    require(widget.m_draw3DCtrlPoints.size()==3,"rejection discarded control points");
}
int main() {
    int failed=0, passed=0;
    auto run=[&](const char* name,const std::function<void()>& test) {
        try { test(); ++passed; std::cout<<"PASS "<<name<<"\n"; }
        catch (const std::exception& e) { ++failed; std::cout<<"FAIL "<<name<<": "<<e.what()<<"\n"; }
    };
    const std::vector<QVector3D> p={{.1f,0,1},{0,.1f,1},{-.1f,0,1}};
    const std::vector<QVector3D> flat(3,QVector3D(0,0,-1));
    run("planar arc retains first-point frame",[&]{ fixedCase(p,flat); });
    run("varying normals still produce one frame",[&]{ fixedCase(p,{{0,0,-1},{.2f,0,-1},{0,.3f,-1}}); });
    run("tilted plane and rotated base frame",[&]{
        std::vector<QVector3D> tilted;
        for (const auto& point:p) tilted.push_back(rotated(point));
        fixedCase(tilted,std::vector<QVector3D>(3,rotated({0,0,-1})));
    });
    run("reverse arc preserves its own first-point frame",[&]{ fixedCase({p[2],p[1],p[0]},flat); });
    run("ellipse keeps eight legacy variable-pose arcs",ellipseCase);
    run("omitted flag preserves legacy default",defaultCallCase);
    run("surface-path rejection appends nothing",rejectedPathCase);
    std::cout<<passed<<" passed, "<<failed<<" failed\n";
    return failed ? 1 : 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--compiler", default="clang++")
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    if not compiler:
        parser.error("C++ compiler not found: " + args.compiler)
    root = args.source_root.resolve()
    source_path = root / "openglwidget.cpp"
    source = source_path.read_text(encoding="utf-8-sig")
    header = (root / "openglwidget.h").read_text(encoding="utf-8-sig")
    common_path = root / "common.cpp"
    common = common_path.read_text(encoding="utf-8-sig")
    methods = ("appendLineTrack", "appendArcTrack", "buildEllipseTracks", "finish3DShape")
    track, _ = extract_braced(header, r"\bstruct\s+WeldTrack\b", "WeldTrack")
    mode, _ = extract_braced(header, r"\benum\s+Draw3DMode\b", "Draw3DMode")
    declarations = "\n".join(declaration(header, name) for name in methods)
    chunks = [SHIM, CLASS_PREFIX, track + ";", mode + ";", declarations, CLASS_SUFFIX]
    helpers, line = extract_braced(source, r"^namespace\s*\{", "path sampling namespace")
    chunks.extend((f"#line {line} {json.dumps(str(source_path))}\n", helpers))
    for name in ("catmullPoint", "catmullRom3D", "calculateOrientationZX"):
        body, line = definition(common, name)
        chunks.extend((f"#line {line} {json.dumps(str(common_path))}\n", body))
    for name in methods:
        body, line = definition(source, "OpenGLWidget::" + name)
        chunks.extend((f"#line {line} {json.dumps(str(source_path))}\n", body))
    chunks.extend(('#line 1 "fixed_arc_regression_harness"\n', TESTS))
    print("Compiling extracted production geometry and route functions; no Qt/SDK/devices.", flush=True)
    print("Boundary: supplied normals and surface-check decisions; actual Euler conversion; no PCA, Qt ABI, IK or controller validation.", flush=True)
    print("Existing zero/parallel-tangent fallback is outside this change and is not certified here.", flush=True)
    with tempfile.TemporaryDirectory(prefix="3dcad-fixed-arc-test-") as work:
        cpp = Path(work) / "fixed_arc_test.cpp"
        executable = Path(work) / "fixed_arc_test"
        cpp.write_text("\n".join(chunks), encoding="utf-8")
        subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", str(cpp), "-o", str(executable)], check=True)
        return subprocess.run([str(executable)]).returncode


if __name__ == "__main__":
    raise SystemExit(main())
