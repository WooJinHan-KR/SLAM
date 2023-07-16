//
// Created by hanwoojin on 23. 7. 16.
//
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>
#include <string>
#include <iostream>

//using namespace std;
//using namespace Eigen;

// path to trajectory file
std::string trajectory_file = "/home/hanwoojin/hello_cmake/examples/trajectory.txt";

void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>);

int main(int argc, char **argv) {

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    //3차원 공간에서 위치와 방향을 나타내는 변환 매트릭스를 표현하는 Isometry3d와 Eigen 라이브러리의 메모리 정렬을 처리하기 위해
    //이 두가지를 가지고 있는 이차원 벡터 poses 선언
    std::ifstream fin(trajectory_file);//txt 파일을 읽어옴
    if (!fin) {//파일을 여는 객체 fin이 열리지 않으면 오류 메세지 발생
        std::cout << "cannot find trajectory file at " << trajectory_file << std::endl;
        return 1;//종료
    }

    while (!fin.eof()) {//파일의 끝까지 읽으면서
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        //파일에서 시간(time), 위치, 방향 값을 읽어온다
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        //qw, qx, qy, qz로 부터 Isometry3d 타입의 Twr 객체 생성(3D 공간에서 회전을 나타내는 객체
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        //Vector3d로 tx,ty,tz 값을 가진 이동 벡터 생성해서 Twr 이동 부분을 tx, ty, tz 값으로 설정
        poses.push_back(Twr);
        //Twr 객체를 poses 벡터에 추가하면서 반복될때마다 읽어온 위치와 방향을 가진 Isometry3d 객체들이 저장
    }
    std::cout << "read total " << poses.size() << " pose entries" << std::endl;
    //읽어온 위치, 방향 값을 사용해서 회전과 이동 변환을 나타내는 Isometry3d 객체를 생성해서
    //반복하며 poses 벡터에 추가(위치와 방향 정보를 가지고 있음)

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            Eigen::Vector3d Ow = poses[i].translation();
            Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}