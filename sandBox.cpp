#include "tutorial/sandBox/sandBox.h"
#include <igl/edge_flaps.h>
#include <igl/collapse_edge.h>
#include "Eigen/dense"
#include <functional>

#include <string>
#include <igl/circulation.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/readOBJ.h>
#include <igl/dqs.h>
#include<igl/deform_skeleton.h>
#include <igl/opengl/glfw/Viewer.h>
#include<igl/sort.h>
#include<igl/sortrows.h>

#include <Eigen/Core>
#include <iostream>
#include <set>
#include <Eigen/LU>
#include <math.h>
#include<algorithm>
#include "igl/circulation.h"
#include <stdlib.h> 
//#include <tgmath.h> 




SandBox::SandBox()
{


}

bool SandBox::endsWith(const std::string& str, const std::string& suffix)
{
    return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

void SandBox::SnakeLoader(const std::string& config) {
    // W - weights matrix
    // BE - Edges between joints
    // C2 - joints positions
    // P - parents
    // M - weights per vertex per joint matrix
    // U - new vertices position after skinning
    std::string item_name;
    std::ifstream nameFileout;
    doubleVariable = 0;
    isActive = false;
    AutoMode = false;
    nameFileout.open(config);
    if (!nameFileout.is_open())
    {
        std::cout << "Can't open file " << config << std::endl;
    }
    else
    {
        int num = 0;
        while (nameFileout >> item_name)
        {
            if (endsWith(item_name,"snake2.obj") || endsWith(item_name,"snake3.obj")) {
                std::cout << "openning " << item_name << std::endl;
                load_mesh_from_file(item_name);
                parents.push_back(-1);
                data().show_overlay_depth = false;
                data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
                data().point_size = 10;
                data().line_width = 2;
                //data().set_visible(true, 1);
                //data().MyScale(Eigen::Vector3d(1, 1, SCALE));
                data().V.col(2) = data().V.col(2) * SHAPE_SCALE;
                Eigen::VectorXd Snaketrans;
                Snaketrans = (Eigen::VectorXd::Ones(data().V.rows()) * SHAPE_TRANS);
                data().V.col(2) = (data().V.col(2)) +Snaketrans;
                target = *new Movable();
                Voriginal = data().V;
                updateDirection = false;
                isActive = false;


                
            }



            

        }

        
        Eigen::Vector3d snake_tail = (Eigen::Vector4d(0, 0, (-HALF_SNAKE_LEN *SHAPE_SCALE) + SHAPE_TRANS,1)).head(3);
        Eigen::Vector3d snake_head = (Eigen::Vector4d(0, 0, (HALF_SNAKE_LEN * SHAPE_SCALE) + SHAPE_TRANS,1)).head(3);



        boxes.push_back(new Eigen::AlignedBox<double, 3>( Eigen::Vector3d(1,1,1 ), - Eigen::Vector3d(1, 1, 1)));// head box
        boxes.push_back(new Eigen::AlignedBox<double, 3>(Eigen::Vector3d(SPHERE_RADIUS, SPHERE_RADIUS, SPHERE_RADIUS), -Eigen::Vector3d(SPHERE_RADIUS, (SPHERE_RADIUS), SPHERE_RADIUS)));



        //draw_obb(SphereIdx, *boxes[1], Eigen::RowVector3d(1, 1, 1));
        step_size = (snake_head - snake_tail ) / (NUM_JOINTS - 1);
        velocity = 0.06 * step_size.norm();
        Eigen::Vector3d pos = snake_tail;
        C2 = *new Eigen::MatrixXd(NUM_JOINTS, 3);
        BE = *new Eigen::MatrixXi(NUM_JOINTS-1, 2);


        //add joints
        for (int i = 0; i < NUM_JOINTS; i++) {
           
            
            Joints.emplace_back();

            Joints[i].MyTranslate(pos, true);

            
            //Joints[i].MyScale(Eigen::Vector3d(1, 1, 5));
            C2.row(i) = pos;
            pos = pos + step_size;

        }
 
        C2_tmp = *new Eigen::MatrixXd(C2);
        updateDirection = false;
        FollowMe = false;


        calculateWeights(data_list[0].V);
        //calculateWeightsZeroOrder(data_list[0].V);
   

    }
    for (int i = 0; i < NUM_TARGETS; i++) {

        load_mesh_from_file(TARGET_OBJ_PATH);
        parents.push_back(-1);
        data().show_overlay_depth = false;
        data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
        data().point_size = 10;
        data().line_width = 2;
        //data().set_visible(false,1);
        data().set_visible(false, 1);
        data().set_visible(false, 2);


        data().MyScale(Eigen::Vector3d(SPHERE_SCALE, SPHERE_SCALE, SPHERE_SCALE));
        //data().MyTranslate(targets_positions[i], true);
    }
    nameFileout.close();
    MyTranslate(Eigen::Vector3d(0, 0, -1), true);
    std::cout << "Done to Load snake." << std::endl;
    

}

void SandBox::ResetJoints() {
    Eigen::Vector3d snake_tail = (Eigen::Vector4d(0, 0, (-HALF_SNAKE_LEN * SHAPE_SCALE) + SHAPE_TRANS, 1)).head(3);
    Eigen::Vector3d snake_head = (Eigen::Vector4d(0, 0, (HALF_SNAKE_LEN * SHAPE_SCALE) + SHAPE_TRANS, 1)).head(3);
    Eigen::Vector3d pos = snake_tail;
    for (int i = 0; i < NUM_JOINTS; i++) {
        Joints[i].resetTranslation();
        Joints[i].MyTranslate(pos, true);
        pos = pos + step_size;

    }


}

int SandBox::calculateGrid(Eigen::Vector3d d1, Eigen::Vector3d d2,Eigen::Vector3d corner,
    double size) 
{
    targets_positions.clear();

    double step;
    int ElementsInRow = sqrt(cur_num_tragets);
    //int ElementsInRow = cur_num_tragets/2;
    step = size / ElementsInRow;

    for (int i = 0; i < ElementsInRow; i++) {
        for (int j = 0; j < ElementsInRow; j++) {
            targets_positions.push_back(corner + d1*i*step + d2*j*step);
        }
    }

    return cur_num_tragets;


}

int SandBox::calculateGrid(double xCorner, double yCorner, double zCorner, double xDim, double yDim,
    double zDim, double xSize, double ySize, double zSize) {
    double xStep, yStep, zStep, x, z, y;
    if (xDim == 1) {
        xStep = 0;
    }
    else {
        xStep = xSize / (xDim - 1);
    }
    if (yDim == 1) {
        yStep = 0;
    }
    else {
        yStep = ySize / (yDim - 1);
    }
    if (zDim == 1) {
        zStep = 0;
    }
    else {
        zStep = zSize / (zDim - 1);
    }
    
    for (int i = 0; i < xDim; i++) {
        x = xCorner + i * xStep;
        for (int j = 0; j < yDim; j++) {
            y = yCorner + j * yStep;
            for (int k = 0; k < zDim; k++) {
                z = zCorner + k * zStep;
                targets_positions.push_back(Eigen::Vector3d(x, y, z));
            }
        }
    }

    return xDim * yDim * zDim;

}

void SandBox::MoveTargets(int numTargets)
{


    for (int i = 0; i < numTargets; i++) {
        Eigen::Vector3d curPos = (data_list[i + 1].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
        data_list[i+1].MyTranslate(targets_positions[i] - curPos, true);
        targets_to_show.push_back(i + 1);
        data_list[i+1].set_visible(true, 1);
        data_list[i+1].set_visible(true, 2);
        std::cout << "target num " << i+1 << " position =  " << targets_positions[i].transpose() << std::endl;

    }


}

void SandBox::StartGame(Renderer& renderer) {
    render = &renderer;
    for (int i = 0; i < NUM_TARGETS; i++) {
        data_list[i + 1].set_visible(false, 1);
        data_list[i + 1].set_visible(false, 2);
    }
    render->SetCameraEye(Eigen::Vector3f(0, 10, 0));
    render->core().camera_up = Eigen::Vector3f(0, 0, 1);
    Points = 0;
    level = 0;
    sqrt_targets = 2;
    cur_num_tragets = sqrt_targets* sqrt_targets;
    center = render->core().camera_center;
    levelUp = false;
    double xCorner, yCorner, zCorner, xDim, yDim, zDim, xSize, ySize, zSize;
    xCorner = -10;
    yCorner = 0;
    zCorner = 10;
    xDim = 2;
    yDim = 1;
    zDim = 2;
    xSize = 10;
    ySize = 0;
    zSize = 10;
   // int numTargets = calculateGrid(xCorner, yCorner, zCorner, xDim, yDim,
        //zDim, xSize, ySize, zSize);
    int size = sqrt_targets*2;
    int numTargets = calculateGrid(Eigen::Vector3d(0,0,1), Eigen::Vector3d(1,0,0), Eigen::Vector3d(1,0,-6),size);
    MoveTargets(numTargets);

}

Eigen::MatrixXd SandBox::SwapCols(Eigen::MatrixXd& M) {
    Eigen::MatrixXd res = *new Eigen::MatrixXd(M);
    Eigen::VectorXd tmp;
    tmp = res.col(0);
    res.col(0) = res.col(2);
    res.col(2) = tmp;
    return res;
}

void SandBox::calculateWeights(Eigen::MatrixXd& V) {
    Eigen::MatrixXd sorted;
    Eigen::VectorXi I;
    Eigen::MatrixXd unsorted = SwapCols(V);
    igl::sortrows(unsorted, true, sorted, I);
    sorted = SwapCols(sorted);
    // M - weights per vertex per joint matrix
    M = *new Eigen::MatrixXd(V.rows(), NUM_JOINTS);
    Eigen::MatrixXd C = C2;
    //C.col(2) /= (SCALE);
    int i = 0;
    int j = 2;
    



     while ( j < NUM_JOINTS) {
         //std::cout << "sorted: " << sorted(i, 0) << " " << sorted(i, 1) << " " << sorted(i, 2) << std::endl;
         //std::cout << "C: " << C(j-2, 0) << " " << C(j-2, 1) << " " << C(j-2, 2) << std::endl;
         //std::cout << "j: " << j << std::endl;
         double x = sorted.row(i)[2];
         double x_2 = C.row(j)[2];
         double x_1 = C.row(j - 1)[2];
         double x_0 = C.row(j - 2)[2];
          double w_2, w_1, w_0;
           w_2 = 0;
           w_1 = 0;
           w_0 = 0;
           if (x == x_2) {
                w_2 = 1;

            }
            else if (x == x_1) {
                w_1 = 1;
            }
            else if (x == x_0) {
                w_0 = 1;
            }
            else {
                w_2 = ((x - x_0) / (x_2 - x_0)) * ((x - x_1) / (x_2 - x_1));
                w_1 = ((x - x_0) / (x_1 - x_0)) * ((x - x_2) / (x_1 - x_2));
                w_0 = ((x - x_2) / (x_0 - x_2)) * ((x - x_1) / (x_0 - x_1));

            }
           double s = w_0 + w_1 + w_2;
           w_0 = w_0 / s;
           w_1 = w_1 / s;
           w_2 = w_2 / s;
            Eigen::VectorXd res = Eigen::VectorXd::Zero(NUM_JOINTS);
            res[j] = w_2;
            res[j - 1] = w_1;
            res[j - 2] = w_0;
            M.row(I[i]) = res;
            //x = sorted.row(++i)[2];
            i++;
            if (i == V.rows()) {
                break;
            } 
            assert(x_2 > x_1);
            if ((j < NUM_JOINTS - 1) && (x > (x_2 + x_1) / 2)) {
                j++;
            }

            


        }

    
    std::cout << i << std::endl;

    std::cout << "Done calculating weights." << std::endl;


}

Eigen::Matrix3d* SandBox::calcParentsRot(int idx) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    for (int i = idx; P[i] >= 0; i = P[i]) {
        rot = Joints[i].GetRotation() * rot;
    }

    return new Eigen::Matrix3d(rot);

}

void SandBox::calculateWeightsZeroOrder(Eigen::MatrixXd& V) {

    Eigen::MatrixXd sorted;
    Eigen::VectorXi I;
    Eigen::MatrixXd unsorted = SwapCols(V);
    igl::sortrows(unsorted, true, sorted, I);
    sorted = SwapCols(sorted);
    // M - weights per vertex per joint matrix
    M = *new Eigen::MatrixXd(V.rows(), NUM_JOINTS);
    Eigen::MatrixXd C = C2;
    C.col(2) /= 5;
    int i = 0;
    int j = 1;
    Eigen::VectorXd curV, joint0, joint1;

    while(j < (NUM_JOINTS)) 
    {
        std::cout << "sorted: " << sorted(i, 0) << " " << sorted(i, 1) << " " << sorted(i, 2) << std::endl;
        
        
        Eigen::VectorXd res = Eigen::VectorXd::Zero(NUM_JOINTS);
        curV = sorted.row(i);
        joint0 = C.row(j-1);
        joint1 = C.row(j);
        double d0, d1;
        d0 = (joint0 - curV).norm();
        d1 = (joint1 - curV).norm();
        if (d0 <= d1) {
            res[j-1] = 1;
            std::cout << "C: " << C(j-1, 0) << " " << C(j-1, 1) << " " << C(j-1, 2) << std::endl;
        

            
        }
        else {
            res[j] = 1;
            std::cout << "C: " << C(j, 0) << " " << C(j, 1) << " " << C(j, 2) << std::endl;
            if (j < NUM_JOINTS - 1 || i == sorted.rows() - 1) {
                j++;
            }
            
            

        }
        assert(sorted.row(i) == V.row(I[i]));
        M.row(I[i]) = res;
        
        i++;

   

    }
    std::cout << "i vs V.rows()  " << i << " " << V.rows() << std::endl;
    std::cout << "Done calculating weights zero order." << std::endl;
}


/*Eigen::Vector3d* SandBox::getJointPos(int idx) {
    Eigen::Vector3d O = (Joints[TAIL_IDX].MakeTransd() *  Eigen::Vector4d(0, 0, 0, 1)).head(3); // parents of all position.
  

    //std::cout << Joints[NUM_JOINTS - 1].MakeTransd() << '\n' << Joints[NUM_JOINTS - 1].MakeTransScaled() << std::endl;
    C2_tmp.row(TAIL_IDX) = O;
    int step = (TAIL_IDX - HEAD_IDX < 0 ? 1 : -1);
    for (int i = TAIL_IDX + step; i != HEAD_IDX + step; i += step) {
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        for (int j = i; P[j] >= 0; j = P[j]) {
            rot =  Joints[P[j]].GetRotation()*rot;
        }
        
        C2_tmp.row(i) = (Eigen::Vector3d(C2_tmp.row(i-step)) + Eigen::Vector3d(rot * (step_size)));
    }
    return new Eigen::Vector3d(C2_tmp.row(idx));



}*/

Eigen::Vector3d* SandBox::getJointPos(int idx) {
    //std::cout << "get Joint Pos: " << idx << "  " << Joints[idx].MakeTransd() << std::endl;
    return new Eigen::Vector3d(Joints[idx].GetRotation()*(Joints[idx].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3));

    
}

 void SandBox::MoveSnake() {
    // vQ - rotations of joints
    // vT - translation of joints
    RotationList vQ, vQ_tmp;
    Eigen::Quaterniond localRot, r;
    Eigen::Vector3d t;
    std::vector<Eigen::Vector3d> vT, vT_tmp;
    if (AutoMode) {
        if (targets_to_show.size() > 0) {
            int idx = targets_to_show[0];
            Eigen::Vector3d curTargetPos = (data_list[idx].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
            Eigen::Vector3d head_pos = *getJointPos(HEAD_IDX);
            t = (curTargetPos - head_pos).normalized() * velocity;


        }
    }
    else {

        //Eigen::Vector3d tarPos = (target.MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
        Eigen::Vector3d tarOrientation = target.GetRotation()*Eigen::Vector3d(0,0,1);
        //Eigen::Vector3d head_pos = *getJointPos(HEAD_IDX);
        //std::cout << tarPos.transpose() << "     " << head_pos.transpose() << std::endl;
        //t = (tarPos - head_pos).normalized() * velocity;
        t = tarOrientation * velocity;
    }

    Joints[HEAD_IDX].MyTranslate(t, true);
    vT_tmp.push_back(t);
    //vQ_tmp.push_back(r);
    vQ_tmp.push_back(Eigen::Quaterniond::Identity());

    for (int i = HEAD_IDX - 1; i >= 0; i--) {
        Eigen::Vector3d nextJoint = *getJointPos(i + 1);
        Eigen::Vector3d curJoint = *getJointPos(i);

        //correct rotation to next joint
        double DistanceToParent = (nextJoint - curJoint).norm();
        double Distance = DistanceToParent - step_size.norm();
        //Eigen::Vector3d target = curJoint + (nextJoint - curJoint).normalized()*Distance;
        //r = Eigen::Quaterniond::FromTwoVectors(curJoint, target);

        //Joints[i].MyRotate(r);
        //vQ_tmp.push_back(r);
        vQ_tmp.push_back(Eigen::Quaterniond::Identity());
        //curJoint = *getJointPos(i);


        //vT_tmp.push_back(target- curJoint);
        vT_tmp.push_back((nextJoint - curJoint).normalized() * Distance);
        Joints[i].MyTranslate((nextJoint - curJoint).normalized() * Distance, true);




    }

    for (int i = vQ_tmp.size() - 1; i >= 0; i--) {
        vQ.push_back(vQ_tmp[i]);
        vT.push_back(vT_tmp[i]);
    }
    igl::dqs(data_list[0].V, M, vQ, vT, U); //original was W instead of M

    data_list[0].set_vertices(U);
    data_list[0].compute_normals();

    //isActive = false;
}


/*void SandBox::MoveSnake() {
    // vQ - rotations of joints
    // vT - translation of joints
    RotationList vQ, vQ_tmp;
    Eigen::Quaterniond localRot, r;
    Eigen::Vector3d t;
    std::vector<Eigen::Vector3d> vT, vT_tmp;
    if (AutoMode) {
        if (targets_to_show.size() > 0) {
            int idx = targets_to_show[0];
            Eigen::Vector3d curTargetPos = (data_list[idx].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
            Eigen::Vector3d head_rotation = Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d head_pos = *getJointPos(HEAD_IDX);
            r = Eigen::Quaterniond::FromTwoVectors(head_rotation, curTargetPos);
            r = r.slerp(0.98, Eigen::Quaterniond::Identity());
            //Joints[HEAD_IDX].MyRotate(r);
            r = Eigen::Quaterniond::Identity();
            head_rotation = Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 0, 1);
            t = (curTargetPos - head_pos).normalized() * velocity;


        }
    }
    else {
        Eigen::Vector3d head_rotation = Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d head_direction = (*getJointPos(HEAD_IDX) - *getJointPos(HEAD_BASE)).normalized();
        Eigen::Vector3d prevPos = *getJointPos(HEAD_IDX);
        Eigen::Vector3d nextPos = prevPos + head_direction.normalized() * velocity;
        r = Eigen::Quaterniond::FromTwoVectors(prevPos, nextPos);
        Joints[HEAD_IDX].MyRotate(r);
        t = nextPos - *getJointPos(HEAD_IDX);
    }
    
    Joints[HEAD_IDX].MyTranslate(t, true);
    vT_tmp.push_back(t);
    vQ_tmp.push_back(r);

    for (int i = HEAD_IDX -1; i >= 0; i--) {
        Eigen::Vector3d nextJoint = *getJointPos(i + 1);
        Eigen::Vector3d curJoint = *getJointPos(i);

        //correct rotation to next joint
        double DistanceToParent = (nextJoint - curJoint).norm();
        double Distance = DistanceToParent - step_size.norm();
        //Eigen::Vector3d target = curJoint + (nextJoint - curJoint).normalized()*Distance;
        //r = Eigen::Quaterniond::FromTwoVectors(curJoint, target);
        
        //Joints[i].MyRotate(r);
        //vQ_tmp.push_back(r);
        vQ_tmp.push_back(Eigen::Quaterniond::Identity());
        //curJoint = *getJointPos(i);


        //vT_tmp.push_back(target- curJoint);
        vT_tmp.push_back((nextJoint- curJoint).normalized()*Distance);
        Joints[i].MyTranslate((nextJoint - curJoint).normalized() * Distance, true);




    }

    for (int i = vQ_tmp.size() - 1; i >= 0; i--) {
        vQ.push_back(vQ_tmp[i]);
        vT.push_back(vT_tmp[i]);
    }
    igl::dqs(data_list[0].V, M, vQ, vT, U); //original was W instead of M

    data_list[0].set_vertices(U);
    data_list[0].compute_normals();

   //isActive = false;
}*/


void SandBox::FollowWithCamera( bool start){
    if (start) {
        FollowMe = true;
        original_base_zoom = render->core().camera_base_zoom;
        render->core().camera_base_zoom = 1;
        render->core().camera_base_translation = Eigen::Vector3f(0, 0, 0);
        //render->core().camera_zoom =;

        Eigen::Vector3f cur_orig = render->core().camera_eye;
        Eigen::Vector3f head_base = getJointPos(HEAD_BASE)->cast<float>();
        Eigen::Vector3f eye_base = getJointPos(HEAD_BASE -2)->cast<float>();
        Eigen::Vector3f local_dir = (getJointPos(HEAD_BASE - 1)->cast<float>() - getJointPos(HEAD_BASE - 3)->cast<float>()).normalized();
        Eigen::Vector3f middle = getJointPos(NUM_JOINTS / 2 + 1)->cast<float>();
        Eigen::Vector3f head_tip = getJointPos(HEAD_IDX)->cast<float>();
        Eigen::Vector3f up_vec = (Joints[HEAD_BASE - 2].GetRotation()*Eigen::Vector3d(0,0,1)).cast<float>();
        Eigen::Vector3f rightDirection = (Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(-1, 0, 0)).cast<float>();
        Eigen::Vector3f head_direction = (Joints[HEAD_IDX].GetRotation()*Eigen::Vector3d(0,0,1)).cast<float>();
        Eigen::Vector3f head_direction2 = (head_tip - head_base).normalized();
        //Eigen::Vector3f up_direction = Joints[HEAD_BASE - 2].GetRotation()*Eigen::Vector3d(0,0,1).cast<float>();
        Eigen::Vector3f up_direction = rightDirection.cross(head_direction).normalized();
        //up_direction = (up_direction - up_direction.dot(local_dir) * up_direction).normalized();
        eye_base = head_base;
        Eigen::Vector3f eye = eye_base + up_direction*4 ;
        //eye = up_direction * 3;
        Eigen::Vector3f center = head_tip  - up_direction;
        //Eigen::Vector3f center = head_tip + head_direction*0.01 - up_direction;
     
        render->SetCameraEye(eye);
        render->centerCamera(center);
        render->core().camera_up = up_direction;
        center = render->core().camera_center;
    }
 




}

/*void SandBox::FollowWithCamera(Renderer& renderer, bool start) {
    if (start) {
        //core.camera_center = getJointPos(HEAD_IDX)->cast<float>();
        FollowMe = true;
        render = &renderer;
        original_base_zoom = render->core().camera_base_zoom;
        render->core().camera_base_zoom = 1;
        
        Eigen::Vector3f eye = (Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 3, 0)).cast<float>();
        //Eigen::Vector3f eye = (  head_tip).normalized();
        Eigen::Vector3f translation = (Joints[HEAD_BASE].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3).cast<float>();

        render->SetCameraTranslation(-translation);
        render->SetCameraEye(eye);
        Eigen::Vector3f up = (Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 0, 1)).cast<float>();
        //render->centerCamera(Eigen::Vector3f(0, 0, 1));
        render->core().camera_up = up.normalized();
        Eigen::Vector3f head_base = getJointPos(HEAD_BASE)->cast<float>();
        Eigen::Vector3f head_tip = getJointPos(HEAD_IDX)->cast<float>();
        Eigen::Vector3f eye = (  head_tip + (head_tip - head_base)).normalized();
        //Eigen::Vector3f eye = (  head_tip).normalized();
        Eigen::Vector3f translation = (Joints[HEAD_IDX].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3).cast<float>();
        
        render->SetCameraTranslation(-translation);
        render->SetCameraEye(eye);
        Eigen::Vector3f up = (Joints[HEAD_IDX].GetRotation() * Eigen::Vector3d(0, 1, 0)).cast<float>();
        //render->centerCamera(Eigen::Vector3f(0, 0, 1));
        render->core().camera_up = up.normalized();
        //Eigen::Vector3f eye = Eigen::Vector3f(0,0,1+ (1.6-1)* render->core().camera_base_zoom);

        //Eigen::Vector3f eye = Eigen::Vector3f(0, 0, 4);
        //render->SetCameraEye(eye);
       // Eigen::Vector3f translation = Eigen::Vector3f(0,0,-1);
        //render->centerCamera(Eigen::Vector3f(0, 0, 4)*render->core().camera_base_zoom);
       // render->SetCameraTranslation(translation);
        


        

    }
    else {
        FollowMe = false;
        render->core().camera_base_zoom = original_base_zoom;
        Eigen::Vector3f p = getJointPos(NUM_JOINTS / 2)->cast<float>() + Eigen::Vector3f(0,0,10);
        render->core().camera_up = Eigen::Vector3f(0, 1, 0);
        Eigen::Vector3f eye = p;
        render->SetCameraEye(eye);

    }
}*/

void SandBox::Init(const std::string &config)
{
   
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
    isActive = false;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
        num = 0;
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
            Eigen::RowVector3d center(0, 0, -0.8);
            data().add_points(center, Eigen::RowVector3d(0, 0, 1));
            if (num <= 1) {
                parents.push_back(-1);
            }
            else {
                parents.push_back(num-1);
            }
			
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(true, 1);
            if (num != 0) {
                data().SetCenterOfRotation(center.transpose());
            }
            
            num++;

			
		}
		nameFileout.close();
	}
    data_list[0].MyTranslate(Eigen::Vector3d(0, 0, 5), true);
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
//	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
   ResetScene(true);

}

void SandBox::ResetScene(bool onStart) {
    if (onStart) {
        Eigen::MatrixXd V_axis(4,3);

        Eigen::MatrixXi E(3,2);
        E <<
            0, 1,
            0, 2,
            0, 3;

       
        for (float i = 1; i < data_list.size(); i++) {
            data_list[i].MyTranslate( Eigen::Vector3d(0, 0, link_len), true);
            Eigen::Vector3d center;

            center = data_list[i].GetCenterOfRotation();
            
            V_axis <<
                center(0), center(1), center(2),
                2 * link_len, 0, 0,
                0, 2 * link_len, 0,
                0, 0, 2 * link_len;
            data_list[i].add_points(V_axis.row(0), Eigen::RowVector3d(0, 0, 0));
            data_list[i].add_edges(V_axis.row(E(0, 0)), V_axis.row(E(0, 1)), Eigen::RowVector3d(1, 0, 0));
            data_list[i].add_edges(V_axis.row(E(1, 0)), V_axis.row(E(1, 1)), Eigen::RowVector3d(0, 1, 0));
            data_list[i].add_edges(V_axis.row(E(2, 0)), V_axis.row(E(2, 1)), Eigen::RowVector3d(0, 0, 1));
        }
        setDataStructure();
    }
    else {
        Eigen::RowVector3d center(0, 0, -0.8);
        for (int i = 1; i < num; i++) {
            data_list[i].resetTranslation();
            data_list[i].MyTranslate(Eigen::Vector3d(0, 0, link_len), true);
            data_list[i].SetCenterOfRotation(center.transpose());
        }
    }
}

void SandBox::setDataStructure() {
    //calculate original tips position
    tips.push_back(new Eigen::Vector3d(0, 0, 0));
    Eigen::Vector3d start(0, 0, link_len/2);
    for (int i = 1; i < num; i++) {
        Eigen::Vector3d tip(0,0,0);
        tip = tip + start;
        tips.push_back(new Eigen::Vector3d(tip));
        start[2] += link_len;
        
    }

    //Saves target position
    dest = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
    armLen = (num-1) * link_len;



}

Eigen::Vector3d* SandBox::getTip(int idx) {
    Eigen::Vector3d O = (data_list[1].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
    *tips[0] = O;
    *tips[1] = O + data_list[1].GetRotation() * (Eigen::Vector3d(0,0,link_len/2));
    int newTips = data_list.size() - tips.size();
    for (int k = 0; k < newTips; k++) {
        tips.push_back(new Eigen::Vector3d(0, 0, 0));
    }
    for (int i = 2; i < num; i++) {
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        for (int j = 0; j < i; j++) {
            rot = rot * data_list[j + 1].GetRotation();
        }
        *tips[i] = (*tips[i - 1]) + rot * (Eigen::Vector3d(0,0,link_len));
    }
    return new Eigen::Vector3d(*tips[idx]);



}

double SandBox::calculateAngle(Eigen::Vector3d vec1, Eigen::Vector3d vec2) {
    double cos_alpha = vec1.normalized().dot(vec2.normalized());
    cos_alpha = std::max(-1.0, cos_alpha);
    cos_alpha = std::min(1.0, cos_alpha);
    return acos(cos_alpha);
}

void SandBox::ccd() {
  

    num = data_list.size();
    for (int i = num - 1; i > 0; i--) {


        Eigen::Vector3d armBase = *getTip(i - 1);
        Eigen::Vector3d endEffector = *getTip(num - 1);
        Eigen::Vector3d armVec = endEffector - armBase;
        Eigen::Vector3d targetVector = dest - armBase;

        double phi = calculateAngle(armVec, targetVector);

        Eigen::Vector3d rotAxis = (armVec.normalized().cross(targetVector.normalized())).normalized();


        //data_list[i].RotateInSystem(rotAxis, phi/10);
        data_list[i].MyRotate(rotAxis, phi/10);




 

        
    }
}

void SandBox::Fabrik() {

    std::vector<Eigen::Vector3d> forward;
    std::vector<Eigen::Vector3d> backward;
    Eigen::Vector3d* base = getTip(0);
    

    //forward reaching
    forward.push_back(dest);
    for (int i = num - 2; i >= 0; i--) {
        Eigen::Vector3d tip = *getTip(i);
        double r = (tip - forward.back()).norm();
        double lamda = link_len / r;
        forward.push_back((1 - lamda) * forward.back() + lamda * tip);
    }

    //Backward reaching
    backward.push_back(*base);
    for (int j = 1; j <= num-1; j++) {
        double r = (forward[num - 1 - j] - backward.back()).norm();
        double lamda = link_len / r;
        backward.push_back((1 - lamda) * backward.back() + lamda * forward[num - 1 - j]);
    }

    //Rotating step
    for(int k = 1; k <= num-1; k++){
        Eigen::Vector3d tipBase = *getTip(k - 1);
        Eigen::Vector3d tip = *getTip(k);
        Eigen::Vector3d original = (tip - tipBase).normalized();
        Eigen::Vector3d calculated = (backward[k] - tipBase).normalized();
        double angle = calculateAngle(original, calculated);

        if ((abs(abs(angle) - M_PI)) < 0.1) {
            std::cout << "opposite direction" << std::endl;
            data_list[k].MyRotate(Eigen::Vector3d(1,0,0), M_PI);
            continue;
        }

        Eigen::Vector3d rotAxis = original.cross(calculated);
        
        data_list[k].MyRotate(rotAxis, angle/15);
        //data_list[k].RotateInSystem(rotAxis, angle/15);

    }

}

void SandBox::ObjectLoader(const std::string config, bool is_shortest)
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    std::string item_name;
    std::ifstream nameFileout;
    nameFileout.open(config);
    isActive = false;
  

    if (!nameFileout.is_open())
    {
        std::cout << "Can't open file " << config << std::endl;
    }
    else
    {
        int i = 0;
        while (nameFileout >> item_name)
        {
            i++;
            // Create new data slot and set to selected
            std::cout << "openning " << item_name << std::endl;
            load_mesh_from_file(item_name);
            parents.push_back(-1);
            data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(true, 1);
            num_collapsed.push_back(0);
            obb_trees.push_back(new igl::AABB<MatrixXd, 3>());
            direction.push_back(new Eigen::Vector3d(0, 0, 0));
    /**        if (!(data().F.rows() == 0 && data().V.rows() == 0))
            {
                append_mesh();
            }
            data().clear();
            MatrixXd V;
            MatrixXi F;
            read_triangle_mesh(item_name, V, F);
            data().set_mesh(V, F);*/
            if (is_shortest) {
                SetQueue(data().V, data().F);
            }
            else {
                _setQueue(data().V, data().F);
                int idx = mesh_index(data().id);
                Eigen::AlignedBox<double, 3> box;
                igl::AABB<Eigen::MatrixXd, 3>* tree = obb_trees[idx];
                box = tree->m_box;
                draw_obb(idx, box, Eigen::RowVector3d(0,0,1));

            }
            



        }
        starting_point.resize(i);
        for (int j = 0; j < i; j++) {
            starting_point[j] = obb_trees[j];
        }

    }
}

void SandBox::SimplifyMesh(int n)
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    size_t curIdx = mesh_index(data().id);
    if (!Q[curIdx]->empty())
    {
        bool something_collapsed = false;
        // collapse edge
        const int max_iter = n;
        Eigen::MatrixXd nV = data().V;
        Eigen::MatrixXi nF = data().F;
        
        for (int j = 0; j < max_iter; j++)
        {

            if (is_shortest) {
                if (!collapse_edge(
                    shortest_edge_and_midpoint, nV, nF, *E[curIdx], *EMAP[curIdx], *EF[curIdx], *EI[curIdx], *Q[curIdx], Qit[curIdx], *C[curIdx]))
                {
                    break;
                }
            }
            else {
                if (!myCollapse_edge(nV, nF)) {
                    break;
                }
            }

            something_collapsed = true;
            num_collapsed[curIdx]++;
            printf("something collapsed: %d\n", num_collapsed[curIdx]);
        }

        if (something_collapsed)
        {
            data().clear();
            data().set_mesh(nV, nF);
            data().set_face_based(true);
            data().dirty = 157;
        }
    }
}


void SandBox::SetQueue(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    // Prepare array-based edge data structures and priority queue
    VectorXi* EMAPtmp = new VectorXi();
    MatrixXi* Etmp = new MatrixXi(), * EFtmp = new MatrixXi(), * EItmp = new MatrixXi();
    PriorityQueue *Qtmp = new PriorityQueue();
    std::vector<PriorityQueue::iterator> QitTmp;
    
    // If an edge were collapsed, we'd collapse it to these points:
    edge_flaps(F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);
    C.push_back(new MatrixXd(Etmp->rows(), V.cols()));
    QitTmp.resize(Etmp->rows());
    VectorXd costs(Etmp->rows());
    Qtmp->clear();
    

    for (int e = 0; e < Etmp->rows(); e++)
    {
        double cost = e;
        RowVectorXd p(1, 3);
        shortest_edge_and_midpoint(e, V, F, *Etmp, *EMAPtmp, *EFtmp, *EItmp, cost, p);
        C.back()->row(e) = p;
        costs(e) = cost;
        QitTmp[e] = Qtmp->insert(std::pair<double, int>(cost, e)).first;
    }


    EMAP.push_back(EMAPtmp);
    E.push_back(Etmp);
    EF.push_back(EFtmp);
    EI.push_back(EItmp);
    Q.push_back(Qtmp);
    Qit.push_back(*new std::vector<PriorityQueue::iterator>(QitTmp));

    
}

void SandBox::_setQueue(Eigen::MatrixXd& V, Eigen::MatrixXi& F) 
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    // Prepare array-based edge data structures and priority queue
    VectorXi* EMAPtmp = new VectorXi();
    MatrixXi* Etmp = new MatrixXi(), * EFtmp = new MatrixXi(), * EItmp = new MatrixXi();
    PriorityQueue* Qtmp = new PriorityQueue();
    std::vector<PriorityQueue::iterator> QitTmp;
    std::vector<Eigen::Matrix4d*>* errors = new std::vector<Eigen::Matrix4d*>;
    errors->resize(V.rows());
    vertex_errors.push_back(*errors);

    // If an edge were collapsed, we'd collapse it to these points:
    edge_flaps(F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);
    C.push_back(new MatrixXd(Etmp->rows(), V.cols()));
    QitTmp.resize(Etmp->rows());
    VectorXd costs(Etmp->rows());
    Qtmp->clear();

    EMAP.push_back(EMAPtmp);
    E.push_back(Etmp);
    EF.push_back(EFtmp);
    EI.push_back(EItmp);

    for (int e = 0; e < Etmp->rows(); e++)
    {
        VectorXi edge = Etmp->row(e);
        update_vertex_errors(edge.x(),V,F);
        update_vertex_errors(edge.y(),V,F);

    }

    for (int e = 0; e < Etmp->rows(); e++)
    {
        double cost = e;
        RowVectorXd p(1, 3);
        calculate_cost_and_placement(e, cost, p);
        C.back()->row(e) = p;
        costs(e) = cost;
        QitTmp[e] = Qtmp->insert(std::pair<double, int>(cost, e)).first;
    }



    Q.push_back(Qtmp);
    Qit.push_back(*new std::vector<PriorityQueue::iterator>(QitTmp));
    obb_trees.back()->init(V, F);
    






}


size_t SandBox::getQsize(int id)
{
    return Q[mesh_index(id)]->size();
}

Eigen::Matrix4d* SandBox::calculate_plane_err(Eigen::Vector3d normal, Eigen::Vector3d v)
{
    double d = v.transpose() * normal;
    d *= -1.0;
    Eigen::Vector4d p;
    p<< normal, d;
    Eigen::Matrix4d tmp = p * p.transpose();
    Eigen::Matrix4d* K = new Eigen::Matrix4d(tmp);
    
    return K;
}

void SandBox::update_vertex_errors(int v, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    int data_idx = mesh_index(data().id);
   
    Eigen::Matrix4d* K =  new Eigen::Matrix4d(Eigen::Matrix4d::Zero());
    Eigen::Matrix4d zero = Eigen::Matrix4d::Zero();
    Eigen::Vector3i face;

    for (int i = 0; i < F.rows(); i++)
    {

        face  =  F.row(i);
        if (face.x() == 0 && face.y() == 0 && face.z() == 0)
        {
            continue; // the face already collapsed
        }
        if (face.x() == v || face.y() == v || face.z() == v)
        {
            
            *K += *calculate_plane_err(data().F_normals.row(i).normalized(), data().V.row(v));

        }


    }
    vertex_errors[data_idx][v] = K;
}



Eigen::Matrix4d* SandBox::derived_matrix(Eigen::Matrix4d& K) 
{
    Eigen::Matrix4d* res = new Eigen::Matrix4d();
    res->row(0) = K.row(0);
    res->row(1) = K.row(1);
    res->row(1)[0] = K.row(0)[1];
    res->row(2) = K.col(2);
    res->row(2)[3] = K.row(2)[3];
    res->row(3) = *new Eigen::RowVector4d(0, 0, 0, 1);
    return res;

    
}

Eigen::Vector4d* SandBox::min_choice(int v1, int v2, Eigen::Matrix4d& K) 
{
    int idx = mesh_index(data().id);
    double cost1, cost2, cost3;
    Eigen::Vector3d V1 = data().V.row(v1), V2 = data().V.row(v2);
    Eigen::Vector4d t1,t2,t3,t;
    t1 << (V1 + V2) / 2, 1;
    t2 << V1, 1;
    t3 << V2, 1;
    if (t1.transpose() * K * t1 < t2.transpose() * K * t2)
    {
        t = t1;
    }
    else {
        t = t2;
    }
    if (t.transpose() * K * t > t3.transpose() * K * t3)
    {
        t = t3;
    }
    Eigen::Vector4d *res = new Eigen::Vector4d(t);

    return res;

}



void SandBox::calculate_cost_and_placement(const int e, double& cost, Eigen::RowVectorXd& p)
{
    using namespace Eigen;
    int curIdx = mesh_index(data().id);
    Vector2i edge = E[curIdx]->row(e);
    Matrix4d K = *vertex_errors[curIdx][edge.x()] + *vertex_errors[curIdx][edge.y()];
    Vector4d new_pos;   
    Matrix4d dK = *derived_matrix(K);
    if (dK.determinant() > 0) {
        
        new_pos = dK.inverse() * Vector4d(0, 0, 0, 1);
    }
    else {
        new_pos = *min_choice(edge.x(), edge.y(), K);
    }
        
   
    double *cost_res = new double (new_pos.transpose() * K * new_pos);
    if (new_pos[3] != 0) {
        new_pos[0] /= new_pos[3];
        new_pos[1] /= new_pos[3];
        new_pos[2] /= new_pos[3];
    }
    p = *new Vector3d(new_pos.head(3));
    cost = *cost_res;
}

void SandBox::print_data_structure()
{
    size_t curIdx = mesh_index(data().id);
    std::cout << "V:\n" << data().V << std::endl;
    std::cout << "F:\n" << data().F << std::endl;
    std::cout << "E:\n" << *E[curIdx] << std::endl;
    std::cout << "EF:\n" << *EF[curIdx] << std::endl;
    std::cout << "EI:\n" << *EI[curIdx] << std::endl;
    std::cout << "EMAP:\n" << *EMAP[curIdx] << std::endl;
    std::cout << "F_normals:\n" << data().F_normals << std::endl;
}



bool SandBox::myCollapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    int curIdx = mesh_index(data().id);
    std::vector<Eigen::Matrix4d*> v_errors = vertex_errors[curIdx];

    if (Q[curIdx]->empty()) {
        return false;
    }

    std::pair<double, int> p = *(*Q[curIdx]).begin();
    int e = p.second;
    double cost = p.first;
    Eigen::Vector3d* n_pos = new Eigen::Vector3d(C[curIdx]->row(e));
   // std::cout << "edge to remove: " << e << std::endl;


        const auto& derived_matrix = [](Eigen::Matrix4d& K, Eigen::Matrix4d& res)
        {
       // Eigen::Matrix4d* res = new Eigen::Matrix4d();
        res.row(0) = K.row(0);
        res.row(1) = K.row(1);
        res.row(1)[0] = K.row(0)[1];
        res.row(2) = K.col(2);
        res.row(2)[3] = K.row(2)[3];
        res.row(3) = *new Eigen::RowVector4d(0, 0, 0, 1);
        

        };
        const auto& min_choice = [&V](int v1, int v2, Eigen::Matrix4d& K, Eigen::Vector4d& res)
        {
            Eigen::Vector3d V1 = V.row(v1), V2 = V.row(v2);
            Eigen::Vector4d t1, t2, t3, t;
            t1 << (V1 + V2) / 2, 1;
            t2 << V1, 1;
            t3 << V2, 1;
            if (t1.transpose() * K * t1 < t2.transpose() * K * t2)
            {
                t = t1;
            }
            else {
                t = t2;
            }
            if (t.transpose() * K * t > t3.transpose() * K * t3)
            {
                t = t3;
            }
            res = *new Eigen::Vector4d(t);

        

        };
        const auto& calculate_cost_and_placement = [&derived_matrix, &min_choice, &v_errors](
            const int e,
            const Eigen::MatrixXd& V,
            const Eigen::MatrixXi& /*F*/,
            const Eigen::MatrixXi& E,
            const Eigen::VectorXi& /*EMAP*/,
            const Eigen::MatrixXi& /*EF*/,
            const Eigen::MatrixXi& /*EI*/,
            double& cost,
            Eigen::RowVectorXd& p)
        {
            using namespace Eigen;
            Vector2i edge = E.row(e);
            Matrix4d K = *v_errors[edge.x()] + *v_errors[edge.y()];
            Vector4d new_pos;
            Matrix4d dK;
            derived_matrix(K,dK);
            if (dK.determinant() > 0) {
                
                new_pos = dK.inverse() * Vector4d(0, 0, 0, 1);
            }
            else {
                min_choice(edge.x(), edge.y(), K, new_pos);
            }


            double* cost_res = new double(new_pos.transpose() * K * new_pos);
            if (new_pos[3] != 0) {
                new_pos[0] /= new_pos[3];
                new_pos[1] /= new_pos[3];
                new_pos[2] /= new_pos[3];
            }
            p = *new Vector3d(new_pos.head(3));
            cost = *cost_res;

        };
        bool res = igl::collapse_edge(calculate_cost_and_placement, V, F, *E[curIdx], *EMAP[curIdx], *EF[curIdx], *EI[curIdx], *Q[curIdx], Qit[curIdx], *C[curIdx]);
        if (res) {
            std::cout << "edge " << e << " cost = " << cost << " new v position " << n_pos->x() << " " << n_pos->y() << " " << n_pos->z() << std::endl;
        }
        return res;
        

}



Eigen::Matrix3d* SandBox::calculateOriginalAxes(Eigen::AlignedBox<double, 3> box) {
  Eigen::Vector3d BottomLeftFloor = box.corner(Eigen::AlignedBox3d::BottomLeftFloor);
  Eigen::Vector3d BottomRightFloor = box.corner(Eigen::AlignedBox3d::BottomRightFloor);
  Eigen::Vector3d TopLeftFloor = box.corner(Eigen::AlignedBox3d::TopLeftFloor);
  Eigen::Vector3d BottomLeftCeil = box.corner(Eigen::AlignedBox3d::BottomLeftCeil);
  Eigen::Matrix3d *res = new Eigen::Matrix3d(3,3);
  res->col(0) = (BottomRightFloor - BottomLeftFloor).normalized();
  res->col(1) = (TopLeftFloor - BottomLeftFloor).normalized();
  res->col(2) = (BottomLeftCeil - BottomLeftFloor).normalized();
  return res;

}

bool SandBox::isIntersect(Eigen::AlignedBox<double, 3> box0, Eigen::AlignedBox<double, 3> box1, Movable* obj0, Movable* obj1) {
    Eigen::Matrix4d translation0 = obj0->MakeTransd();
    Eigen::Matrix4d translation1 = obj1->MakeTransd();
    
    Eigen::Matrix3d orig0 = *calculateOriginalAxes(box0);
    Eigen::Matrix3d orig1 = *calculateOriginalAxes(box1);
    Eigen::Matrix3d AxisBox0 = obj0->GetRotation(); // axis box0
    
    AxisBox0 = AxisBox0 * orig0;
    std::cout << AxisBox0 << std::endl;
    Eigen::Matrix3d AxisBox1 = obj1->GetRotation(); // axis box1
    AxisBox1 = AxisBox1 * orig1;
    Eigen::Vector4d hc0 = Eigen::Vector4d();
    hc0 << box0.center(), 1;

    Eigen::Vector4d hc1 = Eigen::Vector4d();
 
    hc1 << box1.center(), 1;

    Eigen::Vector4d tmp0 = translation0 * hc0;
    Eigen::Vector4d tmp1 = translation1 * hc1;
    if (tmp0(3) != 0) {
        tmp0(0) /= tmp0(3);
        tmp0(1) /= tmp0(3);
        tmp0(2) /= tmp0(3);
    }
    if (tmp1(3) != 0) {
        tmp1(0) /= tmp1(3);
        tmp1(1) /= tmp1(3);
        tmp1(2) /= tmp1(3);

    }
    Eigen::Vector3d c0 = tmp0.head(3); // box center box0
    Eigen::Vector3d c1 = tmp1.head(3); // box center box1
    std::cout << "box 0 center = " << c0 << std::endl;
    std::cout << "box 1 center = " << c1 << std::endl;
    Eigen::Vector3d D = c1 - c0;
    Eigen::Matrix3d C = AxisBox0.transpose() * AxisBox1; // base transfomation matrix
    Eigen::Vector3d extent0 = box0.sizes() / 2; // size of the box
    Eigen::Vector3d extent1 = box1.sizes() / 2; // size of the box


    double l = extent0(0) + extent1.transpose().dot(C.row(0).cwiseAbs());//case 1
    double R = std::abs(AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) + extent1.transpose().dot(C.row(1).cwiseAbs()); // case 2
    R = std::abs(AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(2) + extent1.transpose().dot(C.row(2).cwiseAbs()); // case 3
    R = std::abs(AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(0) + extent0.transpose().dot(C.col(0).cwiseAbs()); // case 4
    R = std::abs(AxisBox1.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(1) + extent0.transpose().dot(C.col(1).cwiseAbs()); // case 5
    R = std::abs(AxisBox1.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(2) + extent0.transpose().dot(C.col(2).cwiseAbs()); // case 6
    R = std::abs(AxisBox1.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) * std::abs(C(2, 0)) + extent0(2) * std::abs(C(1, 0))
        + extent1(1) * std::abs(C(0, 2)) + extent1(2) * std::abs(C(0, 1)); //case 7

    R = std::abs(C(1, 0) * AxisBox0.col(2).dot(D) - C(2, 0) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) * std::abs(C(2, 1)) + extent0(2) * std::abs(C(1, 1))
        + extent1(0) * std::abs(C(0, 2)) + extent1(2) * std::abs(C(0, 0));//case 8

    R = std::abs(C(1, 1) * AxisBox0.col(2).dot(D) - C(2, 1) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) * std::abs(C(2, 2)) + extent0(2) * std::abs(C(1, 2))
        + extent1(0) * std::abs(C(0, 1)) + extent1(1) * std::abs(C(0, 0));//case 9

    R = std::abs(C(1, 2) * AxisBox0.col(2).dot(D) - C(2, 2) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 0)) + extent0(2) * std::abs(C(0, 0))
        + extent1(1) * std::abs(C(1, 2)) + extent1(2) * std::abs(C(1, 1));//case 10

    R = std::abs(C(2, 0) * AxisBox0.col(0).dot(D) - C(0, 0) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 1)) + extent0(2) * std::abs(C(0, 1))
        + extent1(0) * std::abs(C(1, 2)) + extent1(2) * std::abs(C(1, 0));//case 11

    R = std::abs(C(2, 1) * AxisBox0.col(0).dot(D) - C(0, 1) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 2)) + extent0(2) * std::abs(C(0, 2))
        + extent1(0) * std::abs(C(1, 1)) + extent1(1) * std::abs(C(1, 0));//case 12

    R = std::abs(C(2, 2) * AxisBox0.col(0).dot(D) - C(0, 2) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }


    l = extent0(0) * std::abs(C(1, 0)) + extent0(1) * std::abs(C(0, 0))
        + extent1(1) * std::abs(C(2, 2)) + extent1(2) * std::abs(C(2, 1));//case 13

    R = std::abs(C(0, 0) * AxisBox0.col(1).dot(D) - C(1, 0) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(1, 1)) + extent0(1) * std::abs(C(0, 1))
        + extent1(0) * std::abs(C(2, 2)) + extent1(2) * std::abs(C(2, 0));//case 14

    R = std::abs(C(0, 1) * AxisBox0.col(1).dot(D) - C(1, 1) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(1, 2)) + extent0(1) * std::abs(C(0, 2))
        + extent1(0) * std::abs(C(2, 1)) + extent1(1) * std::abs(C(2, 0));//case 15

    R = std::abs(C(0, 2) * AxisBox0.col(1).dot(D) - C(1, 2) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }


    return true;





}

bool SandBox::CatchTarget() {
    if (targets_to_show.size() == 0) {
        levelUp = true;
        return false;
    }
    for (int idx = 0; idx < targets_to_show.size(); idx++) {
        Eigen::Vector3d snake_head = *getJointPos(HEAD_IDX);
        Eigen::Vector3d sphere_center = (data_list[targets_to_show[idx]].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
        //std::cout << "target number " << targets_to_show[idx] << " d from snake head =  " << (snake_head - sphere_center).norm() << std::endl;

        if ((snake_head - sphere_center).norm() <= (SPHERE_RADIUS)) {
            data_list[targets_to_show[idx]].set_visible(false, 1);
            data_list[targets_to_show[idx]].set_visible(false, 2);
            targets_to_show.erase(targets_to_show.begin() + idx);
            return true;
        }
    }

    return false;
    
}

void SandBox::LevelUp(int n) {
    std::cout << "starting level: " << n << std::endl;
    ResetJoints();
    data_list[0].set_vertices(Voriginal);
    data_list[0].compute_normals();
    double amtX = 30*M_PI/180 * (n % 4);
    if (n % 4 == 0) {
        sqrt_targets = std::max((sqrt_targets + 1)%9,2);
        
    }
    cur_num_tragets = sqrt_targets * sqrt_targets;
    Eigen::Matrix3d xMat;
    xMat << 1, 0, 0, 0, cos(amtX), -sin(amtX), 0, sin(amtX), cos(amtX);
    Eigen::Vector3d d1 = xMat * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d d2 = Eigen::Vector3d(1, 0, 0);
    int size = sqrt_targets * 2;
    int numTargets = calculateGrid(d1, d2, Eigen::Vector3d(6, 0, -6), size);
    MoveTargets(numTargets);
    levelUp = false;





}

bool SandBox::isIntersect(Eigen::AlignedBox<double, 3> box0, Eigen::AlignedBox<double, 3> box1) {
    Eigen::Matrix4d translation0 = data_list[0].MakeTransd();
    Eigen::Matrix4d translation1 = data_list[1].MakeTransd();
    Eigen::Matrix3d orig0 = *calculateOriginalAxes(box0);
    Eigen::Matrix3d orig1 = *calculateOriginalAxes(box1);
    Eigen::Matrix3d AxisBox0 = data_list[0].GetRotation(); // axis box0
    AxisBox0 =  AxisBox0*orig0;
    Eigen::Matrix3d AxisBox1 = data_list[1].GetRotation(); // axis box1
    AxisBox1 = AxisBox1 * orig1;
    Eigen::Vector4d hc0 = Eigen::Vector4d();
    hc0 << box0.center(),1;
    Eigen::Vector4d hc1 = Eigen::Vector4d();
    hc1 << box1.center(), 1;

    Eigen::Vector4d tmp0 = translation0 * hc0;
    Eigen::Vector4d tmp1 = translation1 * hc1;
    if (tmp0(3) != 0) {
        tmp0(0) /= tmp0(3);
        tmp0(1) /= tmp0(3);
        tmp0(2) /= tmp0(3);
    }
    if (tmp1(3) != 0) {
        tmp1(0) /= tmp1(3);
        tmp1(1) /= tmp1(3);
        tmp1(2) /= tmp1(3);

    }
    Eigen::Vector3d c0 = tmp0.head(3); // box center box0
    Eigen::Vector3d c1 = tmp1.head(3); // box center box1
    Eigen::Vector3d D = c1 - c0;
    Eigen::Matrix3d C = AxisBox0.transpose() * AxisBox1; // base transfomation matrix
    Eigen::Vector3d extent0 = box0.sizes()/2; // size of the box
    Eigen::Vector3d extent1 = box1.sizes()/2; // size of the box


    double l = extent0(0) + extent1.transpose().dot(C.row(0).cwiseAbs());//case 1
    double R = std::abs(AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) + extent1.transpose().dot(C.row(1).cwiseAbs()); // case 2
    R = std::abs(AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(2) + extent1.transpose().dot(C.row(2).cwiseAbs()); // case 3
    R = std::abs(AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(0) + extent0.transpose().dot(C.col(0).cwiseAbs()); // case 4
    R = std::abs(AxisBox1.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(1) + extent0.transpose().dot(C.col(1).cwiseAbs()); // case 5
    R = std::abs(AxisBox1.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent1(2) + extent0.transpose().dot(C.col(2).cwiseAbs()); // case 6
    R = std::abs(AxisBox1.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1)*std::abs(C(2,0)) + extent0(2)*std::abs(C(1,0))
        + extent1(1)*std::abs(C(0,2))+ extent1(2)*std::abs(C(0,1)); //case 7

    R = std::abs(C(1, 0) * AxisBox0.col(2).dot(D) - C(2, 0) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) * std::abs(C(2, 1)) + extent0(2) * std::abs(C(1, 1))
        + extent1(0) * std::abs(C(0, 2)) + extent1(2) * std::abs(C(0, 0));//case 8

    R = std::abs(C(1, 1) * AxisBox0.col(2).dot(D) - C(2, 1) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(1) * std::abs(C(2, 2)) + extent0(2) * std::abs(C(1, 2))
        + extent1(0) * std::abs(C(0, 1)) + extent1(1) * std::abs(C(0, 0));//case 9

    R = std::abs(C(1, 2) * AxisBox0.col(2).dot(D) - C(2, 2) * AxisBox0.col(1).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 0)) + extent0(2) * std::abs(C(0, 0))
        + extent1(1) * std::abs(C(1, 2)) + extent1(2) * std::abs(C(1, 1));//case 10

    R = std::abs(C(2, 0) * AxisBox0.col(0).dot(D) - C(0, 0) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 1)) + extent0(2) * std::abs(C(0, 1))
        + extent1(0) * std::abs(C(1, 2)) + extent1(2) * std::abs(C(1, 0));//case 11

    R = std::abs(C(2, 1) * AxisBox0.col(0).dot(D) - C(0, 1) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(2, 2)) + extent0(2) * std::abs(C(0, 2))
        + extent1(0) * std::abs(C(1, 1)) + extent1(1) * std::abs(C(1, 0));//case 12

    R = std::abs(C(2, 2) * AxisBox0.col(0).dot(D) - C(0, 2) * AxisBox0.col(2).dot(D));
    if (l < R) {
        return false;
    }


    l = extent0(0) * std::abs(C(1, 0)) + extent0(1) * std::abs(C(0, 0))
        + extent1(1) * std::abs(C(2, 2)) + extent1(2) * std::abs(C(2, 1));//case 13

    R = std::abs(C(0, 0) * AxisBox0.col(1).dot(D) - C(1, 0) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(1, 1)) + extent0(1) * std::abs(C(0, 1))
        + extent1(0) * std::abs(C(2, 2)) + extent1(2) * std::abs(C(2, 0));//case 14

    R = std::abs(C(0, 1) * AxisBox0.col(1).dot(D) - C(1, 1) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }

    l = extent0(0) * std::abs(C(1, 2)) + extent0(1) * std::abs(C(0, 2))
        + extent1(0) * std::abs(C(2, 1)) + extent1(1) * std::abs(C(2, 0));//case 15

    R = std::abs(C(0, 2) * AxisBox0.col(1).dot(D) - C(1, 2) * AxisBox0.col(0).dot(D));
    if (l < R) {
        return false;
    }
    

    return true;

    

}
bool SandBox::isCollide(igl::AABB<Eigen::MatrixXd, 3>* t1, igl::AABB<Eigen::MatrixXd, 3>* t2) {
    //std::cout << "check collide" << std::endl;
    if (!t1 || !t2) {
        return false;
    }
if (isIntersect(t1->m_box, t2->m_box)) {
    if (!t1->m_left && !t1->m_right && !t2->m_left && !t2->m_right) {
        draw_obb(0, t1->m_box, Eigen::RowVector3d(1, 0, 0));
        draw_obb(1, t2->m_box, Eigen::RowVector3d(1, 0, 0));
        return true;
    }
    return isCollide(t1->m_left, t2->m_right) || isCollide(t1->m_right, t2->m_left) ||
        isCollide(t1->m_right, t2->m_right) || isCollide(t1->m_left, t2->m_left);
}
else {
    // std::cout << "update starting point" << std::endl;
    // starting_point[0] = t1;
   //  starting_point[1] = t2;
    return false;
}
}



void SandBox::draw_obb(int idx, Eigen::AlignedBox<double, 3> box, Eigen::RowVector3d color) {


    Eigen::Vector3d BottomLeftFloor = box.corner(Eigen::AlignedBox3d::BottomLeftFloor);
    Eigen::Vector3d BottomRightFloor = box.corner(Eigen::AlignedBox3d::BottomRightFloor);
    Eigen::Vector3d TopLeftFloor = box.corner(Eigen::AlignedBox3d::TopLeftFloor);
    Eigen::Vector3d TopRightFloor = box.corner(Eigen::AlignedBox3d::TopRightFloor);
    Eigen::Vector3d BottomLeftCeil = box.corner(Eigen::AlignedBox3d::BottomLeftCeil);
    Eigen::Vector3d BottomRightCeil = box.corner(Eigen::AlignedBox3d::BottomRightCeil);
    Eigen::Vector3d TopLeftCeil = box.corner(Eigen::AlignedBox3d::TopLeftCeil);
    Eigen::Vector3d TopRightCeil = box.corner(Eigen::AlignedBox3d::TopRightCeil);
    Eigen::MatrixXd V_box(8, 3);
    V_box <<
        BottomLeftFloor(0), BottomLeftFloor(1), BottomLeftFloor(2),
        BottomRightFloor(0), BottomRightFloor(1), BottomRightFloor(2),
        TopLeftFloor(0), TopLeftFloor(1), TopLeftFloor(2),
        TopRightFloor(0), TopRightFloor(1), TopRightFloor(2),
        BottomLeftCeil(0), BottomLeftCeil(1), BottomLeftCeil(2),
        BottomRightCeil(0), BottomRightCeil(1), BottomRightCeil(2),
        TopLeftCeil(0), TopLeftCeil(1), TopLeftCeil(2),
        TopRightCeil(0), TopRightCeil(1), TopRightCeil(2);
    Eigen::MatrixXi E_box(12, 2);
    E_box <<
        0, 1,
        0, 2,
        0, 4,
        1, 5,
        1, 3,
        2, 3,
        2, 6,
        3, 7,
        4, 5,
        4, 6,
        5, 7,
        6, 7;
    // Plot the corners of the bounding box as points
    data_list[idx].add_points(V_box, color);

    // Plot the edges of the bounding box
    for (unsigned i = 0; i < E_box.rows(); ++i)
        data_list[idx].add_edges
        (
            V_box.row(E_box(i, 0)),
            V_box.row(E_box(i, 1)),
            color
        );




}


SandBox::~SandBox()
{

}

void SandBox::change_direction(Eigen::Vector3d d)
{
    int idx = mesh_index(data().id);
    free(direction[idx]);
    direction[idx] = new Eigen::Vector3d(d);
}

void SandBox::change_snake_direction(Eigen::Vector3d d) {
    Eigen::Quaterniond rot;
    Eigen::Vector3d relativeD = target.GetRotation() * d;
    Eigen::Vector3d curD = target.GetRotation() * Eigen::Vector3d(0, 0, 1);
    rot = Eigen::Quaterniond::FromTwoVectors(curD, relativeD);
    rot = rot.slerp(0.90, Eigen::Quaterniond::Identity());
    target.MyRotate(rot);


}

int SandBox::getLevel() {
    return level;
}

int SandBox::getScore() {
    return Points;
}

/*void SandBox::change_snake_direction(Eigen::Vector3d d) {
    if (AutoMode) {
        return;
    }
    RotationList vQ,vQ_tmp;
    std::vector<Eigen::Vector3d> vT,vT_tmp;
    Eigen::Quaterniond rot, localRot;
    
    Eigen::Vector3d target_direction = ( Joints[HEAD_IDX].GetRotation() * d).normalized();
    target = target_direction;
    Eigen::Vector3d current_direction = (Joints[HEAD_IDX].GetRotation()*Eigen::Vector3d(0,0,1)).normalized();

    rot = Eigen::Quaterniond::FromTwoVectors(current_direction, target_direction);
    rot = rot.slerp(0.98, Eigen::Quaterniond::Identity());
    vQ_tmp.push_back(rot);
    vT_tmp.push_back(Eigen::Vector3d(0, 0, 0));
    Joints[HEAD_IDX].MyRotate(rot);


    
    for (int i = HEAD_IDX - 1; i >= 0 ; i--) {


            Eigen::Vector3d next = (Joints[i+1].GetRotation() * Eigen::Vector3d(0, 0, 1)).normalized();
            Eigen::Vector3d current = (Joints[i].GetRotation() * Eigen::Vector3d(0, 0, 1)).normalized();



            localRot = Eigen::Quaterniond::FromTwoVectors(current, next);
            localRot = localRot.slerp(0.9, Eigen::Quaterniond::Identity());

            vQ_tmp.push_back(localRot);
            //vQ_tmp.push_back(Eigen::Quaterniond::Identity());
            Joints[i].MyRotate(localRot);

        
       

        vT_tmp.push_back(Eigen::Vector3d(0, 0, 0));
    }

    for (int i = vQ_tmp.size() - 1; i >= 0; i--) {
        vQ.push_back(vQ_tmp[i]);
        vT.push_back(vT_tmp[i]);
    }

    igl::dqs(data_list[0].V, M, vQ, vT, U); //original was W instead of M

    data_list[0].set_vertices(U);
    data_list[0].compute_normals();

        

}*/




void SandBox::Animate()
{
    if (isActive)
    {
        if (levelUp) {
            LevelUp(++level);
        }
        if (CatchTarget()) {
            Points++;
            std::cout << "Points = " << Points << std::endl;

        }
        MoveSnake();
        if (FollowMe) {
            FollowWithCamera(true);
        }

    }


}

//assignment 3
/*void SandBox::Animate()
{
    if (isActive)
    {

        dest = (data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
        num = data_list.size();
        armLen = (num - 1) * link_len;
        Eigen::Vector3d armBase = *getTip(0);
        double distance = (*getTip(num - 1) - dest).norm();
        double distToTarget = (armBase - dest).norm();
        if (distToTarget > armLen) {
            std::cout << "Unreachable." << std::endl;
            isActive = false;
        }
        else if (distance > tol){

            //ccd();
            Fabrik();
            //isActive = false;
        }
        else {
            double prev = 0;
            for (int i = 1; i < num; i++) {

                Eigen::Vector3d ea = data_list[i].GetRotation().eulerAngles(2, 0, 2);
 
                std::cout << -ea[2] << std::endl;
                data_list[i].RotateInSystem(Eigen::Vector3d(0, 0, 1), prev);
                data_list[i].MyRotate( Eigen::Vector3d(0, 0, 1), -ea[2]);
                
                
                prev = ea[2];
            }
            std::cout << "Distance = " << distance << std::endl;
            isActive = false;
            
        }


		
		
		
	}
}*/

//assignment 2
/*void SandBox::Animate()
{
    if (isActive)
    {
         int idx = mesh_index(data().id);
          if (isCollide(starting_point[0], starting_point[1])) {
         
              std::cout << "collision" << std::endl;
              isActive = false;
          }
          else{
                  data().MyTranslate(*direction[idx], true);
          }
    }
}*/


