//
// Created by unimage on 2026/6/27.
//

#include "../Inc/ESKF.h"                    //dv
#include <Eigen/Dense>                      //dv

using namespace Eigen;                      //dv

typedef struct {                            //dv
    Vector3f acc_meas;                      //dv
    Vector3f omega_meas;                    //dv
}imudata_t;                                 //dv

typedef struct {                            //dv
    Vector3f gps_pos;                       //dv
    Vector3f gps_vel;                       //dv
    Vector3f mag_b;                         //dv
    float    baro_z;                        //dv
}y_t;                                       //dv

void eskf(y_t* y_meas,imudata_t* imu_data);

Matrix3f skew(Vector3f vec);



typedef struct {                    //dv
    Matrix3f P11;                   //dv
    Matrix3f P12;
    Matrix3f P13;
    Matrix3f P14;
    Matrix3f P15;
    Vector3f P16;

    Matrix3f P21;
    Matrix3f P22;
    Matrix3f P23;
    Matrix3f P24;
    Matrix3f P25;
    Vector3f P26;

    Matrix3f P31;
    Matrix3f P32;
    Matrix3f P33;
    Matrix3f P34;
    Matrix3f P35;
    Vector3f P36;

    Matrix3f P41;
    Matrix3f P42;
    Matrix3f P43;
    Matrix3f P44;
    Matrix3f P45;
    Vector3f P46;

    Matrix3f P51;
    Matrix3f P52;
    Matrix3f P53;
    Matrix3f P54;
    Matrix3f P55;
    Vector3f P56;

    RowVector3f P61;
    RowVector3f P62;
    RowVector3f P63;
    RowVector3f P64;
    RowVector3f P65;
    float       P66;
} P_t;                              //dv

typedef struct {                    //dv
    Matrix3f Fx12;                  //dv
    Matrix3f Fx23;                  //dv
    Matrix3f Fx24;                  //dv
    Matrix3f Fx33;                  //dv
    Matrix3f Fx35;                  //dv
}Fx_t;                              //dv

typedef struct {                    //dv
    Matrix3f Qi11;                  //dv
    Matrix3f Qi22;                  //dv
    Matrix3f Qi33;                  //dv
    Matrix3f Qi44;                  //dv
    float    Qi55;                  //dv
}Qi_t;                              //dv

typedef struct {                    //dv
    Vector3f pos;                   //dv
    Vector3f vel;                   //dv
    Quaternionf quat;               //dv
    Vector3f a_b;                   //dv
    Vector3f w_b;                   //dv
    float    z_b;                   //dv
}x_nom_t;                           //dv

typedef struct {                    //dv
    Vector3f delta_pos;             //dv
    Vector3f delta_vel;             //dv
    Vector3f delta_theta;           //dv
    Vector3f delta_a_b;             //dv
    Vector3f delta_w_b;             //dv
    float    delta_z_b;             //dv
}x_err_t;                           //dv

float Ts = 0.005;                   //dv

Vector3f g = {0,0,9.80665};   //dv
Vector3f B_const = {1,0,0};   //？？？？？？

void eskf(float* meas,float* imu) {
    //
    y_t y_meas;
    y_meas.gps_pos = {meas[0],meas[1],meas[2]};
    y_meas.gps_vel = {meas[3],meas[4],meas[5]};
    y_meas.mag_b = {meas[6],meas[7],meas[8]};
    y_meas.baro_z = meas[9];

    imudata_t imu_data;
    imu_data.acc_meas = {imu[0],imu[1],imu[2]};
    imu_data.omega_meas = {imu[3],imu[4],imu[5]};

    // 定义名义状态并进行初始化                          //dv
    static x_nom_t x_nom = []() {          //dv
        x_nom_t tmp;                                //dv
        memset(&tmp, 0, sizeof(tmp));               //dv
        tmp.quat = Quaternionf::Identity();         //dv
        return tmp;                                 //dv
    }();

    //名义状态积分                                    //dv
    x_nom_t x_nom_pred;                             //dv
    x_nom_pred.pos = x_nom.pos + Ts*x_nom.vel + 0.5f*Ts*Ts*(x_nom.quat.toRotationMatrix()*(imu_data.acc_meas - x_nom.a_b)+g);      //dv
    x_nom_pred.vel = x_nom.vel + Ts*(x_nom.quat.toRotationMatrix()*(imu_data.acc_meas - x_nom.a_b)+g);                             //dv

    Quaternionf delta_q;                                        //dv
    delta_q.w() = 1;                                            //dv
    delta_q.vec() = (imu_data.omega_meas - x_nom.w_b)*Ts*0.5;   //dv
    x_nom_pred.quat = x_nom.quat*delta_q;                       //dv
    x_nom_pred.quat = x_nom_pred.quat.normalized();             //dv
    x_nom_pred.a_b = x_nom.a_b;                                 //dv
    x_nom_pred.w_b = x_nom.w_b;                                 //dv
    x_nom_pred.z_b = x_nom.z_b;                                 //dv

    // 定义协方差矩阵并初始化                           //dv
    static P_t P = []() {                      //dv
        P_t mat;                                    //dv
        memset(&mat, 0, sizeof(mat));               //dv
        mat.P11 = Matrix3f::Identity();             //dv
        mat.P22 = Matrix3f::Identity();             //dv
        mat.P33 = Matrix3f::Identity();             //dv
        mat.P44 = Matrix3f::Identity();             //dv
        mat.P55 = Matrix3f::Identity();             //dv
        mat.P66 = 1;                                //dv
        return mat;                                 //dv
    }();                                            //dv

    // 定义误差状态转移矩阵并初始化                      //dv
    static Fx_t Fx = []() {                   //dv
        Fx_t mat;                                   //dv
        memset(&mat, 0, sizeof(mat));               //dv
        mat.Fx12 = Ts*Matrix3f::Identity();         //dv
        mat.Fx35 = -Ts*Matrix3f::Identity();        //dv
        return mat;                                 //dv
    }();

    // 误差状态转移矩阵赋值                                                                                           //dv
    Fx.Fx23 = -Ts*x_nom_pred.quat.toRotationMatrix()*skew(imu_data.acc_meas - x_nom_pred.a_b);            //dv
    Fx.Fx24 = -Ts*x_nom_pred.quat.toRotationMatrix();                                                             //dv
    Fx.Fx33 = Matrix3f::Identity() - Ts*skew(imu_data.omega_meas - x_nom_pred.w_b);                       //??????

    // 过程噪声矩阵初始化
    static Qi_t Qi = []() {                     //dv
        Qi_t mat;                                     //dv
        memset(&mat, 0, sizeof(mat));                 //dv
        mat.Qi11 = 0.01*Matrix3f::Identity();         //dv
        mat.Qi22 = 0.01*Matrix3f::Identity();         //dv
        mat.Qi33 = 0.01*Matrix3f::Identity();         //dv
        mat.Qi44 = 0.01*Matrix3f::Identity();         //dv
        mat.Qi55 = 0.01;                              //dv
        return mat;                                   //dv
    }();                                              //dv

    // 协方差矩阵预测值临时变量                            //dv
    P_t P_pred;                                       //dv

    // 协方差矩阵P预测
    P_pred.P11 = (Fx.Fx12*P.P22 + P.P12)*Fx.Fx12.transpose() + Fx.Fx12*P.P21 + P.P11;
    P_pred.P12 = (Fx.Fx12*P.P23 + P.P13)*Fx.Fx23.transpose() + (Fx.Fx12*P.P24 + P.P14)*Fx.Fx24.transpose() + Fx.Fx12*P.P22 + P.P12;
    P_pred.P13 = (Fx.Fx12*P.P23 + P.P13)*Fx.Fx33.transpose() + (Fx.Fx12*P.P25 + P.P15)*Fx.Fx35.transpose();
    P_pred.P14 = Fx.Fx12*P.P24 + P.P14;
    P_pred.P15 = Fx.Fx12*P.P25 + P.P15;
    P_pred.P16 = Fx.Fx12*P.P26 + P.P16;
    P_pred.P21 = (Fx.Fx23*P.P32 + Fx.Fx24*P.P42 + P.P22)*Fx.Fx12.transpose() + Fx.Fx23*P.P31 + Fx.Fx24*P.P41 + P.P21;
    P_pred.P22 = (Fx.Fx23*P.P33 + Fx.Fx24*P.P43 + P.P23)*Fx.Fx23.transpose() + (Fx.Fx23*P.P34 + Fx.Fx24*P.P44 + P.P24)*Fx.Fx24.transpose() + Fx.Fx23*P.P32 + Fx.Fx24*P.P42 + P.P22 + Qi.Qi11;
    P_pred.P23 = (Fx.Fx23*P.P33 + Fx.Fx24*P.P43 + P.P23)*Fx.Fx33.transpose() + (Fx.Fx23*P.P35 + Fx.Fx24*P.P45 + P.P25)*Fx.Fx35.transpose();
    P_pred.P24 = Fx.Fx23*P.P34 + Fx.Fx24*P.P44 + P.P24;
    P_pred.P25 = Fx.Fx23*P.P35 + Fx.Fx24*P.P45 + P.P25;
    P_pred.P26 = Fx.Fx23*P.P36 + Fx.Fx24*P.P46 + P.P26;
    P_pred.P31 = (Fx.Fx33*P.P32 + Fx.Fx35*P.P52)*Fx.Fx12.transpose() + Fx.Fx33*P.P31 + Fx.Fx35*P.P51;
    P_pred.P32 = (Fx.Fx33*P.P33 + Fx.Fx35*P.P53)*Fx.Fx23.transpose() + (Fx.Fx33*P.P34 + Fx.Fx35*P.P54)*Fx.Fx24.transpose() + Fx.Fx33*P.P32 + Fx.Fx35*P.P52;
    P_pred.P33 = (Fx.Fx33*P.P33 + Fx.Fx35*P.P53)*Fx.Fx33.transpose() + (Fx.Fx33*P.P35 + Fx.Fx35*P.P55)*Fx.Fx35.transpose() + Qi.Qi22;
    P_pred.P34 = Fx.Fx33*P.P34 + Fx.Fx35*P.P54;
    P_pred.P35 = Fx.Fx33*P.P35 + Fx.Fx35*P.P55;
    P_pred.P36 = Fx.Fx33*P.P36 + Fx.Fx35*P.P56;
    P_pred.P41 = P.P41 + P.P42*Fx.Fx12.transpose();
    P_pred.P42 = P.P42 + P.P43*Fx.Fx23.transpose() + P.P44*Fx.Fx24.transpose();
    P_pred.P43 = P.P43*Fx.Fx33.transpose() + P.P45*Fx.Fx35.transpose();
    P_pred.P44 = P.P44 + Qi.Qi33;
    P_pred.P45 = P.P45;
    P_pred.P46 = P.P46;
    P_pred.P51 = P.P51 + P.P52*Fx.Fx12.transpose();
    P_pred.P52 = P.P52 + P.P53*Fx.Fx23.transpose() + P.P54*Fx.Fx24.transpose();
    P_pred.P53 = P.P53*Fx.Fx33.transpose() + P.P55*Fx.Fx35.transpose();
    P_pred.P54 = P.P54;
    P_pred.P55 = P.P55 + Qi.Qi44;
    P_pred.P56 = P.P56;
    P_pred.P61 = P.P61 + P.P62*Fx.Fx12.transpose();
    P_pred.P62 = P.P62 + P.P63*Fx.Fx23.transpose() + P.P64*Fx.Fx24.transpose();
    P_pred.P63 = P.P63*Fx.Fx33.transpose() + P.P65*Fx.Fx35.transpose();
    P_pred.P64 = P.P64;
    P_pred.P65 = P.P65;
    P_pred.P66 = P.P66 + Qi.Qi55;


    //残差计算                                                                    //dv
    y_t y_pred;                                                                 //dv
    y_pred.gps_pos = x_nom_pred.pos;                                            //dv
    y_pred.gps_vel = x_nom_pred.vel;                                            //dv
    y_pred.mag_b = x_nom_pred.quat.conjugate().toRotationMatrix()*B_const;      //dv
    y_pred.baro_z = x_nom_pred.pos.z() + x_nom_pred.z_b;                        //dv
    Vector<float,10> innov;                                                     //dv
    innov.block<3,1>(0,0) = y_meas.gps_pos - y_pred.gps_pos;    //dv
    innov.block<3,1>(3,0) = y_meas.gps_vel - y_pred.gps_vel;    //dv
    innov.block<3,1>(6,0) = y_meas.mag_b - y_pred.mag_b;        //dv
    innov[9] = y_meas.baro_z - y_pred.baro_z;                                  //dv



    // 定义雅克比矩阵并初始化                                                         //dv
    static Matrix<float,10,16> H = []() {                       //dv
        Matrix<float,10,16> mat;                                                  //dv
        mat.setZero();                                                            //dv
        mat.block<3,3>(0,0) = Matrix3f::Identity();                //dv
        mat.block<3,3>(3,3) = Matrix3f::Identity();                //dv
        mat.block<1,3>(9,0) = RowVector3f{0,0,1};            //dv
        mat.block<1,1>(9,15) = Matrix<float,1,1>::Identity();      //dv
        return mat;                                                               //dv
    }();
    // 雅克比矩阵计算                                                                                                //dv
    H.block<3,3>(6,6) = skew(x_nom_pred.quat.toRotationMatrix().transpose()*B_const);       //dv

    // 定义卡尔曼增益矩阵                                                          //dv
    Matrix<float,16,10> K;                                                      //dv

    // 定义观测噪声协方差矩阵并初始化
    static Matrix<float,10,10> V = []() {                             //dv
        Matrix<float,10,10> mat;                                                        //dv
        memset(&mat, 0, sizeof(mat));                                                   //dv
        mat.block<3,3>(0,0) = 5*Matrix3f::Identity();                    //dv
        mat.block<3,3>(3,3) = 0.5*Matrix3f::Identity();                  //dv
        mat.block<3,3>(6,6) = 0.01*Matrix3f::Identity();                 //dv
        mat.block<1,1>(9,9) = 0.001*Matrix<float,1,1>::Identity();       //dv
        return mat;                                                                     //dv
    }();                                                                                //dv

    Matrix<float,16,16> Ppred;                                                          //dv
    Ppred.block<3,3>(0,0) = P_pred.P11;
    Ppred.block<3,3>(0,3) = P_pred.P12;
    Ppred.block<3,3>(0,6) = P_pred.P13;
    Ppred.block<3,3>(0,9) = P_pred.P14;
    Ppred.block<3,3>(0,12) = P_pred.P15;
    Ppred.block<3,1>(0,15) = P_pred.P16;
    Ppred.block<3,3>(3,0) = P_pred.P21;
    Ppred.block<3,3>(3,3) = P_pred.P22;
    Ppred.block<3,3>(3,6) = P_pred.P23;
    Ppred.block<3,3>(3,9) = P_pred.P24;
    Ppred.block<3,3>(3,12) = P_pred.P25;
    Ppred.block<3,1>(3,15) = P_pred.P26;
    Ppred.block<3,3>(6,0) = P_pred.P31;
    Ppred.block<3,3>(6,3) = P_pred.P32;
    Ppred.block<3,3>(6,6) = P_pred.P33;
    Ppred.block<3,3>(6,9) = P_pred.P34;
    Ppred.block<3,3>(6,12) = P_pred.P35;
    Ppred.block<3,1>(6,15) = P_pred.P36;
    Ppred.block<3,3>(9,0) = P_pred.P41;
    Ppred.block<3,3>(9,3) = P_pred.P42;
    Ppred.block<3,3>(9,6) = P_pred.P43;
    Ppred.block<3,3>(9,9) = P_pred.P44;
    Ppred.block<3,3>(9,12) = P_pred.P45;
    Ppred.block<3,1>(9,15) = P_pred.P46;
    Ppred.block<3,3>(12,0) = P_pred.P51;
    Ppred.block<3,3>(12,3) = P_pred.P52;
    Ppred.block<3,3>(12,6) = P_pred.P53;
    Ppred.block<3,3>(12,9) = P_pred.P54;
    Ppred.block<3,3>(12,12) = P_pred.P55;
    Ppred.block<3,1>(12,15) = P_pred.P56;
    Ppred.block<1,3>(15,0) = P_pred.P61;
    Ppred.block<1,3>(15,3) = P_pred.P62;
    Ppred.block<1,3>(15,6) = P_pred.P63;
    Ppred.block<1,3>(15,9) = P_pred.P64;
    Ppred.block<1,3>(15,12) = P_pred.P65;
    Ppred(15,15) = P_pred.P66;


    K = Ppred*H.transpose()*((H*Ppred*H.transpose() + V).inverse());                    //dv

    // 误差状态估计值计算                                                                  //dv
    Vector<float,16> x_err_est = K*innov;                                               //dv

    // 协方差矩阵修正
    Matrix<float,16,16> Pcorrect = (Matrix<float,16,16>::Identity() - K*H)*Ppred*(Matrix<float,16,16>::Identity()\
                                    - K*H).transpose() + K*V*K.transpose();             //dv

    // 协方差矩阵重置                                                                          //dv
    Matrix3f G33 = Matrix3f::Identity()-skew(0.5*x_err_est.segment<3>(6));      //dv
    P.P11 = Pcorrect.block<3,3>(0,0);
    P.P12 = Pcorrect.block<3,3>(0,3);
    P.P13 = Pcorrect.block<3,3>(0,6)*G33.transpose();
    P.P14 = Pcorrect.block<3,3>(0,9);
    P.P15 = Pcorrect.block<3,3>(0,12);
    P.P16 = Pcorrect.block<3,1>(0,15);
    P.P21 = Pcorrect.block<3,3>(3,0);
    P.P22 = Pcorrect.block<3,3>(3,3);
    P.P23 = Pcorrect.block<3,3>(3,6)*G33.transpose();
    P.P24 = Pcorrect.block<3,3>(3,9);
    P.P25 = Pcorrect.block<3,3>(3,12);
    P.P26 = Pcorrect.block<3,1>(3,15);
    P.P31 = G33*Pcorrect.block<3,3>(6,0);
    P.P32 = G33*Pcorrect.block<3,3>(6,3);
    P.P33 = G33*Pcorrect.block<3,3>(6,6)*G33.transpose();
    P.P34 = G33*Pcorrect.block<3,3>(6,9);
    P.P35 = G33*Pcorrect.block<3,3>(6,12);
    P.P36 = G33*Pcorrect.block<3,1>(6,15);
    P.P41 = Pcorrect.block<3,3>(9,0);
    P.P42 = Pcorrect.block<3,3>(9,3);
    P.P43 = Pcorrect.block<3,3>(9,6)*G33.transpose();
    P.P44 = Pcorrect.block<3,3>(9,9);
    P.P45 = Pcorrect.block<3,3>(9,12);
    P.P46 = Pcorrect.block<3,1>(9,15);
    P.P51 = Pcorrect.block<3,3>(12,0);
    P.P52 = Pcorrect.block<3,3>(12,3);
    P.P53 = Pcorrect.block<3,3>(12,6)*G33.transpose();
    P.P54 = Pcorrect.block<3,3>(12,9);
    P.P55 = Pcorrect.block<3,3>(12,12);
    P.P56 = Pcorrect.block<3,1>(12,15);
    P.P61 = Pcorrect.block<1,3>(15,0);
    P.P62 = Pcorrect.block<1,3>(15,3);
    P.P63 = Pcorrect.block<1,3>(15,6)*G33.transpose();
    P.P64 = Pcorrect.block<1,3>(15,9);
    P.P65 = Pcorrect.block<1,3>(15,12);
    P.P66 = Pcorrect(15,15);
}


Matrix3f skew(Vector3f vec)                             //dv
{
    Matrix3f mat;                                       //dv
    mat << 0,        -vec.z(),   vec.y(),               //dv
           vec.z(),     0,       -vec.x(),              //dv
           -vec.y(),  vec.x(),       0;                 //dv
    return mat;                                         //dv
}