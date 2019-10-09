#include "../include/scanner_gui/projetalaser.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

///////////////////////////////////////////////////////////////////////////////////////////
ProjetaLaser::ProjetaLaser(int argc, char **argv):init_argc(argc), init_argv(argv)
{
//    this->init();
    QFuture<void> future = QtConcurrent::run(this, &ProjetaLaser::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
ProjetaLaser::~ProjetaLaser(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::init(){
    // Iniciando os angulos por desencargo
    angulo_min = 0; angulo_max = 60; angulo_central = 30;
    // Iniciando nuvem de pontos
    nuvem = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    // Iniciando matriz de transformação fixa referente a rotaçao de eixos - CAPRICHAR AINDA
    // Eixos LASER -> ASTRA
    ROS_INFO("antes da identidade do eigen");
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ());
    T_eixos = Eigen::Matrix4f::Identity();
    T_eixos.block<3,3>(0, 0) << matrix;
    // Iniciando matriz de projecao semelhante a da camera astra, teoricamente cai na mesma resolucao
    K << 525.1389,    1.4908,  324.1741,
                0,  521.6805,  244.8827,
                0,         0,    1.0000;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::set_angulos(float min, float max){
    angulo_min = min; angulo_max = max;
    angulo_central = (max + min)/2;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::set_nuvem(PointCloud<PointT>::Ptr cloud){
    *nuvem = *cloud;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::process(){
    // Cria imagem da camera virtual no momento atual de captura
    cv::Mat foto_virtual(cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0));
    // Criar matriz de transformacao da nuvem segundo angulo de captura
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(                      0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(DEG2RAD(angulo_central), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(                      0, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f T_roty = Eigen::Matrix4f::Identity();
    T_roty.block<3,3>(0,0) << R.inverse();
    // Transformar nuvem para o frame da camera: matriz anterior e rodar eixos
    transformPointCloud(*nuvem, *nuvem, T_roty*T_eixos);
    pcl::io::savePLYFileASCII("/home/vinicius/Desktop/testando.ply", *nuvem);
    // Projetar os pontos na foto virtual e colorir imagem
    #pragma omp parallel for num_threads(10)
    for(size_t i = 0; i < nuvem->size(); i++){
        /// Pegar ponto em coordenadas normais
        Eigen::MatrixXf X_(3, 1);
        X_ << nuvem->points[i].x,
              nuvem->points[i].y,
              nuvem->points[i].z;
        Eigen::MatrixXf X = K*X_;
        X = X/X(2, 0);
        /// Adicionando ponto na imagem se for o caso de projetado corretamente
        if(floor(X(0,0)) >= 0 && floor(X(0,0)) < foto_virtual.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < foto_virtual.rows){
            cv::Vec3b cor;
            cor[0] = nuvem->points[i].b; cor[1] = nuvem->points[i].g; cor[2] = nuvem->points[i].r;
            foto_virtual.at<cv::Vec3b>(cv::Point(X(0,0), X(1,0))) = cor;
//            ROS_INFO("projetou alguem, com cores %d   %d   %d", cor[0], cor[1], cor[2]);
        }
    }
    // Salvar foto final para analise
//    ROS_INFO(" NAO GRAVAMOS IMAGEM");
    char* home;
    home = getenv("HOME");
    std::string nome = std::string(home)+"/Desktop/camera_virtual.jpg";
    cv::imwrite(nome, foto_virtual);
//    ROS_INFO("GRAVAMOS IMAGEM");
}
///////////////////////////////////////////////////////////////////////////////////////////
