#include "../include/scanner_gui/projetalaser.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

///////////////////////////////////////////////////////////////////////////////////////////
ProjetaLaser::ProjetaLaser(Eigen::Matrix3f K_, Eigen::Matrix4f Tla, std::string p):K(K_), T_eixos(Tla), pasta(p)
{
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
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::set_capture_angles(std::vector<float> in){
    angulos = in;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::set_cloud_vector(std::vector<PointCloud<PointT> > clouds){
    nuvens = clouds;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::process(int indice){
    if(nuvens[indice].size() > 10){
        // Nuvem temporaria ponteiro para ser processada
        PointCloud<PointT>::Ptr nuvem (new PointCloud<PointT>());
        *nuvem = nuvens[indice];
        // Cria imagem da camera virtual no momento atual de captura
        cv::Mat foto_virtual(cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0));
        // Colorir a nuvem segundo distancia dos pontos
        color_cloud_depth(nuvem);
        // Criar matriz de transformacao da nuvem segundo angulo de captura
        Eigen::Matrix4f T = calculate_transformation(angulos[indice]);
        // Transformar nuvem para o frame da camera: matriz anterior e rodar eixos
        transformPointCloud(*nuvem, *nuvem, T);
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
            }
        }
        // Salvar foto final para analisar
        std::string nome = pasta+"camera_virtual_"+std::to_string(indice)+".png";
        std::vector<int> opcoes;
        opcoes.push_back(cv::IMWRITE_PNG_COMPRESSION);
        cv::imwrite(nome, foto_virtual, opcoes);
        ROS_INFO("Depois de salvar imagem na pasta certa");
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f ProjetaLaser::calculate_transformation(float thetay_deg){
    // Constroi matriz de rotacao
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(                  0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(DEG2RAD(thetay_deg), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(                  0, Eigen::Vector3f::UnitZ());
    // Constroi matriz homgenea e retorna
    Eigen::MatrixXf t(3, 1);
    t << 0.0148,
              0,
              0;
    Eigen::Matrix4f Trot;
    Trot << R, R*t,
            0, 0, 0, 1;

    return Trot*T_eixos;
}
///////////////////////////////////////////////////////////////////////////////////////////
void ProjetaLaser::color_cloud_depth(PointCloud<PointT>::Ptr cloud){
    // Varre nuvem atras da maior e menor distancia
    float mindist = 1000, maxdist = 0, dist, scale, dev = 300;
    float r, g, b;
    float alpha = 250.0 / normaldist(0, 0, dev);
    for(unsigned long i=0; i < cloud->size(); i++){
        dist = sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z);
        if(dist > maxdist)
            maxdist = dist;
        if(dist < mindist)
            mindist = dist;
    }

    // Calcula distancias de cada ponto, regra sobre a distancia e atribui cor
    #pragma omp parallel for num_threads(10)
    for(unsigned long i=0; i < cloud->size(); i++){
        dist = sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z);
        scale = 750 * (dist - mindist)/(maxdist - mindist);
        // Pegar a cor como funcao normal
        r = alpha*normaldist(scale, 0, dev); g = alpha*normaldist(scale, 390, dev); b = alpha*normaldist(scale, 750, dev);

        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
float ProjetaLaser::normaldist(float x, float media, float dev){
    return exp( -0.5*((x - media)/dev)*((x - media)/dev) ) / sqrt( 2*M_PI*dev*dev );
}
///////////////////////////////////////////////////////////////////////////////////////////
