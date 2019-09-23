#include "../include/scanner_gui/registra_nuvem.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <string>
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

#include <boost/lexical_cast.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace scanner_gui {
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::RegistraNuvem(int argc, char** argv):init_argc(argc), init_argv(argv){
    QFuture<void> future = QtConcurrent::run(this, &RegistraNuvem::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::~RegistraNuvem(){
    if(ros::isStarted())
        ros::shutdown();
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::init(){
    ros::init(init_argc, init_argv, "RegistraNuvem");
    if ( !ros::master::check() )  {
        cout << "check ros master not good" << endl;
        return;
    }
    ros::start();
    ros::NodeHandle nh_;

    // Inicia as nuvens
    src       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    src_temp  = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    tgt       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();

    // Inicia a transformaçao
    T_fim = Eigen::Matrix4f::Identity();
    R_fim = Eigen::Matrix3f::Identity();
    t_fim << 0.0, 0.0, 0.0;

    // Publicadores (a principio)
    pub_srctemp   = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_fonte_temp"    , 1);
    pub_tgt       = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_alvo_temp"     , 1);
    pub_acumulada = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_acumulada_temp", 1);

    // Mutex de publicacao
    mutex_publicar = true;

    ros::Rate rate(0.5);
    while(ros::ok()){
        rate.sleep();
        publicar_nuvens();
    }

    ros::shutdown();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_inicio_processo(bool inicio){
    processando = inicio;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::criaMatriz(){
    T_fim << R_fim, t_fim,
             0, 0, 0, 1;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::publicar_nuvens(){
    sensor_msgs::PointCloud2 src_temp_msg, tgt_msg, acumulada_msg;
    std::string frame = "map";
//    if(true){
    if(processando && mutex_publicar){
        if(src_temp->size() > 0){
            toROSMsg(*src_temp, src_temp_msg);
            src_temp_msg.header.frame_id = frame;
            pub_srctemp.publish(src_temp_msg);
        }
        if(tgt->size() > 0){
            toROSMsg(*tgt, tgt_msg);
            tgt_msg.header.frame_id = frame;
            pub_tgt.publish(tgt_msg);
        }
        if(acumulada->size() > 0){
            toROSMsg(*acumulada, acumulada_msg);
            acumulada_msg.header.frame_id = frame;
            pub_acumulada.publish(acumulada_msg);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_alvo(QString nome){
    mutex_publicar = false;
    arquivo_tgt = nome.toStdString();
    loadPLYFile(arquivo_tgt, *tgt);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    std::string nome_arquivo_sozinho;
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_tgt = nome.left(i).toStdString();        
        nome_arquivo_sozinho = nome.right(nome.length()-i-1).toStdString();
        break;
      }
    }
    // Calcular centroide para deslocar nuvem
    ROS_INFO("Deslocando nuvem alvo para o centro do frame...");
    ROS_INFO("Tamanho da nuvem: %zu", tgt->size());
    centroide_tgt = this->calcula_centroide(tgt);
    transformPointCloud(*tgt, *tgt,  -centroide_tgt, Eigen::Quaternion<float>::Identity());
    ROS_INFO("Nuvem alvo deslocada.");
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_fonte(QString nome){
    mutex_publicar = false;
    arquivo_src = nome.toStdString();
    loadPLYFile(arquivo_src, *src);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    std::string nome_arquivo_sozinho;
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_src = nome.left(i).toStdString();
        nome_arquivo_sozinho = nome.right(nome.length()-i-1).toStdString();
        break;
      }
    }
    // Calcular centroide para deslocar nuvem
    ROS_INFO("Deslocando nuvem fonte para o centro do frame...");
    ROS_INFO("Tamanho da nuvem: %zu", src->size());
    centroide_src = this->calcula_centroide(src);
    transformPointCloud(*src, *src,  -centroide_src, Eigen::Quaternion<float>::Identity());
    ROS_INFO("Nuvem fonte deslocada.");
    // A principio as nuvens sao iguais, depois havera modificacao
    *src_temp = *src;
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_translacao(float tx, float ty, float tz){
    mutex_publicar = false;
    // Recebe em Centimetros, converte pra METROS
    t_fim << tx/100.0, ty/100.0, tz/100.0;
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T_fim);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_rotacao(float rx, float ry, float rz){
    mutex_publicar = false;
    // Recebe em GRAUS, converte pra radianos
    rx = deg2rad(rx);
    ry = deg2rad(ry);
    rz = deg2rad(rz);

    R_fim = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T_fim);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::registrar_nuvens(bool icp_flag){
    mutex_publicar = false;
    // Se vamos usar o ICP ou nao, decide aqui
    if(icp_flag){

        // Recebe a matriz de transformacao final do ICP
        Eigen::Matrix4f Ticp = icp(src, tgt, T_fim);
        // Transforma de forma fina para a src_temp, para nao perder a src
        transformPointCloudWithNormals(*src, *src_temp, Ticp);
        *acumulada = *tgt + *src_temp;
        // Guarda para escrever no arquivo de cameras
        T_fim = Ticp;

    } else {

        *acumulada = *tgt + *src_temp;
        criaMatriz();

    }
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_grid(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(in);
    grid.filter(*out);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::get_TFinal(float &x, float &y, float &z, float &rx, float &ry, float &rz){
    x = 100*T_fim(0, 3); y = 100*T_fim(1, 3); z = 100*T_fim(2, 3); // em centimetros aqui

    Eigen::Matrix3f rot;
    rot = T_fim.block(0, 0, 3, 3);
    Eigen::Vector3f rpy;
    rpy = rot.eulerAngles(0, 1, 2);

    rx = rad2deg(rpy[0]); ry = rad2deg(rpy[1]); rz = rad2deg(rpy[2]);
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f RegistraNuvem::icp(const PointCloud<PointT>::Ptr src,
                                   const PointCloud<PointT>::Ptr tgt,
                                   Eigen::Matrix4f T){
    ROS_INFO("Entrando no ICP");
    // Reduzindo complexidade das nuvens
    PointCloud<PointT>::Ptr temp_src (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr temp_tgt (new PointCloud<PointT>());

    *temp_src = *src; *temp_tgt = *tgt;

    float leaf_size;
    if(profundidade_icp == 0){
        leaf_size = 0.01;
    } else {
        leaf_size = profundidade_icp;
    }
//    filter_grid(temp_src, leaf_size);
//    filter_grid(temp_tgt, leaf_size);

    Eigen::Matrix4f T_icp = T;

    /// ICP COMUM ///
    // Criando o otimizador de ICP comum
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
    rej_med->setMedianFactor (3.0);
    icp.addCorrespondenceRejector (rej_med);
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej_samp (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>);
    icp.addCorrespondenceRejector (rej_samp);
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputTarget(temp_tgt);
    icp.setInputSource(temp_src);
    icp.setMaximumIterations(200); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-12);
    icp.setEuclideanFitnessEpsilon(1*1e-13);
    icp.setMaxCorrespondenceDistance(0.06);

    #pragma omp parallel
    PointCloud<PointT> final2;
    icp.align(final2, T);

    if(icp.hasConverged())
        T_icp = icp.getFinalTransformation();

    temp_src->clear(); temp_tgt->clear();

    ROS_INFO("ICP realizado.");

    return T_icp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::salvar_dados_finais(QString pasta){
    // Nova pasta no Desktop
    char* home;
    home = getenv("HOME");
    std::string pasta_final = std::string(home)+"/Desktop/"+pasta.toStdString();

    // Criar nova pasta na area de trabalho
    const int dir_err = mkdir(pasta_final.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if( !(dir_err == -1) ){ // Nao houve erro na criacao do diretorio
        // Gravar a nuvem registrada no diretorio final, junto com a nuvem fonte e a alvo deslocadas
        std::string arquivo_nuvem_final = pasta_final + "/nuvem_final.ply";
        std::string arquivo_fonte_final = pasta_final + "/nuvem_fonte.ply";
        std::string arquivo_alvo_final  = pasta_final + "/nuvem_alvo.ply" ;
        savePLYFileASCII(arquivo_fonte_final, *src_temp );
        savePLYFileASCII(arquivo_alvo_final , *tgt      );
        savePLYFileASCII(arquivo_nuvem_final, *acumulada);

        ROS_INFO("Arquivos criados com sucesso.");

    } // Fim do if para mkdir
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f RegistraNuvem::calcula_centroide(PointCloud<PointT>::Ptr cloud){
    float x = 0, y = 0, z = 0;
    for(unsigned long i=0; i < cloud->size(); i++){
        x = x + cloud->points[i].x;
        y = y + cloud->points[i].y;
        z = z + cloud->points[i].z;
    }
    Eigen::Vector3f centroide;
    centroide << x/(float)cloud->size(), y/(float)cloud->size(), z/(float)cloud->size();
    // Retorna centro medio dos pontos da nuvem, na transformacao deve ser passado negativo
    return centroide;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::remove_outlier(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*out);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_profundidade_icp(double p){
    profundidade_icp = p;
}
///////////////////////////////////////////////////////////////////////////////////////////
} // fim do namespace scanner_gui