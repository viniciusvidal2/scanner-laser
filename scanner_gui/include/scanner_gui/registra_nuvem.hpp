#ifndef REGISTRA_NUVEM_H
#define REGISTRA_NUVEM_H

#endif // REGISTRA_NUVEM_H

#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/geometry/mesh_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ctime>

#include <string>
#include <iostream>
#include <istream>
#include <ostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>
#include <qobject.h>
#include <QThread>
#include <QMutex>
#include <QString>
#include <boost/filesystem.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "projetalaser.hpp"

namespace scanner_gui{

using namespace pcl;
using namespace pcl::io;
using namespace pcl::geometry;
using namespace cv;
using namespace std;

class RegistraNuvem : public QThread
{
    Q_OBJECT
public:
    /// Definicoes ///
    typedef PointNormal PointT;
    typedef PointXYZRGBNormal PointF;

    RegistraNuvem(int argc, char **argv);
    virtual ~RegistraNuvem();
    void init();

    void set_inicio_processo(bool inicio);

    void set_nuvem_fonte(QString nome);
    void set_nuvem_alvo(QString nome);
    void set_arquivo_cameras_fonte(QString nome);
    void set_arquivo_cameras_alvo(QString nome);

    void set_translacao(float tx, float ty, float tz);
    void set_rotacao(float rx, float ry, float rz);

    void set_profundidade_icp(double p);

    void publicar_nuvens();    
    void registrar_nuvens(bool icp_flag);
    void salvar_dados_finais(QString pasta);

    void get_TFinal(float &x, float &y, float &z, float &rx, float &ry, float &rz);

    /// Para filtragem da nuvem ///
    void set_nuvem_filtrar(QString n);
    void set_new_voxel(float v);
    void set_new_outlier(float k, float stddev);
    void salvar_nuvem_filtrada(QString nome);
    void reseta_filtros();
    void aplica_filtro_polinomio(int grau, float raio);

private:
    /// Variaveis ///
    // Inicio da classe
    int init_argc;
    char** init_argv;
    // Flag para comecar a carregar nuvens e processar
    bool processando;
    // Nuvens lidas dos arquivos
    PointCloud<PointT>::Ptr src;
    PointCloud<PointT>::Ptr src_temp;
    PointCloud<PointT>::Ptr tgt;
    PointCloud<PointT>::Ptr acumulada;
    // Nomes das pastas e arquivos
    std::string pasta_src, pasta_tgt;
    std::string arquivo_src;
    std::string arquivo_tgt;
    std::string arquivo_cameras_alvo;
    std::string arquivo_cameras_fonte;
    // Publicadores ROS
    ros::Publisher pub_srctemp;
    ros::Publisher pub_tgt;
    ros::Publisher pub_acumulada;
    // Transformacao das nuvens
    Eigen::Matrix4f T_fim;
    Eigen::Vector3f t_fim;
    Eigen::Matrix3f R_fim;
    // Centroide calculado pelas medias dos pontos das nuvens
    Eigen::Vector3f centroide_src, centroide_tgt;
    // MUTEX para nao publicar enquanto nao transformar a nuvem
    bool mutex_publicar; // True: pode publicar; False: nao pode, esta processando
    // Profundidade do icp para filtragem de voxel
    double profundidade_icp;
    /// aba3 ///
    PointCloud<PointF>::Ptr nuvem_filtrar;
    PointCloud<PointF>::Ptr nuvem_filtrar_temp;
    ros::Publisher pub_filtrada;
    std::string pasta_filtrada;
    std::string nome_nuvem_filtrada;
    bool aba3;

    // Projetar o laser ao final
    ProjetaLaser* pl;

    /// Metodos ///
    void criaMatriz();
    void filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size);
    void filter_grid(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float leaf_size);
    void filter_grid(PointCloud<PointF>::Ptr in, PointCloud<PointF>::Ptr out, float leaf_size);
    Eigen::Matrix4f icp(PointCloud<PointT>::Ptr src, PointCloud<PointT>::Ptr tgt, Eigen::Matrix4f T);
    Eigen::Vector3f calcula_centroide(PointCloud<PointT>::Ptr cloud);
    Eigen::Vector3f calcula_centroide(PointCloud<PointF>::Ptr cloud);
    void remove_outlier(PointCloud<PointF>::Ptr in, PointCloud<PointF>::Ptr out, float mean, float deviation);
    void color_cloud_depth();
    float normaldist(float x, float media, float dev);

};

}
