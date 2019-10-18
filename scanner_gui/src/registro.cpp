#include "../include/scanner_gui/registro.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>


Registro::Registro()
{
        QFuture<void> future = QtConcurrent::run(this, &Registro::init);
}

Registro::~Registro()
{
        if(ros::isStarted()){
                ros::shutdown();
                ros::waitForShutdown();
        }
        wait();
}

void Registro::init()
{
        K_astra << 525.1389,    1.4908  ,  324.1741,
                      0    ,    521.6805,  244.8827,
                      0    ,       0    ,    1.0000;

        K_zed <<         0,  0,  0,      // (TODO) Colocar matriz correta de calibracao
                         0,  0,  0,
                         0,  0,  0;

        profundidade_icp = 1;           // (TODO) Colocar profundidade_icp correto
}


void Registro::process(std::string directory, std::string ext)
{
//     Contando o numero de imagens em um determinado diretorio
    int cnt_img = count_files("directory" , "ext")/2;
    //int cnt_img = 5;
    std::vector<cv::Mat> imagens_zed;
    std::vector<cv::Mat> imagens_astra;
    std::vector<PointCloud<PointXYZ>> nuvens_astra;
    std::vector<PointCloud<PointXYZ>> nuvens_laser;
    std::vector<float> angulos_captura;

    for(int i = 0; i < cnt_img; i++)
    {
        std::cout << directory + "im_zed_" + std::to_string(i) + "." + ext + "\n";
        std::cout << directory + "im_astra_" + std::to_string(i) + "." + ext + "\n";
        imagens_zed[i] = imread(directory + "im_zed_" + std::to_string(i) + "." + ext, CV_LOAD_IMAGE_COLOR);
        imagens_astra[i] = imread(directory + "im_astra_" + std::to_string(i) + "." + ext, CV_LOAD_IMAGE_COLOR);
//        imshow("Image", imagens_zed[i]);
//        cvWaitKey(10);
        loadPLYFile(directory + "nv_astra_" + std::to_string(i) + ".ply", nuvens_astra[i]);
        loadPLYFile(directory + "parcial_" + std::to_string(i+1) + ".ply", nuvens_laser[i]);
        //nuvens_astra[i] =;
        //nuvens_laser[i] =;
        //angulos_captura[i] =;
    }

    // Projetando
    run(imagens_zed, imagens_astra, nuvens_astra, nuvens_laser, angulos_captura);

}


void Registro::run(std::vector<cv::Mat> imagens_zed, std::vector<cv::Mat> imagens_astra,
                           std::vector<PointCloud<PointXYZ>> nuvens_astra, std::vector<PointCloud<PointXYZ>> nuvens_laser,
                                   std::vector<float> angulos_captura)
{
        PointCloud<PointXYZ> nuvem_i;

        for (size_t i = 0; i < angulos_captura.size(); i++)
        {

        // Passar nuvem_laser para coordenadas da câmera Astra (RGB-D)

                // Transformação: Laser Rotacionado -> Laser Inicial
                Eigen::Matrix4f T1 = transformada_laser_rot(angulos_captura[i]);

                // Transformação: Laser -> Astra
                Eigen::Matrix4f T2 = transformada_laser2astra();

                // Transformação: Astra -> ZED
                Eigen::Matrix4f T3 = transformada_astra2zed();

                // Transformação Combinada
                Eigen::Matrix4f T_comb = T1.inverse()*T2;

        // Refinando a transformada utilizando ICP
                PointCloud<PointT>::Ptr nuvens_laser_tmp (new PointCloud<PointT>());
                PointCloud<PointT>::Ptr nuvens_astra_tmp (new PointCloud<PointT>());
                //*a = nuvens_laser[i];
                //*b = nuvens_astra[i];
                copyPointCloud(nuvens_laser[i], *nuvens_laser_tmp);
                copyPointCloud(nuvens_astra[i], *nuvens_astra_tmp);
                Eigen::Matrix4f T_icp = icp(nuvens_laser_tmp, nuvens_astra_tmp, T_comb);
                //Eigen::Matrix4f T_icp = T_comb;

        // Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ASTRA)
                PointCloud<PointXYZRGB>  nuvem_i = projetar_3d_2_2d(nuvens_laser[i], imagens_astra[i], K_astra, T_icp);

        // Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ZED -  sem otimização)
                // Eigen::Matrix4f T_zed = T_comb * T3;
                // Eigen::Matrix4f T_zed = T_icp * T3;
                // PointCloud<PointXYZRGB>  nuvem_i = projetar_3d_2_2d(nuvens_laser[i], imagens_zed[i], K_zed, T_zed);

        // Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ZED com otimização por BAT)
                // (TODO) PointCloud<PointXYZRGB>  nuvem_i = projetar_3d_2_2d(nuvens_laser[i], imagens_zed[i], K_zed, T_zed_opt);

        // Acumulando nuvem
                *acc_cor = *acc_cor + nuvem_i; // Nuvem acumulada com cor
                *acc = *acc + nuvens_laser[i]; // Nuvem acumulada sem cor

        }

        // Salvando nuvens parciais e nuvem acumulada
        saw->process_color_and_save(imagens_zed, nuvens_laser, angulos_captura, acc, acc_cor);

}


std::vector<float> Registro::ler_angulos_captura(std::string directory)
{
    std::vector<float> angulos_captura;
    string string_tmp;
    ifstream infile;
    infile.open (directory + "angulos.txt");
    vector<string> result;
    while(!infile.eof())
    {
        getline(infile, string_tmp);

            stringstream ss (string_tmp);
            string item;

            while (getline (ss, item, ' '))
            {
                result.push_back (item);
            }

            angulos_captura.push_back (strtof((result.at(1)).c_str(),0)); // pega apenas a segunda posicao que 'e a posicao correspondente ao angulo de captura

    }

    infile.close();
    return angulos_captura;
}

Eigen::Matrix4f Registro::transformada_laser_rot(float theta) // adaptado de 'saveandwork.cpp'
{
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(    0,               Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(DEG2RAD(theta),      Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(    0, 			   Eigen::Vector3f::UnitZ());
        Eigen::MatrixXf t(3, 1);
        t << 0.0148,
                          0,
                          0;
        Eigen::Matrix4f T_laser_rot;
        T_laser_rot << R, R*t,
                        0, 0, 0, 1;

        return T_laser_rot;
}

Eigen::Matrix4f Registro::transformada_laser2astra() // adaptado de 'projetalaser.cpp'
{
        Eigen::Matrix3f matrix;
        matrix = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()) *
                         Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f T_laser2astra = Eigen::Matrix4f::Identity();
        T_laser2astra.block<3,3>(0, 0) << matrix;
        T_laser2astra.block<3,1>(0, 3) << 0.004, 0.105, 0;
        return T_laser2astra;
}

Eigen::Matrix4f Registro::transformada_astra2zed()
{
        Eigen::Matrix4f T_astra2zed;
        T_astra2zed << 1, 0, 0,  0.052,
                       0, 1, 0,  0.01 ,
                       0, 0, 1, -0.01 ,
                       0, 0, 0,   1   ;
        return T_astra2zed;
}




int Registro::count_files(std::string directory, std::string ext) //  Funcao para contar o numero de arquivos de uma determinada extensao (ext) em um determinado diretorio (directory)
{
        namespace fs = boost::filesystem;
        fs::path Path(directory);
        int Nb_ext = 0;
        fs::directory_iterator end_iter;

        for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
                if (iter->path().extension() == ext)
                        ++Nb_ext;

        return Nb_ext;
}


PointCloud<PointXYZRGB> Registro::projetar_3d_2_2d(PointCloud<PointXYZ> nuvem_in, cv::Mat img,
                                                                                                   Eigen::Matrix3f K, Eigen::Matrix4f T) // adaptado de saveandwork
{
        // Matriz de projecao
        Eigen::MatrixXf P(3, 4);
        P = K*T.block<3,4>(0, 0);
        // Nuvem de saida
        PointCloud<PointXYZRGB> nuvem_out;
        ROS_INFO("Nuvem que chegou aqui, tamanho %zu. Dimensoes da imagem: %d  %d", nuvem_in.size(), img.cols, img.rows);
        #pragma omp parallel for num_threads(20)
        for(size_t i=0; i < nuvem_in.size(); i++){
                Eigen::MatrixXf X_(4,1);
                X_ << nuvem_in.points[i].x, nuvem_in.points[i].y, nuvem_in.points[i].z, 1;
                // Projeta e tira a escala
                Eigen::MatrixXf X = P*X_;
                X = X/X(2,0);
                // Atribui a cor se for possivel
                if(floor(X(0,0)) >= 0 && floor(X(0,0)) < img.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < img.rows){
                        // Cria ponto, colore e adiciona na nuvem
                        PointXYZRGB ponto;
                        ponto.b = img.at<cv::Vec3b>(int(X(0, 0)), int(X(1, 0)))[2];
                        ponto.g = img.at<cv::Vec3b>(int(X(0, 0)), int(X(1, 0)))[1];
                        ponto.r = img.at<cv::Vec3b>(int(X(0, 0)), int(X(1, 0)))[0];
                        ponto.x = nuvem_in.points[i].x; ponto.y = nuvem_in.points[i].y; ponto.z = nuvem_in.points[i].z;
                        nuvem_out.push_back(ponto);
                }
        }

        return nuvem_out;
}


Eigen::Matrix4f Registro::icp(const PointCloud<PointT>::Ptr src, // tirado de registra_nuvem.cpp
                           const PointCloud<PointT>::Ptr tgt,
                           Eigen::Matrix4f T)
{
        //ROS_INFO("Entrando no ICP");
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
        filter_grid(temp_src, leaf_size);
        filter_grid(temp_tgt, leaf_size);

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

        //ROS_INFO("ICP realizado.");

        return T_icp;
}

void Registro::filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size)
{
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}

void Registro::filter_grid(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float leaf_size)
{
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(in);
    grid.filter(*out);
}

void Registro::filter_grid(PointCloud<PointF>::Ptr in, PointCloud<PointF>::Ptr out, float leaf_size)
{
    VoxelGrid<PointF> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(in);
    grid.filter(*out);
}


