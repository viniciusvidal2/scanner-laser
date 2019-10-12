#include "../include/scanner_gui/saveandwork.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

///////////////////////////////////////////////////////////////////////////////////////////
SaveAndWork::SaveAndWork(Eigen::Matrix3f K_, Eigen::Matrix4f Tla):
    K_astra(K_), T_laser_astra(Tla)
{
    QFuture<void> future = QtConcurrent::run(this, &SaveAndWork::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
SaveAndWork::~SaveAndWork(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::init(){

    // Inicia a pasta de salvar os arquivos
    char* home;
    home = getenv("HOME");

    time_t t = time(0);
    struct tm * now = localtime( & t );
    string month, day, hour, minutes;
    month   = boost::lexical_cast<std::string>(now->tm_mon );
    day     = boost::lexical_cast<std::string>(now->tm_mday);
    hour    = boost::lexical_cast<std::string>(now->tm_hour);
    minutes = boost::lexical_cast<std::string>(now->tm_min );
    std::string date = "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";

    pasta = std::string(home)+"/Desktop/Aquisicao"+date+"/";

    // Criando a pasta
    bool criado = mkdir((std::string(home)+"/Desktop/Aquisicao"+date).c_str(), 0777);
    if(!criado)
        ROS_INFO("Diretorio para salvar dados criado.");
    else
        ROS_ERROR("NAO FOI POSSIVEL CRIAR O DIRETORIO");

}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::save_image_and_clouds_partial(cv::Mat imagem, PointCloud<PointC>::Ptr nuvem_astra, PointCloud<PointXYZ>::Ptr nuvem_pixels, size_t indice){
    // Salvar a imagem na pasta certa
    std::string nome_imagem = "im_astra_"+std::to_string(int(indice))+".jpg";
    std::vector<int> opcoes;
    opcoes.push_back(cv::IMWRITE_JPEG_QUALITY);
    cv::imwrite(pasta+nome_imagem, imagem, opcoes);

    // Salvar a nuvem na pasta certa da astra
    std::string nome_nuvem_ast = "nv_astra_"+std::to_string(int(indice))+".ply";
    savePLYFileASCII(pasta+nome_nuvem_ast, *nuvem_astra);

    // Salvar a nuvem de pixels na pasta certa
    std::string nome_nuvem_pix = "nv_pixel_"+std::to_string(int(indice))+".ply";
    savePLYFileASCII(pasta+nome_nuvem_pix, *nuvem_pixels);
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::process_color_and_save(std::vector<cv::Mat> imagens, std::vector<PointCloud<PointXYZ>> nuvens, std::vector<float> angulos, PointCloud<PointXYZ>::Ptr acumulada_raiz, PointCloud<PointC>::Ptr acumulada_colorida){
    /// --- Projetar cada nuvem parcial na foto correspondente e colorir todo mundo ali para o resultado final --- ///
    // Cria vetor de nuvens coloridas parciais
    std::vector<PointCloud<PointC>> nuvens_coloridas(nuvens.size());
    // Para cada nuvem
    for(size_t i=0; i < nuvens_coloridas.size(); i++){
        // Obter transformaçao entre camera e laser
        Eigen::Matrix4f T = this->calculate_transformation(angulos[i]);
        // Projetar e obter nuvem com cor
        nuvens_coloridas  = this->project_cloud_to_image(nuvens[i], imagens[i], T);
    }

    // Alterar total a nuvem que se ve para nuvem colorida sem normais
    acumulada_colorida->clear();
    for(size_t i=0; i < nuvens_coloridas->size(); i++){
        *acumulada_colorida += nuvens_coloridas[i];
    }
    acumulada_raiz->clear();

    // Calcular normais de cada nuvem e salvar
    // Criar nuvem final somando as parciais novamente
    PointCloud<PointT>::Ptr final (new PointCloud<PointT>());
//    std::vector<PointCloud<PointT>> parciais_normal(nuvens.size());
    PointCloud<PointT>::Ptr temp (new PointCloud<PointT>());
    for(size_t i=0; i < nuvens.size(); i++){
        calculate_normals(nuvens[i], temp);
        final += temp;
//        parciais_normal[i] = *temp;
        savePLYFileASCII(pasta+"parcial_"+std::to_string(i+1)+".ply", *temp);
    }

    // Salvar a nuvem acumulada colorida
    this->process_and_save_final_cloud(final);
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::save_angles_file(std::vector<float> in, std::vector<float> fn, std::vector<float> ac){
    // Abrir arquivo
    std::string nome_arquivo = pasta+"angulos.txt";
    std::string linha;
    ofstream arquivo(nome_arquivo);
    if(arquivo.is_open()){
        // Escrever linha a linha
        for(size_t i=0; i < in.size(); i++){
            // ANGULO_INICIO ANGULO_CAPTURA ANGULO_FINAL
            linha = std::to_string(in[i]) + " " + std::to_string(ac[i]) + " " + std::to_string(fn[i]) + "\n";
            std::replace(linha.begin(), linha.end(), ',', '.');
            arquivo << linha;
        }
    }
    arquivo.close();
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::process_and_save_final_cloud(PointCloud<PointT>::Ptr entrada){
    // Limpar outliers aqui de uma vez
    ROS_INFO("Comecando a filtrar a nuvem ...");
    pcl::RadiusOutlierRemoval<PointT> out;
    out.setRadiusSearch(0.1);
    out.setMinNeighborsInRadius(10);
    out.setInputCloud(entrada);
    out.filter(*entrada);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(entrada);
    sor.setMeanK(1);
    sor.setStddevMulThresh(1);
    sor.filter(*entrada);

    ROS_INFO("Nuvem filtrada.");

    // Salvar a nuvem final com cor e normais certas
    std::string nome_nuvem = "nuvem_final.ply";
    savePLYFileASCII(pasta+nome_nuvem, *entrada);
    ROS_INFO("Nuvem final salva na pasta.");
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::calculate_normals(PointCloud<PointC> entrada, PointCloud<PointT>::Ptr acc_normal){

    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    search::KdTree<PointC>::Ptr tree (new search::KdTree<PointC>());

    // Calculando as normais
    NormalEstimationOMP<PointC, Normal> ne;
    ne.setInputCloud(entrada);

    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(20);
    ne.setNumberOfThreads(8);

    ne.compute(*cloud_normals);

    concatenateFields(*entrada, *cloud_normals, *acc_normal);

    std::vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra
    for(unsigned long i=0; i < acc_normal->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_normal->points[i].normal_x, acc_normal->points[i].normal_y, acc_normal->points[i].normal_z;
        cp << C(0)-acc_normal->points[i].x, C(1)-acc_normal->points[i].y, C(2)-acc_normal->points[i].z;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_normal->points[i].normal_x = -acc_normal->points[i].normal_x;
            acc_normal->points[i].normal_y = -acc_normal->points[i].normal_y;
            acc_normal->points[i].normal_z = -acc_normal->points[i].normal_z;
        }
    }

}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::calculate_normals(PointCloud<PointXYZ> entrada, PointCloud<PointNormal>::Ptr acc_normal){

    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>());

    // Calculando as normais
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setInputCloud(entrada);

    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(20);
    ne.setNumberOfThreads(8);

    ne.compute(*cloud_normals);

    concatenateFields(*entrada, *cloud_normals, *acc_normal);

    std::vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra
    for(unsigned long i=0; i < acc_normal->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_normal->points[i].normal_x, acc_normal->points[i].normal_y, acc_normal->points[i].normal_z;
        cp << C(0)-acc_normal->points[i].x, C(1)-acc_normal->points[i].y, C(2)-acc_normal->points[i].z;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_normal->points[i].normal_x = -acc_normal->points[i].normal_x;
            acc_normal->points[i].normal_y = -acc_normal->points[i].normal_y;
            acc_normal->points[i].normal_z = -acc_normal->points[i].normal_z;
        }
    }

}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f SaveAndWork::calculate_transformation(float thetay_deg){
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

    return Trot.inverse()*T_laser_astra;
}
///////////////////////////////////////////////////////////////////////////////////////////
PointCloud<PointC> SaveAndWork::project_cloud_to_image(PointCloud<PointXYZ> in, cv::Mat img, Eigen::Matrix4f T){
    // Matriz de projecao
    Eigen::MatrixXf P(3, 4);
    P = K_astra*T.block<3,4>(0, 0);
    // Nuvem de saida
    PointCloud<PointC> saida;
    #pragma omp parallel for num_threads(20)
    for(size_t i=0; i < in.size(); i++){
        Eigen::MatrixXf X_(4,1);
        X_ << in.points[i].x, in.points[i].y, in.points[i].z, 1;
        // Projeta e tira a escala
        Eigen::MatrixXf X = P*X_;
        X = X/X(0,2);
        // Atribui a cor se for possivel
        if(floor(X(0,0)) >= 0 && floor(X(0,0)) < img.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < img.rows){
            // Cria ponto, colore e adiciona na nuvem
            PointC ponto;

            saida.push_back(ponto);
        }
    }

    return saida;
}
