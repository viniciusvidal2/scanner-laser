#include "../include/scanner_gui/saveandwork.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

///////////////////////////////////////////////////////////////////////////////////////////
SaveAndWork::SaveAndWork()
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
    string year, month, day, hour, minutes;
    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
    month   = boost::lexical_cast<std::string>(now->tm_mon );
    day     = boost::lexical_cast<std::string>(now->tm_mday);
    hour    = boost::lexical_cast<std::string>(now->tm_hour);
    minutes = boost::lexical_cast<std::string>(now->tm_min );
    std::string date = "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";

    pasta = std::string(home)+"/Desktop/nuvem"+date+"/";

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
    // Projetar cada nuvem parcial na foto correspondente e colorir todo mundo ali para o resultado final

    // Alterar total a nuvem que se ve para nuvem colorida sem normais

    // Calcular normais de cada nuvem

    // Salvar cada nuvem parcial

    // Salvar a nuvem acumulada colorida
    this->process_and_save_final_cloud(acumulada_colorida);
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
    pcl::RadiusOutlierRemoval<PointNormal> out;
    out.setRadiusSearch(0.1);
    out.setMinNeighborsInRadius(10);
    out.setInputCloud(entrada);
    out.filter(*entrada);

    pcl::StatisticalOutlierRemoval<PointNormal> sor;
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
void SaveAndWork::calculate_normals(PointCloud<PointC>::Ptr entrada, PointCloud<PointT>::Ptr acc_normal){

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
