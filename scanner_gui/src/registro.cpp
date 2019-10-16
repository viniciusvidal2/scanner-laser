


namespace scanner_gui 
{
	
	
	void Registro::Registro()
	{
		
		K_astra << 525.1389,    1.4908,  324.1741,
			0,  521.6805,  244.8827,
			0,         0,    1.0000;
		
	}
	
	
	void Registro::run()
	{
		for (size_t i = 0; i < angulos_captura.size(); i++)
		{
			
		// Passar nuvem_laser para coordenadas da câmera Astra (RGB-D)
		
			// Transformação: Laser Rotacionado -> Laser Inicial
			Eigen::Matrix4f T1 = this->transformada_laser_rot(angulos_captura[i]);
			
			// Transformação: Laser -> Astra
			Eigen::Matrix4f T2 = this->transformada_laser2astra();
			
			// Transformação Combinada
			Eigen::Matrix4f T_comb = T1.inverse()*T2;
			
		// Refinando a transformada utilizando ICP
			Eigen::Matrix4f T_icp = this->icp(nuvem_laser[i], nuvem_astra[i], T_comb);
			
		// Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ASTRA)	
			PointCloud<PointXYZRGB>  nuvem_i[i] = projetar_3d_2_2d(nuvem_in, img, K_astra, T_icp);
			
		// Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ZED -  sem otimização)	
			//PointCloud<PointXYZRGB>  nuvem_i[i] = projetar_3d_2_2d(nuvem_in, img, K_astra, T_zed);
		
		// Projetando nuvem nas imagens correspondentes para adquirir cor (usando imagem da ZEC com otimização por BAT)	
			//PointCloud<PointXYZRGB>  nuvem_i[i] = projetar_3d_2_2d(nuvem_in, img, K_astra, T_zed_opt);

		// Acumulando nuvem
			acc = acc + nuvem_i[i];
			
		// Salvando nuvem parcial
			
		}
		
		// Salvando nuvem acumulada
		
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
		
		return T_laser2astra;
	}
	
	
PointCloud<PointXYZRGB> Registro::projetar_3d_2_2d(PointCloud<PointXYZ> nuvem_in, cv::Mat img, Eigen::Matrix3f K, Eigen::Matrix4f T){ // adaptado de saveandwork
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
	
} // Fim namespace scanner_gui
