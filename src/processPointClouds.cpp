// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>); 
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered); 

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>); 

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint); 
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion); 


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (int index: inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);



    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size()==0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// alternative function without using existing RANSAC function in PCL library. 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    while(maxIterations--){
        pcl::PointIndices::Ptr inliers_tmp {new pcl::PointIndices}; 
        while(inliers_tmp->indices.size() < 3){
            inliers_tmp->indices.push_back(rand()%(cloud->points.size()));
        }

        float x1, x2, x3, y1, y2, y3, z1, z2, z3; 

        auto itr = inliers_tmp->indices.begin();

        x1 = cloud->points[*itr].x; 
        y1 = cloud->points[*itr].y; 
        z1 = cloud->points[*itr].z; 
        itr++;
        x2 = cloud->points[*itr].x; 
        y2 = cloud->points[*itr].y; 
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x; 
        y3 = cloud->points[*itr].y; 
        z3 = cloud->points[*itr].z;
        
        float a = (y2 - y1)*(z3 - z1) - (y3 - y1)*(z2 - z1);
        float b = (z2 - z1)*(x3 - x1) - (z3 - z1)*(x2 - x1);
        float c = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
        float d = - (a*x1 + b*y1 + c*z1);

        for (int index =0; index < cloud->points.size(); index++){
            //if (inliers_tmp->indices.count(index) >0 ) { continue;}

            pcl::PointXYZI point = cloud->points[index];    // as a point to improve, this can be changed to flexible Point type

            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c+c);

            if (dist <= distanceThreshold) { inliers_tmp->indices.push_back(index);}
        }

        if(inliers_tmp->indices.size() > inliers->indices.size()){
            inliers = inliers_tmp;
        }

    }


    if(inliers->indices.size()==0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices; 
    pcl::EuclideanClusterExtraction<PointT> ec; 
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 

        clusters.push_back(cloudCluster);
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// alternative function without using existing Euclidean function in PCL library. 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices; 

    std::vector<bool> processed(cloud->points.size(), false);

    int i = 0;
    while(i < cloud->points.size()){
        if (processed[i]){
            i++;
            continue; 
        }

        pcl::PointIndices indices_tmp;

        clusterHelper(i, cloud, indices_tmp, processed, tree, clusterTolerance);
        if (indices_tmp.indices.size() >= minSize && indices_tmp.indices.size() <= maxSize){
            clusterIndices.push_back(indices_tmp);
        } else {
            for (int remove_index: indices_tmp.indices){
                processed[remove_index] = false;
                std::cout << "cluster size is not valid:" << indices_tmp.indices.size() << " min and max size: "<<  minSize << " : " <<  maxSize << std::endl;  
            }
        }
        i++;
    }

    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 

        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper( int indice, 
                                                typename pcl::PointCloud<PointT>::Ptr cloud,  
                                                pcl::PointIndices &indices_tmp,
                                                std::vector<bool> &processed, 
                                                typename pcl::search::KdTree<PointT>::Ptr tree,
                                                float clusterTolerance)
{
    processed[indice] = true; 
    indices_tmp.indices.push_back(indice); 

    std::vector<int> nearest;
    std::vector<float> distances;
    tree->radiusSearch(cloud->points[indice], clusterTolerance, nearest, distances);

    for (int id : nearest){
        if (!processed[id]){
            clusterHelper(id, cloud, indices_tmp, processed, tree, clusterTolerance); 
        }
    }
}


// alternative function without using existing Euclidean function and KD Tree in PCL library. 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Scratch_newKDTree
(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //std::shared_ptr<KdTree<PointT>> tree;
    KdTree<PointT> *tree = new KdTree<PointT>;
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> clusterIndices; 

    std::vector<bool> processed(cloud->points.size(), false);

    int i = 0;
    while(i < cloud->points.size()){
        if (processed[i]){
            i++;
            continue; 
        }

        pcl::PointIndices indices_tmp;

        clusterHelper_newKDTree(i, cloud, indices_tmp, processed, tree, clusterTolerance);
        if (indices_tmp.indices.size() >= minSize && indices_tmp.indices.size() <= maxSize){
            clusterIndices.push_back(indices_tmp);
        } else {
            for (int remove_index: indices_tmp.indices){
                processed[remove_index] = false;
                //std::cout << "cluster size is not valid:" << indices_tmp.indices.size() << " min and max size: "<<  minSize << " : " <<  maxSize << std::endl;  
            }
        }
        i++;
    }

    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 

        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// alternative function without using existing Euclidean function and KD Tree in PCL library. 
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper_newKDTree( int indice, 
                                                typename pcl::PointCloud<PointT>::Ptr cloud,  
                                                pcl::PointIndices &indices_tmp,
                                                std::vector<bool> &processed, 
                                                //std::shared_ptr<KdTree<PointT>> tree,
                                                KdTree<PointT> *tree, 
                                                float clusterTolerance)
{
    processed[indice] = true; 
    indices_tmp.indices.push_back(indice); 

    std::vector<int> nearest;
    std::vector<float> distances;
    tree->radiusSearch(cloud->points[indice], clusterTolerance, nearest, distances);

    //std::cout <<"indice: " << indice <<  " nearest.size: " << nearest.size() << std::endl;

    for (int id : nearest){
        if (!processed[id]){
            clusterHelper_newKDTree(id, cloud, indices_tmp, processed, tree, clusterTolerance); 
        }
    }
}



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}