#include <iostream> 
#include <string>  
#include <vector>

// Structure to represent node of kd tree

struct Node
{
	pcl::PointXYZI point;   
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI pnt, int setId)
	:	point(pnt), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
class KdTree
{
	
    public: 
        Node *root;

        KdTree()
        : root(NULL)
        {}

        void insertHelper(Node **node, uint depth, pcl::PointXYZI point, int id){
            if (*node==NULL) { 
                std::cout << "node==NULL" << std::endl;
                *node = new Node(point, id);
            }
            else{
                std::cout << "insertHelper checkpoint 1" << std::endl;
                std::vector<float> newPoint = {point.x, point.y, point.z}; 
                std::vector<float> nodePoint = {(*node)->point.x, (*node)->point.y, (*node)->point.z}; 

                uint cd = depth % 3;
                if (newPoint[cd] < nodePoint[cd]){ 
                    insertHelper(&((*node)->left), depth+1, point, id);
                }
                else {
                    insertHelper(&((*node)->right), depth+1, point, id);
                }
            }
        }

        void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            //std::cout << "cloud->points.size (): " << cloud->points.size () << std::endl;
            for (int i = 0; i < cloud->points.size(); ++i){
                std::cout << "setInputCloud i:" << i << std::endl;
                insertHelper(&root, 0, cloud->points[i], i);
            }
            
        }

        
        void searchHelper(pcl::PointXYZI target, Node* node, int depth, float tol, std::vector<int> &nearest){
            
            if(node != NULL){
                if ( node->point.x >= (target.x - tol) && node->point.x <= (target.x + tol) &&
                     node->point.y >= (target.y - tol) && node->point.y <= (target.y + tol) &&
                     node->point.z >= (target.z - tol) && node->point.z <= (target.z + tol))
                    {
                        float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) + 
                                              (node->point.y - target.y)*(node->point.y - target.y) + 
                                              (node->point.z - target.z)*(node->point.z - target.z));
                                            
                        if (distance <= tol){
                            nearest.push_back(node->id);
                        }
                    }

                std::vector<float> targetPoint = {target.x, target.y, target.z}; 
                std::vector<float> nodePoint = {node->point.x, node->point.y, node->point.z}; 

                if((targetPoint[depth%3] - tol) < nodePoint[depth%3]){
                    searchHelper(target, node->left, depth+1, tol, nearest); 
                } else {
                    searchHelper(target, node->right, depth+1, tol, nearest); 
                }

            }
        }
        
        // return a list of point ids in the tree that are within distance of target
        void radiusSearch(pcl::PointXYZI target, double clusterTolerance, std::vector<int> &nearest, std::vector<float> &distances )
        {
            searchHelper(target, root, 0, clusterTolerance, nearest);
        }
        
        

};




