#include <iostream> 
#include <string>  
#include <vector>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
class KdTree
{
	
    public: 
        Node* root;

        KdTree()
        : root(NULL)
        {}

        void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
            if (*node==NULL) { *node = new Node(point, id);}
            else{
                uint cd = depth % 2;
                if (point[cd] < ((*node)->point[cd])){ 
                    insertHelper(&((*node)->left), depth+1, point, id);
                }
                else {
                    insertHelper(&((*node)->right), depth+1, point, id);
                }
            }
        }

        void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            insertHelper(&root, 0, point, id);
        }

        /*
        void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids){
            if(notde!=NULL){
                if ((node->point[0]>=(target[0] - distanceTol) && node ->point[0]<=(target[0] + distanceTol)) && (node->point[1]>=(target[1] -distanceTol)))
                    float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
                    if (distance <= distanceTol){
                        ids.push_back(node->id);
                    }

                if((target[depth%2] - distanceTol) < node->point[depth%2]){
                    searchHelper(target, node->left, depth+1, distanceTol, ids); 
                } else {
                    searchHelper(target, node->right, depth+1, distanceTol, ids); 
                }

            }
        }
        
        
        
        // return a list of point ids in the tree that are within distance of target
        void radiusSearch(const PointT &point, double clusterTolerance, std::vector<int> &nearest, std::vector<float> &distances )
        {
            std::vector<int> ids;  
        }
        */
        

};




