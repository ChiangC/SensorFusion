/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	int getTreeDepth(Node * node){
		if(node == NULL){
			return 0;
		}

		if(node->left == NULL && node->right){
			return 1;
		}

		int leftDepth = getTreeDepth(node->left);
		int rightDepth = getTreeDepth(node->right);

		return leftDepth > rightDepth?leftDepth:rightDepth;
	}

	Node* getNewNode(std::vector<float> point, int id){
		Node* node = new Node(point, id);
		return node;
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
		/*if(*node == NULL){
			*node = getNewNode(point, id);
		}else{
			if(depth%2 == 0){//splits the x region
				if(point[0] < (*node)->left->point[0]){
					insertHelper(&(*node)->left, getTreeDepth(root), point, id);
				}else{
					insertHelper(&(*node)->right, getTreeDepth(root), point, id);
				}
			}else{//splits the y region
				if(point[1] < (*node)->left->point[1]){
					insertHelper(&(*node)->left, getTreeDepth(root), point, id);
				}else{
					insertHelper(&(*node)->right, getTreeDepth(root), point, id);
				}
			}
		}*/
		if(*node == NULL){
			*node = getNewNode(point, id);
		}else{
			//calculate current dim
			uint cd = depth%2;
			if(point[cd] < (*node)->point[cd]){
				// insertHelper(&(*node)->left, getTreeDepth(root), point, id);
				insertHelper(&(*node)->left, depth + 1, point, id);
			}else{
				insertHelper(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// insertHelper(&root, getTreeDepth(root), point, id);
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids){
		if(NULL != node){
			if( (node->point[0]>=(target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol))
			  && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))){
				  
				  float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
				  if(distance <= distanceTol){
					  ids.push_back(node->id);
				  }

				//check accross boundary
				if((target[depth%2] - distanceTol) < node->point[depth%2]){
					searchHelper(target, distanceTol, node->left, depth+1, ids);
				}

				if((target[depth%2] + distanceTol) > node->point[depth%2]){
					searchHelper(target, distanceTol, node->right, depth+1, ids);
				}
			}
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		if(root != NULL){
			searchHelper(target, distanceTol, root, 0, ids);
		}
		return ids;
	}
	

};




