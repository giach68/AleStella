#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;
	//vectors for storing input clouds and then the results of pair aligning
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> input;
	


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \store the input clouds in the input vector
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  */
void loadData (int argc, char **argv)
{
    for(int i=1; i<argc; i++){
		input.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );
		pcl::io::loadPCDFile(argv[i], *input[i-1]);
		PCL_INFO("cloud %s loaded \n", argv[i]);
		}
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.005, 0.005, 0.005);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  float n;
  //float it;

	//PCL_INFO("Set the max correspondence distance: ");
  //std::cin>> n;
	//PCL_INFO("Set the maximum number of iterations: ");
  //std::cin>> it;
  //PCL_INFO("Set the reciprocal correspondences: ");
  //char d;
	//std::cin >> d;
		
  
  // Align
  pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg; //IterativeClosestPointNonLinear
  reg.setTransformationEpsilon (1e-8);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1); //n
  //reg.setEuclideanFitnessEpsilon (1e-15); 
  //reg.setRANSACIterations(200);
  reg.setUseReciprocalCorrespondences(true);
  reg.setRANSACOutlierRejectionThreshold(0.01);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 100; ++i) //it
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  
  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
 }


int main (int argc, char** argv)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> output;
	//two variables for loop control and output indexes
	bool loop=true;
	int index=0;
	bool down = false;
  // Check user input
  if (argc==1)
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO("ITERATIVE CLOSEST POINT ALIGNMENT \n");
	PCL_INFO("This program hierarchically align pair of clouds. \n");
	PCL_INFO("The resulting cloud is the combination of all the aligned input clouds. \n \n");

	PCL_INFO ("Downsampling clouds can increase working speed but it looses in precision, if you have a small cloud it is not recommended. \n \n");
  // Load data
  loadData (argc, argv);
  PCL_INFO ("Loaded %d datasets. \n \n", input.size());

  
  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

	PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
  while(loop){
	if(input.size()%2==0){
	for(int i=0; i<(input.size()-1);i+=2){	

		source = input[i];
		target = input[i+1];
		showCloudsLeft(source, target);

		

		bool ok =true;
		while(ok){
		PointCloud::Ptr temp (new PointCloud);
		
		//PCL_INFO("Do you want to use downsample? (y/n) ");

		//char d;
		//std::cin >> d;
		//if(d =='y'){ down =true;}
		pairAlign (source, target, temp, pairTransform, true);
		down =false;
		//PCL_INFO("Corrected alignment? (y/n) ");
		//char c ='y';
		//std::cin >> c;
		//if(c =='y'){ 
					//transform current pair into the global transform
					pcl::transformPointCloud (*temp, *result, GlobalTransform);

					//update the global transform
					GlobalTransform = pairTransform * GlobalTransform;

					output.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );//}
					*output[index] = *result;
					index++;
			ok =false;//}
		}



	}

	}
	else{ 
		for(int i=0; i<(input.size());i+=2){	
		if(i==input.size()-1){output.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );
		*output[index] = *input[i];
		index++;} else{

		source = input[i];
		target = input[i+1];
		showCloudsLeft(source, target);

		PointCloud::Ptr temp (new PointCloud);
		

	


		bool ok =true;
		while(ok){

		//PCL_INFO("Do you want to use downsample? (y/n) ");

		//char d;
		//std::cin >> d;
		//if(d =='y'){ down =true;}
		pairAlign (source, target, temp, pairTransform, true);
		//down =false;
		//PCL_INFO("Corrected alignment? (y/n) ");
		//char c;
		//std::cin >> c;
		//if(c =='y'){ 
				//transform current pair into the global transform
				pcl::transformPointCloud (*temp, *result, GlobalTransform);

				//update the global transform
				GlobalTransform = pairTransform * GlobalTransform;

				output.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );
				*output[index] = *result;
				index++;
			ok =false;//}
		}


		}
		}
	
	}
	PCL_INFO("Pair alignemnt completed. \n");
	input.clear();
	for(int i=0; i<output.size(); i++){
		input.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );
		*input[i]=*output[i];
	}
	output.clear();
	index=0;
	if(input.size()==1){ 
		PCL_INFO("All pair of clouds aligned. \n");
		pcl::io::savePCDFileASCII("final.pcd", *input[0]);
		loop=false;
	}
  }
  return(0);
}