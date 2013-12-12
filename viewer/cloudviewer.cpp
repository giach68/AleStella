#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void){
								if (event.getKeySym () == "i" && event.keyDown ()){
									std::cout <<"speriamo in bene"<<std::endl;}
}

void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    std::cout << "i only run once" << std::endl;
	viewer.setCameraPosition(0,0,-2,0,-1,0,0);
	viewer.resetCamera();


    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    //FIXME: possible race condition here:
    user_data++;
	 
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile(argv[1], *cloud);

  pcl::visualization::CloudViewer viewer("viewer");
  viewer.showCloud(cloud);
  viewer.runOnVisualizationThreadOnce (viewerOneOff);
  viewer.runOnVisualizationThread(viewerPsycho);
  viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

   while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
			
    user_data++;
    }
   return(0);
}
