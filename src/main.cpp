#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// VTK include needed for drawing lines.
#include <vtkPolyLine.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>


// Types
using PointT = pcl::PointXYZRGBA;
using PointCloudT = pcl::PointCloud <PointT>;
using PointNT = pcl::PointNormal;
using PointNCloudT = pcl::PointCloud <PointNT>;
using PointLT = pcl::PointXYZL;
using PointLCloudT = pcl::PointCloud <PointLT>;

void addSupervoxelConnectionsToViewer (
    PointT & supervoxel_center,
    PointCloudT & adjacent_supervoxel_centers,
    std::string const & supervoxel_name,
    boost::shared_ptr <pcl::visualization::PCLVisualizer> & viewer);


auto main (int argc, char ** argv) -> int {
  if (argc < 2) {
    pcl::console::print_error (
        "Syntax is: %s <pcd-file> \n "
            "--NT Dsables the single cloud transform \n"
            "-v <voxel resolution>\n-s <seed resolution>\n"
            "-c <color weight> \n-z <spatial weight> \n"
            "-n <normal_weight>\n", argv[0]);
    return (1);
  }

  auto cloud = boost::make_shared <PointCloudT> ();

  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile <PointT> (argv[1], *cloud)) {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  auto is_transform_disabled = pcl::console::find_switch (argc, argv, "--NT");

  auto voxel_resolution = 0.008f;
  auto voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);

  auto seed_resolution = 0.1f;
  auto is_seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (is_seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);

  auto color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);

  auto spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);

  auto normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  auto super = pcl::SupervoxelClustering <PointT> {voxel_resolution,
                                                   seed_resolution};
  if (is_transform_disabled) {
    super.setUseSingleCameraTransform (false);
  }
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  auto supervoxel_clusters =
      std::map <uint32_t, pcl::Supervoxel <PointT>::Ptr> {};

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n",
                            supervoxel_clusters.size ());

  auto viewer =
      boost::make_shared <pcl::visualization::PCLVisualizer> ("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);

  auto voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      2.0,
      "voxel centroids");

  viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_OPACITY,
      0.95,
      "voxel centroids");

  auto labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_OPACITY,
      0.8,
      "labeled voxels");

  auto sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

  // Enabled to see the supervoxel normals.
  viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,
                                         1,
                                         0.05f,
                                         "supervoxel_normals");

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  auto supervoxel_adjacency = std::multimap <uint32_t, uint32_t> {};
  super.getSupervoxelAdjacency (supervoxel_adjacency);

  // To make a graph of the supervoxel adjacency, we need to iterate through the
  // supervoxel adjacency multimap
  auto label_iter = supervoxel_adjacency.begin ();
  while (label_iter != supervoxel_adjacency.end ()) {
    // First get the label
    auto supervoxel_label = label_iter->first;

    // Now get the supervoxel corresponding to the label
    auto supervoxel = supervoxel_clusters.at (supervoxel_label);

    // Now iterate through adjacent supervoxels and make a point cloud of them
    auto adjacent_supervoxel_centers = PointCloudT {};
    auto adjacent_range = supervoxel_adjacency.equal_range (supervoxel_label);

    std::for_each (
        adjacent_range.first,
        adjacent_range.second,
        [&](auto const & adj) {
          adjacent_supervoxel_centers.push_back (
              supervoxel_clusters.at (adj.second)->centroid_);
        }
    );

    // Now we make a name for this polygon
    auto ss = std::stringstream {};
    ss << "supervoxel_" << supervoxel_label;

    // This function is shown below, but is beyond the scope of this tutorial -
    // basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_,
                                      adjacent_supervoxel_centers,
                                      ss.str (),
                                      viewer);
    //Move iterator forward to next label
    label_iter = supervoxel_adjacency.upper_bound (supervoxel_label);
  }

  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
  }
  return (0);
}


void addSupervoxelConnectionsToViewer (
    PointT & supervoxel_center,
    PointCloudT & adjacent_supervoxel_centers,
    std::string const & supervoxel_name,
    boost::shared_ptr <pcl::visualization::PCLVisualizer> & viewer) {
  auto points = vtkSmartPointer <vtkPoints>::New ();
  auto cells = vtkSmartPointer <vtkCellArray>::New ();
  auto polyLine = vtkSmartPointer <vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent pair
  auto adjacent_itr = adjacent_supervoxel_centers.begin ();
  for (; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr) {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  auto polyData = vtkSmartPointer <vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds ()->SetNumberOfIds (points->GetNumberOfPoints ());
  for (unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i, i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData, supervoxel_name);
}
