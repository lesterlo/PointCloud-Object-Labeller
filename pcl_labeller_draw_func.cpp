// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


//###################################################
//
//          UI Render function
//
//###################################################

#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"

void 
PCL_Labeller::drawAllLabel(int highlisted_index)
{
  int render_id = 0;

  //Remove all shape and Coordinate System before Drawing
  viewer->removeAllShapes();//Clear all bounding cube first
  viewer->removeAllCoordinateSystems(); //Clear all CoordinateSystem

  //Add the element
  viewer->addCoordinateSystem(); //Add the Center Axis
  for(HSTM_Label item : label_holder)
  {
    //Get common element
    //Rotation
    Eigen::Quaternionf box_rotate = Eigen::Quaternionf( //Rotation of the Cube
        Eigen::AngleAxisf(TO_RAD(item.rotate_x), Eigen::Vector3f::UnitX()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_y), Eigen::Vector3f::UnitY()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_z), Eigen::Vector3f::UnitZ())  
      );
    //Translation
    Eigen::Vector3f box_vector = Eigen::Vector3f( 
        //Translation of center of the bounding box
          item.center_x, 
          item.center_y, 
          item.center_z
        );
    //Draw the Reference 3D axis
    if(render_id == highlisted_index)
      viewer->addCoordinateSystem(
        REF_AXIS_WIDTH, //Scale size
        Eigen::Affine3f( //Add Pose
          Eigen::Translation3f(box_vector) * //Add translation
          box_rotate//Add X, Y, Z rotation
        ),
        "cs_"+std::to_string(render_id)//Specific ID, for coordinateSystem (cs_)
      );
    //Draw the bounding cube
    viewer->addCube(
      box_vector,  //Ceter of the Box
      box_rotate,  //Rotation of the Box
      item.x_size, //width
      item.y_size, //Height
      item.z_size, //Depth
      "bb_"+std::to_string(render_id)//Specific ID, for bounding box (bb_)
    );

    //Make the cube to wireframe
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      "bb_"+std::to_string(render_id)
    );

    //Make the cube to red/green depends on the selected item
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      render_id == highlisted_index ? 0.0:1.0, //Red Color
      render_id == highlisted_index ? 1.0:0.0, //Green Color
      0.0, //Blue Color
      "bb_"+std::to_string(render_id)//Use the spcific ID, for bounding box (bb_)
    );
    //Set the Wireframe line width
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
      BCUBE_LINEWIDTH, //Line Width
      "bb_"+std::to_string(render_id)//Use the spcific ID, for bounding box (bb_)
    );
    // viewer->addText3D(
    //   item.name, 
    //   pcl::PointXYZ(
    //     item.center_x + 15.0, 
    //     item.center_y + 15.0, 
    //     item.center_z + 15.0), 
    //   1.0, //textScale
    //   1.0, //red color
    //   1.0, //green color
    //   1.0, //blue color
    //   std::to_string(render_id)
    // );//Add anotation Text

    //Display object depend on tab
    switch(ui->tabWidget->currentIndex())
    {
      //In Bounding Box tab, draw ceter line reference
      case 0:
      {
      Eigen::Vector3f ref_Arrow = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            0.0, 
            0.0, 
            0.5
          ) + 
          box_vector; //Translation of center of the bounding box
        //Render the Arrow
        viewer->addArrow(
          pcl::PointXYZ(
            ref_Arrow(0),
            ref_Arrow(1),
            ref_Arrow(2)
          ),
          pcl::PointXYZ(
            box_vector(0),
            box_vector(1),
            box_vector(2)
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          false, //Disable the distance display
          "box_pose_ref_Arrow_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          REF_ARROW_LINEWIDTH, //Line Width
          "box_pose_ref_Arrow_"+std::to_string(render_id)//Use the spcific ID
        );
        //Render a line
        viewer->addLine(
          pcl::PointXYZ(
            ref_Arrow(0),
            ref_Arrow(1),
            ref_Arrow(2)
          ),
          pcl::PointXYZ(
            box_vector(0),
            box_vector(1),
            box_vector(2)
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "box_pose_ref_line_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          REF_ARROW_LINEWIDTH, //Line Width
          "box_pose_ref_line_"+std::to_string(render_id)//Use the spcific ID
        );
      }break;

      //In skeleton Tab, draw skeleton
      case 1:
      {
      //Render the Skeleton
      bool drawSkeleton = true;
      if (drawSkeleton)
      {
        //Add Skeleton Node drawing
        //Sk1
        Eigen::Vector3f sk1_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n1_x, 
            item.sk_n1_y, 
            item.sk_n1_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk1_shift(0), 
            sk1_shift(1), 
            sk1_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk1_"+std::to_string(render_id)
        );

        //Sk2
        Eigen::Vector3f sk2_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n2_x, 
            item.sk_n2_y, 
            item.sk_n2_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk2_shift(0), 
            sk2_shift(1), 
            sk2_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk2_"+std::to_string(render_id)
        );

        //Sk3
        Eigen::Vector3f sk3_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n3_x, 
            item.sk_n3_y, 
            item.sk_n3_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk3_shift(0), 
            sk3_shift(1), 
            sk3_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk3_"+std::to_string(render_id)
        );

        //Sk4
        Eigen::Vector3f sk4_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n4_x, 
            item.sk_n4_y, 
            item.sk_n4_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk4_shift(0), 
            sk4_shift(1), 
            sk4_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk4_"+std::to_string(render_id)
        );

        //Sk5
        Eigen::Vector3f sk5_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n5_x, 
            item.sk_n5_y, 
            item.sk_n5_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk5_shift(0), 
            sk5_shift(1), 
            sk5_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk5_"+std::to_string(render_id)
        );

        //Sk6
        Eigen::Vector3f sk6_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n6_x, 
            item.sk_n6_y, 
            item.sk_n6_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk6_shift(0), 
            sk6_shift(1), 
            sk6_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk6_"+std::to_string(render_id)
        );

        //Sk7
        Eigen::Vector3f sk7_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n7_x, 
            item.sk_n7_y, 
            item.sk_n7_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk7_shift(0), 
            sk7_shift(1), 
            sk7_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk7_"+std::to_string(render_id)
        );

        //Sk8
        Eigen::Vector3f sk8_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n8_x, 
            item.sk_n8_y, 
            item.sk_n8_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk8_shift(0), 
            sk8_shift(1), 
            sk8_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk8_"+std::to_string(render_id)
        );

        //Sk9
        Eigen::Vector3f sk9_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n9_x, 
            item.sk_n9_y, 
            item.sk_n9_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk9_shift(0), 
            sk9_shift(1), 
            sk9_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk9_"+std::to_string(render_id)
        );

        //Sk10
        Eigen::Vector3f sk10_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n10_x, 
            item.sk_n10_y, 
            item.sk_n10_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk10_shift(0), 
            sk10_shift(1), 
            sk10_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk10_"+std::to_string(render_id)
        );

        //Sk11
        Eigen::Vector3f sk11_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n11_x, 
            item.sk_n11_y, 
            item.sk_n11_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk11_shift(0), 
            sk11_shift(1), 
            sk11_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk11_"+std::to_string(render_id)
        );

        //Sk12
        Eigen::Vector3f sk12_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n12_x, 
            item.sk_n12_y, 
            item.sk_n12_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk12_shift(0), 
            sk12_shift(1), 
            sk12_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk12_"+std::to_string(render_id)
        );

        //Sk13
        Eigen::Vector3f sk13_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n13_x, 
            item.sk_n13_y, 
            item.sk_n13_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk13_shift(0), 
            sk13_shift(1), 
            sk13_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk13_"+std::to_string(render_id)
        );

        //Sk14
        Eigen::Vector3f sk14_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n14_x, 
            item.sk_n14_y, 
            item.sk_n14_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk14_shift(0), 
            sk14_shift(1), 
            sk14_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk14_"+std::to_string(render_id)
        );

        //Sk15
        Eigen::Vector3f sk15_shift = 
          box_rotate * 
          Eigen::Vector3f( //Translation from the local coordinate of skeleton node
            item.sk_n15_x, 
            item.sk_n15_y, 
            item.sk_n15_z
          ) + 
          box_vector; //Translation of center of the bounding box

        viewer->addSphere(
          pcl::PointXYZ(
            //Apply The Vector to the pointxyz
            sk15_shift(0), 
            sk15_shift(1), 
            sk15_shift(2)
          ),
          SK_NODE_SIZE, //Radius of the sphere
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "sk15_"+std::to_string(render_id)
        );

        //Add Skeleton Line drawing
        //Line 1-2
        viewer->addLine(
          pcl::PointXYZ(
            sk1_shift.x(),
            sk1_shift.y(),
            sk1_shift.z()
          ),
          pcl::PointXYZ(
            sk2_shift.x(),
            sk2_shift.y(),
            sk2_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk1-2_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk1-2_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 2-3
        viewer->addLine(
          pcl::PointXYZ(
            sk2_shift.x(),
            sk2_shift.y(),
            sk2_shift.z()
          ),
          pcl::PointXYZ(
            sk3_shift.x(),
            sk3_shift.y(),
            sk3_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk2-3_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk2-3_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 2-4
        viewer->addLine(
          pcl::PointXYZ(
            sk2_shift.x(),
            sk2_shift.y(),
            sk2_shift.z()
          ),
          pcl::PointXYZ(
            sk4_shift.x(),
            sk4_shift.y(),
            sk4_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk2-4_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk2-4_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 2-7
        viewer->addLine(
          pcl::PointXYZ(
            sk2_shift.x(),
            sk2_shift.y(),
            sk2_shift.z()
          ),
          pcl::PointXYZ(
            sk7_shift.x(),
            sk7_shift.y(),
            sk7_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk2-7_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk2-7_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 4-5
        viewer->addLine(
          pcl::PointXYZ(
            sk4_shift.x(),
            sk4_shift.y(),
            sk4_shift.z()
          ),
          pcl::PointXYZ(
            sk5_shift.x(),
            sk5_shift.y(),
            sk5_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk4-5_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk4-5_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 5-6
        viewer->addLine(
          pcl::PointXYZ(
            sk5_shift.x(),
            sk5_shift.y(),
            sk5_shift.z()
          ),
          pcl::PointXYZ(
            sk6_shift.x(),
            sk6_shift.y(),
            sk6_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk5-6_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk5-6_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 7-8
        viewer->addLine(
          pcl::PointXYZ(
            sk7_shift.x(),
            sk7_shift.y(),
            sk7_shift.z()
          ),
          pcl::PointXYZ(
            sk8_shift.x(),
            sk8_shift.y(),
            sk8_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk7-8_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk7-8_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 8-9
        viewer->addLine(
          pcl::PointXYZ(
            sk8_shift.x(),
            sk8_shift.y(),
            sk8_shift.z()
          ),
          pcl::PointXYZ(
            sk9_shift.x(),
            sk9_shift.y(),
            sk9_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk8-9_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk8-9_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 3-10
        viewer->addLine(
          pcl::PointXYZ(
            sk3_shift.x(),
            sk3_shift.y(),
            sk3_shift.z()
          ),
          pcl::PointXYZ(
            sk10_shift.x(),
            sk10_shift.y(),
            sk10_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk3-10_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk3-10_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 10-11
        viewer->addLine(
          pcl::PointXYZ(
            sk10_shift.x(),
            sk10_shift.y(),
            sk10_shift.z()
          ),
          pcl::PointXYZ(
            sk11_shift.x(),
            sk11_shift.y(),
            sk11_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk10-11_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk10-11_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 11-12
        viewer->addLine(
          pcl::PointXYZ(
            sk11_shift.x(),
            sk11_shift.y(),
            sk11_shift.z()
          ),
          pcl::PointXYZ(
            sk12_shift.x(),
            sk12_shift.y(),
            sk12_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk11-12_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk11-12_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 3-13
        viewer->addLine(
          pcl::PointXYZ(
            sk3_shift.x(),
            sk3_shift.y(),
            sk3_shift.z()
          ),
          pcl::PointXYZ(
            sk13_shift.x(),
            sk13_shift.y(),
            sk13_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk3-13_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk3-13_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 13-4
        viewer->addLine(
          pcl::PointXYZ(
            sk13_shift.x(),
            sk13_shift.y(),
            sk13_shift.z()
          ),
          pcl::PointXYZ(
            sk14_shift.x(),
            sk14_shift.y(),
            sk14_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk13-14_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk13-14_"+std::to_string(render_id)//Use the spcific ID
        );
        //Line 14-15
        viewer->addLine(
          pcl::PointXYZ(
            sk14_shift.x(),
            sk14_shift.y(),
            sk14_shift.z()
          ),
          pcl::PointXYZ(
            sk15_shift.x(),
            sk15_shift.y(),
            sk15_shift.z()
          ),
          render_id == highlisted_index ? 0.0:1.0, //Red Color
          render_id == highlisted_index ? 1.0:0.0, //Green Color
          0.0, //Blue color
          "line_sk14-15_"+std::to_string(render_id)
        );
        viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
          SK_BONE_LINEWIDTH, //Line Width
          "line_sk14-15_"+std::to_string(render_id)//Use the spcific ID
        );

      }//END-if(draw_skeleton)
      }break;
    }
    render_id++;//Inrement the id counter
  }
  ui->qvtkWidget->GetRenderWindow()->Render();//Update the qvtkWidget to show the updated render view
}

void
PCL_Labeller::clean_viewer()
{
  // Clear and empty the viewer
  viewer->removeAllPointClouds(); //Clear all Point Cloud
  viewer->removeAllCoordinateSystems(); //Clear all CoordinateSystem
  viewer->removeAllShapes(); //Clear all Shape such as Cube
}