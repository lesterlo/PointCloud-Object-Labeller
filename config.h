// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


#ifndef PCL_LABELLER_CONFIG_H_
#define PCL_LABELLER_CONFIG_H_

//Program parameter
#define BCUBE_LINEWIDTH 1  //The Bounding box line width
#define REF_AXIS_WIDTH 0.5 //The Reference XYZ Axis size
#define SK_NODE_SIZE 0.02   //For skeleton node raidus dize
#define SK_BONE_LINEWIDTH 5

//String config

#define WINDOW_TITLE "PointCloud Object Labeller"
#define ABOUT_TITLE "About This Application"
#define ABOUT_CONTENT "<p><b>PointCloud Object Labeller</b> is a program that used to label multiple objects in a PointCloud.</p>"\
                "<p>For the user guide, please read the README file or github page.</p>"\
                "<br><p>Author: Lester Lo</p>"


#define ERROR_DIALOG_TITLE "Error !"
#define READING_FOLDER "Reading Folder: "
#define NO_TARGET_FILE_FOUND "No pcd file found at: "
#define FINISH_READING_FOLDER "Finish reading Folder: "
#define CANNOT_OPEN_PCD "Cannot read pcd file: "
#define CAN_OPEN_PCD "Successfully read the pcd file: "
#define LOADING_PCD_FILE "Loading pcd file: "
#define CAN_OPEN_LABEL "Successfully open the label file: "
#define CANNOT_OPEN_LABEL_P1 "Cannot read label file: "
#define CANNOT_OPEN_LABEL_P2 "\nCreate it?"
#define LOADING_LABEL_FILE "Loading label file: "
#define LOADED_LABEL_FILE "Successfully load the label file: "
#define PARSING_LABEL_FILE_ERROR_P1 "Error when parsing label file: "
#define PARSING_LABEL_FILE_ERROR_P2 " in Line: "
#define PARSING_LABEL_FILE_ERROR_P3 "\nABORT the parsing procedure.\nOR\nIGNORE to delete this line."
#define ABORT_LABEL_PARSING "Label parsing procedure is stopped!"
#define SAVING_LABEL "Saving Label File: "
#define CAN_SAVE_LABEL "Successfully save the Label File: "
#define CANNOT_SAVE_LABEL "Fail to save the Label File: "
#define CAN_DELETE_LABEL "Successfully delete the Label"
#define NONEED_DELETE_LABEL "Nothing to delete"




//Node default position placement
#define DEFAULT_CENTER_X 0.0
#define DEFAULT_CENTER_Y 0.0
#define DEFAULT_CENTER_Z -0.45
#define DEFAULT_X_SIZE 1.0
#define DEFAULT_Y_SIZE 1.0
#define DEFAULT_Z_SIZE 2.0
#define DEFAULT_SK1_POSITION_X 0.0
#define DEFAULT_SK1_POSITION_Y 0.0
#define DEFAULT_SK1_POSITION_Z 0.65
#define DEFAULT_SK2_POSITION_X 0.0
#define DEFAULT_SK2_POSITION_Y 0.0
#define DEFAULT_SK2_POSITION_Z 0.45
#define DEFAULT_SK3_POSITION_X 0.0
#define DEFAULT_SK3_POSITION_Y 0.0
#define DEFAULT_SK3_POSITION_Z -0.05
#define DEFAULT_SK4_POSITION_X 0.0
#define DEFAULT_SK4_POSITION_Y 0.15
#define DEFAULT_SK4_POSITION_Z 0.45
#define DEFAULT_SK5_POSITION_X 0.0
#define DEFAULT_SK5_POSITION_Y 0.2
#define DEFAULT_SK5_POSITION_Z 0.2
#define DEFAULT_SK6_POSITION_X 0.0
#define DEFAULT_SK6_POSITION_Y 0.2
#define DEFAULT_SK6_POSITION_Z -0.05
#define DEFAULT_SK7_POSITION_X 0.0
#define DEFAULT_SK7_POSITION_Y -0.15
#define DEFAULT_SK7_POSITION_Z 0.45
#define DEFAULT_SK8_POSITION_X 0.0
#define DEFAULT_SK8_POSITION_Y -0.2
#define DEFAULT_SK8_POSITION_Z 0.15
#define DEFAULT_SK9_POSITION_X 0.0
#define DEFAULT_SK9_POSITION_Y -0.2
#define DEFAULT_SK9_POSITION_Z -0.05
#define DEFAULT_SK10_POSITION_X 0.0
#define DEFAULT_SK10_POSITION_Y 0.1
#define DEFAULT_SK10_POSITION_Z -0.15
#define DEFAULT_SK11_POSITION_X 0.0
#define DEFAULT_SK11_POSITION_Y 0.1
#define DEFAULT_SK11_POSITION_Z -0.5
#define DEFAULT_SK12_POSITION_X 0.0
#define DEFAULT_SK12_POSITION_Y 0.1
#define DEFAULT_SK12_POSITION_Z -0.95
#define DEFAULT_SK13_POSITION_X 0.0
#define DEFAULT_SK13_POSITION_Y -0.1
#define DEFAULT_SK13_POSITION_Z -0.15
#define DEFAULT_SK14_POSITION_X 0.0
#define DEFAULT_SK14_POSITION_Y -0.1
#define DEFAULT_SK14_POSITION_Z -0.5
#define DEFAULT_SK15_POSITION_X 0.0
#define DEFAULT_SK15_POSITION_Y -0.1
#define DEFAULT_SK15_POSITION_Z -0.95

#endif