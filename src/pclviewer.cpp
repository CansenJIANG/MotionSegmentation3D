#include "pclviewer.h"
#include "ui_PclViewer.h"
#include "OctreeViewer.h"

#include <fstream>
#include <pcl/recognition/distance_map.h>

//#include "../../JiangDLL/JiangDLL/jiangdll.h"

/////////////////////////////////////////////////////////////////////////////////////
/// Define PCLViewer
/////////////////////////////////////////////////////////////////////////////////////
PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    cloud2.reset(new PointCloudT);

    // The number of points in the cloud
    cloud->points.resize (200);

    // initialize clipping threshold as 1.0m
    ui->clipThreshold->setText("1.0");
    ui->shiftX_val->setText("0.0");
    ui->shiftY_val->setText("0.0");
    ui->shiftZ_val->setText("0.0");
    this->clipThd = 1.0;
    this->shiftPC_X = 0.0;
    this->shiftPC_Y = 0.0;
    this->shiftPC_Z = 0.0;

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
    connect (ui->movePcSlider, SIGNAL (valueChanged (int)), this, SLOT (movePcSlider (int)));
    connect (ui->lineWidthSlider, SIGNAL (valueChanged (int)), this, SLOT (lineWidthSlider (int)));
    viewer->addPointCloud (cloud, "cloud");
    pSliderValueChanged (1);
    movePcSlider(1);
    lineWidthSlider(1);

    // set camera position
    viewer->resetCamera ();
    viewer->initCameraParameters();
    // Get Current View Pose
    //    Eigen::Affine3f camPos = viewer->getViewerPose();
    //    float *cPdata = camPos.data();
    //    std::cout<<"camPos_1: "<<*(cPdata+1)<<" camPos_2: "<<*(cPdata)<<" camPos_3: "<<*(cPdata+2)<<std::endl;
    viewer->setCameraPosition(0, 0, 0,  -0.00723988,-0.999971, 0.0021689);

    // add camera coordinate
    //    viewer->addCoordinateSystem (0.01);

    // update viewer
    ui->qvtkWidget->update ();

    // Output message
    ui->outputMsg->appendPlainText(QString("Program start ..."));

    cb_args.ptColor[0] = 255;
    cb_args.ptColor[0] = 0;
    cb_args.ptColor[0] = 0;
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Define load point cloud function
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_LoadPC_clicked()
{
    // load *.pcd file
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home/jiang/CvData/kinect_textured/", tr("Files (*.pcd)"));
    if(fileName.size()<1)
    {
        return;
    }

    fName = fileName.toStdString();
    pcl::io::loadPCDFile<PointT>(fName, *cloud);
    std::vector<s16> nanIdx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIdx);

    // Store the point cloud color
    for (size_t i = 0; i < cloud->size(); i++)
    {
        cloudR.push_back(cloud->points[i].r);
        cloudG.push_back(cloud->points[i].g);
        cloudB.push_back(cloud->points[i].b);
    }

    if( cloud->points.size() )
    {
        // display the loaded point cloud filename
        std::string dispFileName = fName.substr(fName.size()-20).c_str();
        dispFileName = "File_1: " + dispFileName;
        ui->fileName_label->setText( dispFileName.c_str());
        dispFileName = fName.substr(fName.size()-20).c_str();
        dispFileName = "Loaded file " + dispFileName + ".";
        ui->outputMsg->appendPlainText( QString(dispFileName.c_str()) );
        char oMsg[200];
        std::sprintf(oMsg, "Size of loaded point cloud: %u.", cloud->points.size());
        ui->outputMsg->appendPlainText( QString(oMsg) );

        // Point selection function description
        ui->outputMsg->appendPlainText( QString("Shift+click to select feature points ...") );
    }else
    {
        ui->outputMsg->appendPlainText( QString("ERROR: Load .pcd file failed ... ") );
    }

    // Activate point selection function
    this->on_getPoint_clicked();

    // update the point cloud viewer
    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();
}

void
PCLViewer::on_showCloud_1_clicked()
{
    QString showKeypts = "Show cloud_1";
    QString hideKeypts = "Hide cloud_1";

    if(cloud->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( showKeypts, ui->showCloud_1->text(), Qt::CaseInsensitive) )
    {
        ui->showCloud_1->setText(showKeypts);
        viewer->removePointCloud("cloud");
        ui->outputMsg->appendPlainText(QString("Cloud_1 is hidden"));
    }else
    {
        ui->showCloud_1->setText(hideKeypts);
        viewer->addPointCloud(cloud,"cloud");
        ui->outputMsg->appendPlainText(QString("Cloud_1 is shown."));
    }
    ui->qvtkWidget->update();
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to add multiple point clouds
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_add_PC_clicked()
{
    // load *.pcd file
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home/jiang/CvData/kinect_textured/", tr("Files (*.pcd)"));
    if(fileName.size()<1)
    {
        return;
    }
    this->fName2 = fileName.toStdString();

    cloud2.reset(new PointCloudT);
    pcl::io::loadPCDFile<PointT>(this->fName2, *cloud2);
    std::vector<s16> nanIdx;
    pcl::removeNaNFromPointCloud(*cloud2,*cloud2, nanIdx);

    if( cloud2->points.size() )
    {
        // display the loaded point cloud filename
        std::string dispFileName = this->fName2.substr(this->fName2.size()-20).c_str();
        dispFileName = this->fName2.substr(this->fName2.size()-20).c_str();
        dispFileName =  "File_2:" + dispFileName;
        ui->fileName_label2->setText(dispFileName.c_str());
        dispFileName = this->fName2.substr(this->fName2.size()-20).c_str();
        dispFileName = "Loaded file " + dispFileName + ".";
        ui->outputMsg->appendPlainText( QString(dispFileName.c_str()) );
        char oMsg[200];
        std::sprintf(oMsg, "Size of loaded point cloud: %u.", cloud2->points.size());
        ui->outputMsg->appendPlainText( QString(oMsg) );
        viewer->addPointCloud(cloud2,this->fName2.substr(this->fName2.size()-20).c_str());
        viewer->updatePointCloud (cloud2, this->fName2.substr(this->fName2.size()-20).c_str());
    }else
    {
        ui->outputMsg->appendPlainText( QString("ERROR: Load .pcd file failed ... ") );
    }

    // Activate point selection function
    this->on_getPoint_clicked();

    // update the point cloud viewer
    ui->qvtkWidget->update ();
}

void
PCLViewer::on_showCloud_2_clicked()
{
    QString showKeypts = "Show cloud_2";
    QString hideKeypts = "Hide cloud_2";

    if(cloud2->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( showKeypts, ui->showCloud_2->text(), Qt::CaseInsensitive) )
    {
        ui->showCloud_2->setText(showKeypts);
        viewer->removePointCloud(this->fName2.substr(this->fName2.size()-20).c_str());
        ui->outputMsg->appendPlainText(QString("Cloud_2 is hidden"));
    }else
    {
        ui->showCloud_2->setText(hideKeypts);
        viewer->addPointCloud(cloud2,this->fName2.substr(this->fName2.size()-20).c_str());
        ui->outputMsg->appendPlainText(QString("Cloud_2 is shown."));
    }
    ui->qvtkWidget->update();
}

/////////////////////////////////////////////////////////////////////////////////////
/// Display the point cloud with color information
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_chkbox_withColor_clicked()
{
    // show point cloud color
    if(ui->chkbox_withColor->checkState()){
        for (size_t i = 0; i < cloud->size(); i++)
        {
            cloud->points[i].r = cloudR[i];
            cloud->points[i].g = cloudG[i];
            cloud->points[i].b = cloudB[i];
        }
    }
    // show grey point cloud
    else
    {
        for (size_t i = 0; i < cloud->size(); i++)
        {
            cloud->points[i].r = 128;
            cloud->points[i].g = 128;
            cloud->points[i].b = 128;
        }
    }

    // update point cloud viewer
    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();
}

/////////////////////////////////////////////////////////////////////////////////////
/// change size of point cloud
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::pSliderValueChanged (int value)
{
    // change the size of point cloud according to the value of slider
    viewer->setPointCloudRenderingProperties (pcl::visualization::
                                              PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();
}

/////////////////////////////////////////////////////////////////////////////////////
/// Shift point cloud
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::movePcSlider (int value)
{
    //    this->cloud
    //    ui->qvtkWidget->update ();
}

void
PCLViewer::lineWidthSlider(int value)
{
    this->featDescrStr.lineWidth = (f32)value;
    if(featDescrStr.lineDrawOn)
    {
        on_removeLines_clicked();
        on_drawMatches_clicked();
    }
}


/////////////////////////////////////////////////////////////////////////////////////
/// func to voxelize the point cloud
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_getVoxel_clicked()
{

}

/////////////////////////////////////////////////////////////////////////////////////
/// func to start the kinect streaming
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_StartKinect_clicked()
{

}

/////////////////////////////////////////////////////////////////////////////////////
/// define click point event to select feature points on the point cloud
/////////////////////////////////////////////////////////////////////////////////////
f32
distanceL2(PointT p1, PointT p2)
{
    return std::sqrt( std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) +
                      std::pow(p1.z - p2.z, 2) );
}

void
clickPoint_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *) args;

    // if clicking event is null, return.
    if (event.getPointIndex () == -1)
    {
        return;
    }

    // initialize point containers
    PointT current_point, previous_point;
    current_point.x  = .0; current_point.y  = .0; current_point.z  = .0;
    previous_point.x = .0; previous_point.y = .0; previous_point.z = .0;

    // waiting for clicking event
    event.getPoint(current_point.x, current_point.y, current_point.z);

    // pushback the selected features
    unsigned int featureNb = data->clicked_points_3d->points.size();
    if(featureNb)
    {
        previous_point = data->clicked_points_3d->points[featureNb-1];
    }

    // avoid multiple selections of same feature
    f32        tree_distance = 100000.0;
    uc8    save_feature  = 1;
    u16 i = featureNb, stop_loop = 50;
    PointT *tree_feature = &data->clicked_points_3d->points[featureNb];
    // variable stop_loop to avoid infinite searching of duplicate points
    while(i-- && stop_loop--)
    {
        --tree_feature;
        // avoid duplicate points
        tree_distance = distanceL2(*tree_feature, current_point);
        if(tree_distance < 10e-6)
        {
            save_feature = 0; break;
        }
    }
    // constrain two neighbor features have distance bigger than 0.01 meter
    float feature_distance = distanceL2( previous_point, current_point );
    if( feature_distance > 0.01 && save_feature)
    {
        data->clicked_points_3d->push_back(current_point);
    }

    // draw clicked points in green:
    PointColor clickedColor (data->clicked_points_3d, data->ptColor[0],
            data->ptColor[1], data->ptColor[2]);
    data->viewerPtr->removePointCloud("selected_features");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, clickedColor, "selected_features");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      10, "selected_features");

    // Output the information of selected features
    char oMsg[200];
    std::sprintf(oMsg, "Selected pt: %f %f %f.", current_point.x,
                 current_point.y, current_point.z);
    data->txtEditor->appendPlainText( QString(oMsg) );
    std::sprintf(oMsg, "# of selected pt: %u.", data->clicked_points_3d->points.size());
    data->txtEditor->appendPlainText( QString(oMsg) );
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to get feature points from cloud
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_getPoint_clicked()
{
    // initialize the feature points
    featurePts.reset(new PointCloudT);
    cb_args.clicked_points_3d = featurePts;

    // Output the number of selected features
    if( cb_args.clicked_points_3d->size() )
    {
        char oMsg[200];
        std::sprintf(oMsg, "Size of selected features: %u.", cb_args.clicked_points_3d->size());
        ui->outputMsg->appendPlainText( QString(oMsg) );
    }
    // point to interface point cloud visualizer
    cb_args.viewerPtr = viewer;
    cb_args.txtEditor = ui->outputMsg;

    // activate clickPoint callback function
    viewer->registerPointPickingCallback (clickPoint_callback, (void*)&cb_args);

    // refresh point cloud viewer
    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to save selected features
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_saveFeatures_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                    "/home/jiang/CvData/Kinect_MultiObj_Motions/features_.txt", tr("features (*.txt)"));

    std::ofstream ofile;
    // create a new file or select the existing files to continue saving selected features
    ofile.open(fileName.toStdString().c_str(), std::ios_base::app);

    // Data structure: columns are different features; rows are X, Y, Z values
    for(uc8 i=0; i<3; i++)
    {
        PointT *current_point = &featurePts->points[0];
        for(u16 j = 0; j<featurePts->points.size();j++)
        {
            float *dataPt = (*current_point).data;
            ofile << *(dataPt+i) <<" " ;
            ++current_point;
        }
        ofile << std::endl;
    }
    ofile.close();

    pcl::io::savePCDFileASCII ("features_pcd.pcd", *featurePts);

    ui->outputMsg->appendPlainText( QString("Selected features are saved.") );
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to take a screenshot of the point cloud viewer
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_takeScreenshot_clicked()
{
    // take screenshot of active widget
    QPixmap screenshot;
    screenshot = QPixmap::grabWidget(ui->qvtkWidget,ui->qvtkWidget->rect());

    // Name the screenshot according the loaded .pcd file
    std::string ScreenshotName = fName;
    ScreenshotName.erase(ScreenshotName.size()-4);
    ScreenshotName += ".png";

    // save the screenshot
    QFile fileN(ScreenshotName.c_str());
    fileN.open(QIODevice::WriteOnly);
    screenshot.save(&fileN, "PNG");

    ui->outputMsg->appendPlainText( QString("Screenshot is saved.") );
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to clean the selected feature points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_cleanFeatures_clicked()
{
    // delete the selected feature points
    featurePts->clear();
    viewer->removePointCloud("selected_features");
    ui->qvtkWidget->update ();
    ui->outputMsg->appendPlainText( QString("Selected features deleted.\n") );
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to delete the previous selected feature point
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_delOnePt_clicked()
{
    // pop out the last stored feature
    if(featurePts->points.size()>0)
    {
        featurePts->points.pop_back();

        // update the point cloud
        uc8 pColor[3] = {0, 255, 0};
        this->drawKeyPts(featurePts, "selected_features", pColor, 10);
        ui->outputMsg->appendPlainText( QString("Previous selected features deleted.\n") );
    }
    else
    {
        ui->outputMsg->appendPlainText( QString("No selected feature exists.\n") );
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// Get parameters from user setting
/////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_kParamVal_0_editingFinished()
{    this->keyPtsStr.params[0] = ui->kParamVal_0->text().toFloat();   }
void PCLViewer::on_kParamVal_1_editingFinished()
{    this->keyPtsStr.params[1] = ui->kParamVal_1->text().toFloat();   }
void PCLViewer::on_kParamVal_2_editingFinished()
{    this->keyPtsStr.params[2] = ui->kParamVal_2->text().toFloat();   }
void PCLViewer::on_kParamVal_3_editingFinished()
{    this->keyPtsStr.params[3] = ui->kParamVal_3->text().toFloat();   }
void PCLViewer::on_kParamVal_4_editingFinished()
{    this->keyPtsStr.params[4] = ui->kParamVal_4->text().toFloat();   }
void PCLViewer::on_kParamVal_5_editingFinished()
{    this->keyPtsStr.params[5] = ui->kParamVal_5->text().toFloat();   }
void PCLViewer::on_kParamVal_6_editingFinished()
{    this->keyPtsStr.params[6] = ui->kParamVal_6->text().toFloat();   }
void PCLViewer::on_kParamVal_7_editingFinished()
{    this->keyPtsStr.params[7] = ui->kParamVal_7->text().toFloat();   }
void PCLViewer::on_kParamVal_8_editingFinished()
{    this->keyPtsStr.params[8] = ui->kParamVal_8->text().toFloat();   }
void PCLViewer::on_kParamVal_9_editingFinished()
{    this->keyPtsStr.params[9] = ui->kParamVal_9->text().toFloat();   }
void PCLViewer::on_kParamVal_10_editingFinished()
{    this->keyPtsStr.params[10] = ui->kParamVal_10->text().toFloat(); }
void PCLViewer::on_kParamVal_11_editingFinished()
{    this->keyPtsStr.params[11] = ui->kParamVal_11->text().toFloat(); }
void PCLViewer::on_fParamVal_0_editingFinished()
{    this->featDescrStr.params[0] = ui->fParamVal_0->text().toFloat();   }
void PCLViewer::on_fParamVal_1_editingFinished()
{    this->featDescrStr.params[1] = ui->fParamVal_1->text().toFloat();   }
void PCLViewer::on_fParamVal_2_editingFinished()
{    this->featDescrStr.params[2] = ui->fParamVal_2->text().toFloat();   }
void PCLViewer::on_fParamVal_3_editingFinished()
{    this->featDescrStr.params[3] = ui->fParamVal_3->text().toFloat();   }
void PCLViewer::on_fParamVal_4_editingFinished()
{    this->featDescrStr.params[4] = ui->fParamVal_4->text().toFloat();   }
void PCLViewer::on_fParamVal_5_editingFinished()
{    this->featDescrStr.params[5] = ui->fParamVal_5->text().toFloat();   }

// Preprocessing
void PCLViewer::on_clipThreshold_editingFinished()
{    this->clipThd = ui->clipThreshold->text().toFloat();                }
void PCLViewer::on_shiftX_val_editingFinished()
{    this->shiftPC_X = ui->shiftX_val->text().toFloat();                 }
void PCLViewer::on_shiftY_val_editingFinished()
{    this->shiftPC_Y = ui->shiftY_val->text().toFloat();                 }
void PCLViewer::on_shiftZ_val_editingFinished()
{    this->shiftPC_Z = ui->shiftZ_val->text().toFloat();                 }

/////////////////////////////////////////////////////////////////////////////////////
/// Set key point color and size
/////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_keyPtColor_activated(int index)
{
    keyPtsStr.viewColor[0] = 0;
    keyPtsStr.viewColor[1] = 0;
    keyPtsStr.viewColor[2] = 0;
    switch(index)
    {
    case 0: // Red
    {   this->keyPtsStr.viewColor[0] = 255;                             break;}
    case 1: // Green
    {   this->keyPtsStr.viewColor[1] = 255;                             break;}
    case 2: // Blue
    {   this->keyPtsStr.viewColor[2] = 255;                             break;}
    case 3: // Cyan
    {   this->keyPtsStr.viewColor[0] = 255;
        this->keyPtsStr.viewColor[2] = 255;                             break;}
    case 4: // Magenta
    {   this->keyPtsStr.viewColor[1] = 255;
        this->keyPtsStr.viewColor[2] = 255;                             break;}
    case 5: // Black
    {                                                           break;}
    case 6: // White
    {   this->keyPtsStr.viewColor[0] = 255;
        this->keyPtsStr.viewColor[1] = 255;
        this->keyPtsStr.viewColor[2] = 255;                             break;}
    default:
    {   this->keyPtsStr.viewColor[0] = 255;                             break;}
    }
}
void PCLViewer::on_keyPtSize_activated(int index)
{
    switch(index)
    {
    case 0: // medium
    {   this->keyPtsStr.viewSize = 10;                                   break;}
    case 1: // small
    {   this->keyPtsStr.viewSize = 5;                                    break;}
    case 2: // big
    {   this->keyPtsStr.viewSize = 15;                                   break;}
    default:
    {   this->keyPtsStr.viewSize = 10;                                   break;}
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// Clear detected key points
/////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_clearKeypts_clicked()
{
    // keep only the loaded point cloud
    keyPts->points.clear();
    viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud, "cloud");
    ui->qvtkWidget->update();
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to draw points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::drawKeyPts(const PointCloudT::Ptr &keyPts, const std::string pcName,
                      const uc8 pColor[], const uc8 ptSize)
{
    // color setting
    PointColor color(keyPts, pColor[0], pColor[1], pColor[2]);
    // remove previous point cloud
    viewer->removePointCloud(pcName.c_str());
    // add point cloud
    viewer->addPointCloud(keyPts, color, pcName.c_str());
    // point size setting
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             ptSize, pcName.c_str());
    ui->qvtkWidget->update ();
}


/////////////////////////////////////////////////////////////////////////////////////
/// List button to selecte key point detectors
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_keyPtDetectors_activated(int index)
{
    // if no point cloud loaded, return
    if(cloud->points.size()<1)
    {
        ui->outputMsg->appendPlainText(
                    QString("ERROR: no point cloud exists.\n") );
        ui->keyPtDetectors->setCurrentIndex(0);
        return;
    }

    // Setup the cloud pointer
    keyPts.reset          (new PointCloudT);
    filteredKeyPts. reset (new PointCloudT);
    featureDetector  = new extractFeatures;

    // Initialize parameters
    std::memset(&keyPtsStr.params, 0, sizeof(keyPtsStr.params));
    std::memset(&keyPtsStr.viewColor, 0, sizeof(keyPtsStr.viewColor));
    keyPtsStr.viewSize      = 10;
    keyPtsStr.detectorIdx   = 0;
    keyPtsStr.name          = "";
    keyPtsStr.filteredName  = "";
    keyPtsStr.filteredName2 = "";
    ui->outputMsg->appendPlainText(
                QString("Feature extractor initialized.\n") );

    // initialize key point detector parameters
    this->keyPtsStr.viewColor[0] = 255;
    this->keyPtsStr.viewColor[1] = 0;
    this->keyPtsStr.viewColor[2] = 0;
    this->keyPtsStr.viewSize     = 10;
    this->keyPtsStr.detectorIdx  = index;

    // "IssKeyPts" "SiftKeyPts" "Harris KeyPts" "NARF KeyPts"
    switch (index) {
    case 1:
    {
        this->keyPtsStr.name = "ISS";

        //  ISS parameter display
        ui->kParam_0->setText("Salient radius");
        ui->kParam_1->setText("Non-max radius");
        ui->kParam_2->setText("Gamma 2 to 1");
        ui->kParam_3->setText("Gamma 3 to 2");
        ui->kParam_4->setText("Min neighbors");
        ui->kParam_5->setText("Thread number");

        ui->kParamVal_0->setText("0.008");
        ui->kParamVal_1->setText("0.012");
        ui->kParamVal_2->setText("0.975");
        ui->kParamVal_3->setText("0.975");
        ui->kParamVal_4->setText("25");
        ui->kParamVal_5->setText("8");

        // ISS parameters default settings
        this->keyPtsStr.params[0] = 0.008;
        this->keyPtsStr.params[1] = 0.012;
        this->keyPtsStr.params[2] = 0.975;
        this->keyPtsStr.params[3] = 0.975;
        this->keyPtsStr.params[4] = 25;
        this->keyPtsStr.params[5] = 8;

        break;
    }
    case 2:
    {
        this->keyPtsStr.name = "SIFT";

        //  SIFT parameter display
        ui->kParam_0->setText("Min scale stddev");
        ui->kParam_1->setText("Octave number");
        ui->kParam_2->setText("Octave scale num.");
        ui->kParam_3->setText("Min conrast");
        ui->kParam_4->setText("Null");
        ui->kParam_5->setText("Null");

        ui->kParamVal_0->setText("0.003");
        ui->kParamVal_1->setText("6");
        ui->kParamVal_2->setText("3");
        ui->kParamVal_3->setText("5.0");
        ui->kParamVal_4->setText("NA");
        ui->kParamVal_5->setText("NA");

        // SIFT parameters default settings
        this->keyPtsStr.params[0] = 0.003;
        this->keyPtsStr.params[1] = 6;
        this->keyPtsStr.params[2] = 3;
        this->keyPtsStr.params[3] = 5.0;
        this->keyPtsStr.params[4] = 0;
        this->keyPtsStr.params[5] = 0;

        break;
    }
    case 3:
    {
        this->keyPtsStr.name = "Harris";

        //  Harris parameter display
        ui->kParam_0->setText("Min scale stddev");
        ui->kParam_1->setText("Octave number");
        ui->kParam_2->setText("Octave scale num.");
        ui->kParam_3->setText("Min conrast");
        ui->kParam_4->setText("Null");
        ui->kParam_5->setText("Null");

        // Harris parameters default settings
        this->keyPtsStr.params[0] = 0.1;
        this->keyPtsStr.params[1] = 4;
        this->keyPtsStr.params[2] = 2;
        this->keyPtsStr.params[3] = 0.5;
        this->keyPtsStr.params[4] = 0;
        this->keyPtsStr.params[5] = 0;

        break;
    }
    case 4:
    {
        this->keyPtsStr.name = "NARF";

        //  NARF parameter display
        ui->kParam_0->setText("Min scale stddev");
        ui->kParam_1->setText("Octave number");
        ui->kParam_2->setText("Octave scale num.");
        ui->kParam_3->setText("Min conrast");
        ui->kParam_4->setText("Null");
        ui->kParam_5->setText("Null");

        // NARF parameters default settings
        this->keyPtsStr.params[0] = 0.1;
        this->keyPtsStr.params[1] = 4;
        this->keyPtsStr.params[2] = 2;
        this->keyPtsStr.params[3] = 0.5;
        this->keyPtsStr.params[4] = 0;
        this->keyPtsStr.params[5] = 0;

        break;
    }
    default:
        break;
    }

    // point cloud density in a spherical space with search radius R
    // density = (number of neighbors)/100
    // threshold to remove unstable point near edges
    ui->kParam_6   ->setText("Knn radius.");
    ui->kParam_7   ->setText("Pt density thresh.");
    ui->kParamVal_6->setText("0.02");
    ui->kParamVal_7->setText("1.8");
    this->keyPtsStr.params[6]  = 0.02;
    this->keyPtsStr.params[7]  = 1.8;


    // voxel filtering
    ui->kParam_8->setText("Voxel size");
    ui->kParamVal_8 ->setText("0.01");
    ui->kParamVal_9 ->setText("0.01");
    ui->kParamVal_10->setText("0.01");
    this->keyPtsStr.params[8] = 0.01;
    this->keyPtsStr.params[9] = 0.01;
    this->keyPtsStr.params[10]= 0.01;

    // clipping distance
    ui->kParam_11   ->setText("Clipping Threshold");
    ui->kParamVal_11->setText("1.0");
    this->keyPtsStr.params[11] = 1.0;
}




/////////////////////////////////////////////////////////////////////////////////////
/// Excute key point detector
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::detectKeypts(PointCloudT::Ptr &cloud, PointCloudT::Ptr &keyPts,
                        extractFeatures *fDetector,  str_keyPts &keyPtsStr)
{
    // "IssKeyPts" "SiftKeyPts" "Harris KeyPts" "NARF KeyPts"
    char oMsg[200];
    switch (keyPtsStr.detectorIdx) {
    case 1:
    {
        // Detect Intrinsic Shape Signature key points
        keyPts = fDetector->keyPtsIss3d(cloud, keyPtsStr.params);

        // Out put info
        std::sprintf(oMsg, "%u Intrinsic Shape Signature key points detected",
                     keyPts->points.size());
        break;
    }
    case 2:
    {
        // Detect SIFT key points
        keyPts = fDetector->keyPtsSIFT(cloud, keyPtsStr.params);

        // Out put info
        std::sprintf(oMsg, "%u SIFT key points detected",
                     keyPts->points.size());
        break;
    }
    case 3:
    {
        f32 searchRadius = 0.01; // unit in meter
        NormalsT::Ptr normals = fDetector->estimateSurfaceNormals(cloud, keyPtsStr.params);
        keyPts = fDetector->keyPtsHarris3d(cloud, normals, keyPtsStr.params);

        // Out put info
        std::sprintf(oMsg, "%u Harris key points detected",
                     keyPts->points.size());
        break;
    }
    case 4:
    {
        pcl::RangeImage rangeImage = fDetector->renderRangeImage(cloud, keyPtsStr.params);
        keyPts = fDetector->keyPtsNARF(rangeImage, keyPtsStr.params);

        // Out put info
        std::sprintf(oMsg, "%u NARF key points detected",
                     keyPts->points.size());
        break;
    }
    default:
        break;
    }

    // if keep key point command activated, rename the point cloud.
    if(ui->keepKeyPts->checkState())
    {
        keyPtsStr.name = keyPtsStr.name + "x";
    }

    // draw key points and the amount
    //    this->drawKeyPts(keyPts, keyPtsStr.name, keyPtsStr.viewColor, keyPtsStr.viewSize);
    ui->outputMsg->appendPlainText( QString(oMsg) );
}

void
PCLViewer::on_runKeyPtsDetector_1_clicked()
{
    // if no point cloud loaded, return
    if(cloud->points.size()<1)
    {
        ui->outputMsg->appendPlainText(
                    QString("ERROR: no point cloud exists.\n") );
        return;
    }

    // refresh key point container
    keyPts.reset(new PointCloudT);
    keyPts->points.clear();

    // detect key points
    detectKeypts(this->cloud, this->keyPts, featureDetector,this->keyPtsStr);
}

void
PCLViewer::on_runKeyPtsDetector_2_clicked()
{
    // if no point cloud loaded, return
    if(cloud2->points.size()<1)
    {
        ui->outputMsg->appendPlainText(
                    QString("ERROR: no point cloud_2 exists.\n") );
        return;
    }

    // refresh key point container
    keyPts2.reset(new PointCloudT);
    keyPts2->points.clear();
    ui->keyPtColor->setCurrentIndex(1);
    keyPtsStr.viewColor[0] = 0;
    keyPtsStr.viewColor[1] = 255;
    keyPtsStr.viewColor[2] = 0;

    // detect key points
    detectKeypts(this->cloud2, this->keyPts2, featureDetector,this->keyPtsStr);
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to show or hide the key points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_showKeypts_1_clicked()
{
    QString showKeypts = "Show keypts_1";
    QString hideKeypts = "Hide keypts_1";

    if(keyPts->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( hideKeypts, ui->showKeypts_1->text(), Qt::CaseInsensitive) )
    {
        ui->showKeypts_1->setText(hideKeypts);
        this->drawKeyPts(keyPts, showKeypts.toStdString().c_str(),
                         this->keyPtsStr.viewColor, this->keyPtsStr.viewSize);
        ui->outputMsg->appendPlainText(QString("Key points 1 are shown."));
    }else
    {
        ui->showKeypts_1->setText(showKeypts);
        viewer->removePointCloud(showKeypts.toStdString().c_str());
        ui->qvtkWidget->update();
        ui->outputMsg->appendPlainText(QString("Key points 1 are hiden."));
    }
}

void
PCLViewer::on_showKeypts_2_clicked()
{
    QString showKeypts = "Show keypts_2";
    QString hideKeypts = "Hide keypts_2";

    if(keyPts2->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( hideKeypts, ui->showKeypts_2->text(), Qt::CaseInsensitive) )
    {
        ui->showKeypts_2->setText(hideKeypts);
        this->drawKeyPts(keyPts2, showKeypts.toStdString().c_str(),
                         this->keyPtsStr.viewColor, this->keyPtsStr.viewSize);
        ui->outputMsg->appendPlainText(QString("Key points 2 are shown."));
    }else
    {
        ui->showKeypts_2->setText(showKeypts);
        viewer->removePointCloud(showKeypts.toStdString().c_str());
        ui->qvtkWidget->update();
        ui->outputMsg->appendPlainText(QString("Key points 2 are hiden."));
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to filter the unstable key points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::filterKeyPts(PointCloudT::Ptr &cloud, PointCloudT::Ptr &keyPts,
                        PointCloudT::Ptr &filteredKeyPts,
                        extractFeatures *fDetector, str_keyPts &keyPtsStr)
{
    // remove unstable key points (e.g. near edges, small segments) using knn filtering
    filteredKeyPts = fDetector->keyPtsFilter(cloud, keyPts, keyPtsStr.params);

    // voxel filtering
    filteredKeyPts = fDetector->voxelFilter(filteredKeyPts, keyPtsStr.params);

    // remove non exist keypoints generated from voxel filtering
    filteredKeyPts = fDetector->removeNon_exist(keyPts, filteredKeyPts);


    // draw key points and the amount
    //    this->drawKeyPts(filteredKeyPts, keyPtsStr.filteredName,
    //                        keyPtsStr.viewColor, keyPtsStr.viewSize);

    // output info
    char oMsg[200];
    std::sprintf(oMsg, "%u key points remain after filtering.",
                 filteredKeyPts->points.size());
    ui->outputMsg->appendPlainText( QString(oMsg) );
}

void
PCLViewer::on_filterKeypts_1_clicked()
{
    filteredKeyPts.reset(new PointCloudT);
    keyPtsStr.filteredName = "filtered keypts";

    if(keyPts->points.size()<1)
    {
        return;
    }

    // filter key points
    filterKeyPts(cloud, keyPts, filteredKeyPts, featureDetector, keyPtsStr);
}

void
PCLViewer::on_filterKeypts_2_clicked()
{
    filteredKeyPts2.reset(new PointCloudT);
    keyPtsStr.filteredName = "filtered keypts 2";

    if(keyPts2->points.size()<1)
    {
        return;
    }

    // filter key points
    filterKeyPts(cloud2, keyPts2, filteredKeyPts2, featureDetector, keyPtsStr);
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to show or hide the filtered key points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_showFilteredKeypts_1_clicked()
{
    QString showKeypts = "Show filtered_1";
    QString hideKeypts = "Hide filtered_1";

    if(keyPts->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( hideKeypts, ui->showFilteredKeypts_1->text(), Qt::CaseInsensitive) )
    {
        ui->showFilteredKeypts_1->setText(hideKeypts);
        this->drawKeyPts(filteredKeyPts, showKeypts.toStdString().c_str(), this->keyPtsStr.viewColor, this->keyPtsStr.viewSize);
        ui->outputMsg->appendPlainText(QString("Filtered key points_1 are shown."));
    }else
    {
        ui->showFilteredKeypts_1->setText(showKeypts);
        viewer->removePointCloud(showKeypts.toStdString().c_str());
        ui->qvtkWidget->update();
        ui->outputMsg->appendPlainText(QString("Filtered key points_1 are hiden."));
    }
}

void
PCLViewer::on_showFilteredKeypts_2_clicked()
{
    QString showKeypts = "Show filtered_2";
    QString hideKeypts = "Hide filtered_2";

    if(keyPts2->points.size()<1)
    {
        return;
    }
    // switch show/hide state to control the visualization of keypts
    if( QString ::compare( hideKeypts, ui->showFilteredKeypts_2->text(), Qt::CaseInsensitive) )
    {
        ui->showFilteredKeypts_2->setText(hideKeypts);
        this->drawKeyPts(filteredKeyPts2, showKeypts.toStdString().c_str(), this->keyPtsStr.viewColor, this->keyPtsStr.viewSize);
        ui->outputMsg->appendPlainText(QString("Filtered key points_2 are shown."));
    }else
    {
        ui->showFilteredKeypts_2->setText(showKeypts);
        viewer->removePointCloud(showKeypts.toStdString().c_str());
        ui->qvtkWidget->update();
        ui->outputMsg->appendPlainText(QString("Filtered key points_2 are hiden."));
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to compute feature descriptor to match key points
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_featureDescriptor_activated(int index)
{
    // if no point cloud loaded, return
    if(cloud->points.size()<1 || cloud2->points.size()<1)
    {
        ui->outputMsg->appendPlainText(
                    QString("ERROR: feature matching require two point clouds.\n") );
        ui->featureDescriptor->setCurrentIndex(0);
        return;
    }
    // Initialize parameters
    std::memset(&featDescrStr.params, 0, sizeof(featDescrStr.params));
    std::memset(&featDescrStr.matchColor, 0, sizeof(featDescrStr.matchColor));
    featDescrStr.viewSize      = 10;
    featDescrStr.detectorIdx   = 0;
    featDescrStr.name          = "";
    featDescrStr.lineDrawOn    = 0;

    // initialize feature descriptors' parameters
    this->featDescrStr.matchColor[0] = 255;
    this->featDescrStr.matchColor[1] = 0;
    this->featDescrStr.matchColor[2] = 0;
    this->featDescrStr.lineIdx       = 0;
    this->featDescrStr.viewSize      = 10;
    this->featDescrStr.detectorIdx   = (uc8) index;
    std::cout<<"\n"<<index<<" descriptor selected\n";
    std::cout<<"\n"<<featDescrStr.detectorIdx<<" descriptor selected\n";
    this->featDescrStr.matchIdx1.resize(0);
    this->featDescrStr.matchIdx2.resize(0);

    switch (index) {
    case 1: // RIFT descriptor
    {
        ui->fParam_0->setText("Normal search rad.");
        ui->fParam_1->setText("RIFT search rad.");
        ui->fParam_2->setText("Distance bin num.");
        ui->fParam_3->setText("Gradient bin num.");
        ui->fParam_4->setText("Ransac threshold");
        ui->fParam_5->setText("Ransac iteration");
        ui->fParamVal_0->setText("0.03");
        ui->fParamVal_1->setText("0.03");
        ui->fParamVal_2->setText("4");
        ui->fParamVal_3->setText("8");
        ui->fParamVal_4->setText("0.01");
        ui->fParamVal_5->setText("1000");
        this->featDescrStr.params[0] = 0.03;
        this->featDescrStr.params[1] = 0.02;
        this->featDescrStr.params[2] = 4;
        this->featDescrStr.params[3] = 8;
        this->featDescrStr.params[4] = 0.01;
        this->featDescrStr.params[5] = 1000;
        break;
    }
    case 2:
    {
        ui->fParam_0->setText("Normal search rad.");
        ui->fParam_1->setText("SHOT352 search rad.");
        ui->fParam_2->setText("NULL");
        ui->fParam_3->setText("NULL");
        ui->fParam_4->setText("Ransac threshold");
        ui->fParam_5->setText("Ransac iteration");
        ui->fParamVal_0->setText("0.03");
        ui->fParamVal_1->setText("0.03");
        ui->fParamVal_2->setText("NA");
        ui->fParamVal_3->setText("NA");
        ui->fParamVal_4->setText("0.01");
        ui->fParamVal_5->setText("1000");
        this->featDescrStr.params[0] = 0.03;
        this->featDescrStr.params[1] = 0.02;
        this->featDescrStr.params[2] = 0;
        this->featDescrStr.params[3] = 0;
        this->featDescrStr.params[4] = 0.01;
        this->featDescrStr.params[5] = 1000;
        break;
    }
    case 3:
    {
        ui->fParam_0->setText("Normal search rad.");
        ui->fParam_1->setText("SHOT1344 search rad.");
        ui->fParam_2->setText("NULL");
        ui->fParam_3->setText("NULL");
        ui->fParam_4->setText("Ransac threshold");
        ui->fParam_5->setText("Ransac iteration");
        ui->fParamVal_0->setText("0.03");
        ui->fParamVal_1->setText("0.03");
        ui->fParamVal_2->setText("NA");
        ui->fParamVal_3->setText("NA");
        ui->fParamVal_4->setText("0.01");
        ui->fParamVal_5->setText("1000");
        this->featDescrStr.params[0] = 0.03;
        this->featDescrStr.params[1] = 0.03;
        this->featDescrStr.params[2] = 0;
        this->featDescrStr.params[3] = 0;
        this->featDescrStr.params[4] = 0.01;
        this->featDescrStr.params[5] = 1000;
        break;
    }
    default:
        break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// Match keypoints using knn search
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_matchKeypts_clicked()
{
    featDescrStr.matchIdx1.clear();
    featDescrStr.matchIdx2.clear();

    if(keyPts->points.size() <1 || keyPts2->points.size()< 1)
    {
        // output info
        ui->outputMsg->appendPlainText( QString("Key points are not computed.") );
        return;
    }

    // if keypoint filtering not activated, compute descriptor for all key points
    if(filteredKeyPts->points.size()<1)
    {   pcl::copyPointCloud(*keyPts, *filteredKeyPts);    }
    if(filteredKeyPts2->points.size()<1)
    {   pcl::copyPointCloud(*keyPts2, *filteredKeyPts2);  }

    uc8 matchesFound = 0;
    std::cout<<"\n"<<(int)this->featDescrStr.detectorIdx<<" descriptor selected\n";



    pcl::io::savePCDFileASCII ("filteredKeyPts.pcd", *filteredKeyPts);
    pcl::io::savePCDFileASCII ("filteredKeyPts2.pcd", *filteredKeyPts2);

    switch ( (int) this->featDescrStr.detectorIdx) {
    case 1: // RIFT descriptor
    {
        std::cout<<"compute RIFT descriptor\n";
        // construct descriptors
        pcl::PointCloud<RIFT32>::Ptr riftCloud_1 (new pcl::PointCloud<RIFT32>);
        pcl::PointCloud<RIFT32>::Ptr riftCloud_2 (new pcl::PointCloud<RIFT32>);

        riftCloud_1 = featureDetector->RIFTcolorDescriptor(cloud, filteredKeyPts,
                                                           this->featDescrStr.params);

        riftCloud_2 = featureDetector->RIFTcolorDescriptor(cloud2, filteredKeyPts2,
                                                           this->featDescrStr.params);

        // write out the descriptors
        {
            std::ofstream riftDesc_1, riftDesc_2;
            riftDesc_1.open ("riftDesc_1.txt");
            for(int i_1 = 0; i_1<riftCloud_1->points.size();i_1++)
            {
                RIFT32 desc_i = riftCloud_1->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    riftDesc_1 << desc_i.histogram[j_1] <<" ";
                }
                riftDesc_1 << "\n";
            }
            riftDesc_1.close();

            riftDesc_2.open ("riftDesc_2.txt");
            for(int i_1 = 0; i_1<riftCloud_2->points.size();i_1++)
            {
                RIFT32 desc_i = riftCloud_2->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    riftDesc_2 << desc_i.histogram[j_1] <<" ";
                }
                riftDesc_2 << "\n";
            }
            riftDesc_2.close();
        }

        matchesFound = featureDetector->crossMatching(riftCloud_1, riftCloud_2,
                                                      &featDescrStr.matchIdx1, &featDescrStr.matchIdx2);

        // output info
        ui->outputMsg->appendPlainText( QString("RIFT descriptor computed.") );
        break;
    }
    case 2:
    {
        std::cout<<"compute SHOT 152 descriptor\n";
        // construct descriptors
        pcl::PointCloud<SHOT352>::Ptr Shot352Cloud_1;// (new pcl::PointCloud<SHOT352>);
        pcl::PointCloud<SHOT352>::Ptr Shot352Cloud_2;// (new pcl::PointCloud<SHOT352>);

        Shot352Cloud_1 = featureDetector->Shot352Descriptor(cloud, filteredKeyPts,
                                                            this->featDescrStr.params);

        Shot352Cloud_2 = featureDetector->Shot352Descriptor(cloud2, filteredKeyPts2,
                                                            this->featDescrStr.params);

        // write out the descriptors
        {
            std::ofstream Shot352Desc_1, Shot352Desc_2;
            Shot352Desc_1.open ("Shot352Desc_1.txt");
            for(int i_1 = 0; i_1<Shot352Cloud_1->points.size();i_1++)
            {
                SHOT352 desc_i = Shot352Cloud_1->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    Shot352Desc_1 << desc_i.descriptor[j_1] <<" ";
                }
                Shot352Desc_1 << "\n";
            }
            Shot352Desc_1.close();

            Shot352Desc_2.open ("Shot352Desc_2.txt");
            for(int i_1 = 0; i_1<Shot352Cloud_2->points.size();i_1++)
            {
                SHOT352 desc_i = Shot352Cloud_2->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    Shot352Desc_2 << desc_i.descriptor[j_1] <<" ";
                }
                Shot352Desc_2 << "\n";
            }
            Shot352Desc_2.close();
        }

        std::cout<<"shot descriptor computed successfully\n";
        std::cout<<"size of descriptor 1: "<<Shot352Cloud_1->points.size()<<std::endl;
        std::cout<<"size of descriptor 2: "<<Shot352Cloud_2->points.size()<<std::endl;

        matchesFound = featureDetector->crossMatching(Shot352Cloud_1, Shot352Cloud_2,
                                                      &featDescrStr.matchIdx1, &featDescrStr.matchIdx2);

        // output info
        ui->outputMsg->appendPlainText( QString("SHOT352 descriptor computed.") );
        break;
    }
    case 3:
    {
        std::cout<<"compute SHOT 1344 descriptor\n";
        // construct descriptors
        pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_1;// (new pcl::PointCloud<SHOT352>);
        pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_2;// (new pcl::PointCloud<SHOT352>);

        Shot1344Cloud_1 = featureDetector->Shot1344Descriptor(cloud, filteredKeyPts,
                                                              this->featDescrStr.params);

        Shot1344Cloud_2 = featureDetector->Shot1344Descriptor(cloud2, filteredKeyPts2,
                                                              this->featDescrStr.params);

        // write out the descriptors
        {
            std::ofstream Shot1344Desc_1, Shot1344Desc_2;
            Shot1344Desc_1.open ("Shot1344Desc_1.txt");
            for(int i_1 = 0; i_1<Shot1344Cloud_1->points.size();i_1++)
            {
                SHOT1344 desc_i = Shot1344Cloud_1->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    Shot1344Desc_1 << desc_i.descriptor[j_1] <<" ";
                }
                Shot1344Desc_1 << "\n";
            }
            Shot1344Desc_1.close();

            Shot1344Desc_2.open ("Shot1344Desc_2.txt");
            for(int i_1 = 0; i_1<Shot1344Cloud_2->points.size();i_1++)
            {
                SHOT1344 desc_i = Shot1344Cloud_2->at(i_1);
                for(int j_1 = 0; j_1<desc_i.descriptorSize(); j_1++)
                {
                    Shot1344Desc_2 << desc_i.descriptor[j_1] <<" ";
                }
                Shot1344Desc_2 << "\n";
            }
            Shot1344Desc_2.close();
        }
        std::cout<<"shot descriptor computed successfully\n";
        std::cout<<"size of descriptor 1: "<<Shot1344Cloud_1->points.size()<<std::endl;
        std::cout<<"size of descriptor 2: "<<Shot1344Cloud_2->points.size()<<std::endl;

        matchesFound = featureDetector->crossMatching(Shot1344Cloud_1, Shot1344Cloud_2,
                                                      &featDescrStr.matchIdx1, &featDescrStr.matchIdx2,
                                                      &featDescrStr.matchDist);

        // output info
        ui->outputMsg->appendPlainText( QString("SHOT1344 descriptor computed.") );
        break;
    }
    default:
        break;
    }

    // match feature descriptors
    if( matchesFound )
    {
        // output info
        char oMsg[200];
        std::sprintf(oMsg, "%d matches found.", featDescrStr.matchIdx1.size());
        ui->outputMsg->appendPlainText( QString(oMsg) );
    }else
    {
        ui->outputMsg->appendPlainText( QString("no matches fould.") );
        std::cout<<"finding match error \n";
        return;
    }

}

/////////////////////////////////////////////////////////////////////////////////////
/// reject outliers matches using ransac
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_matchRansac_clicked()
{
    // reject outliers using ransac
    std::vector<s16> inliers;
    inliers = featureDetector->matchRansac(filteredKeyPts, filteredKeyPts2, &featDescrStr.matchIdx1,
                                           &featDescrStr.matchIdx2, featDescrStr.params[4],
            (u16) featDescrStr.params[5]);
    if(inliers.size()>0)
    {
        // output info
        char oMsg[200];
        std::sprintf(oMsg, "%u matches left after RANSAC.", featDescrStr.matchIdx1.size());
        ui->outputMsg->appendPlainText( QString(oMsg) );
    }else
    {
        ui->outputMsg->appendPlainText( QString("Ransac failed") );
    }
}

/////////////////////////////////////////////////////////////////////////////////////
/// func to extract points located at the edges
/////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::on_extractEdge_1_clicked()
{

}


///////////////////////////////////////////////////////////////////////////////////////
///// draw matching lines
///////////////////////////////////////////////////////////////////////////////////////
void
PCLViewer::drawMatches(PointCloudT::Ptr &corr_1, PointCloudT::Ptr & corr_2,
                       uc8 viewColor[])
{
    PointT *pt_1 = &corr_1->points.at(0);
    PointT *pt_2 = &corr_2->points.at(0);
    u16 idxLine = 0;
    float maxDist = -1000;
    float minDist = 1000;
    for(u16 i = 0; i<featDescrStr.matchDist.size(); i++)
    {
        if(featDescrStr.matchDist[i]>maxDist)
        {
            maxDist = featDescrStr.matchDist[i];
        }
        if(featDescrStr.matchDist[i]<minDist)
        {
            minDist = featDescrStr.matchDist[i];
        }
    }

    while( idxLine < corr_1->points.size() )
    {
        QString lineName = QString::number(idxLine);
        // scale the line width with the descriptor distance
        // smaller distance, higher the value
        float scl = minDist/featDescrStr.matchDist[idxLine];
        float idxLineWidth = featDescrStr.lineWidth*scl;
        viewer->addLine(*pt_1, *pt_2, viewColor[0], viewColor[1], viewColor[2],
                lineName.toStdString().c_str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                            idxLineWidth, lineName.toStdString().c_str());

        ++pt_1;
        ++pt_2;
        ++idxLine;
    }
    featDescrStr.lineIdx = idxLine;
    ui->qvtkWidget->update();
}

///////////////////////////////////////////////////////////////////////////////////////
///// on draw matches
///////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_drawMatches_clicked()
{
    if(featDescrStr.matchIdx1.size()<1)
    {
        return;
    }
    PointCloudT::Ptr corr_1 (new PointCloudT);
    PointCloudT::Ptr corr_2 (new PointCloudT);

    if(featDescrStr.lineIdx != 0)
    {
        ui->removeLines->click();
    }
    std::cout<<"size of matchIdx1: "<<featDescrStr.matchIdx1.size()
            <<"size of matchIdx2: "<<featDescrStr.matchIdx2.size()<<"\n";

    f32 medianDist = getMedian(featDescrStr.matchDist);

    // get correspondences
    if(ui->goodMatches->checkState())
    {// draw half of the good matches
        for(u16 i=0; i<featDescrStr.matchIdx1.size(); ++i)
        {
            if(featDescrStr.matchDist[i]<medianDist)
            {
                corr_1->push_back(filteredKeyPts ->points[ featDescrStr.matchIdx1[i]]);
                corr_2->push_back(filteredKeyPts2->points[ featDescrStr.matchIdx2[i]]);
            }
        }
    }
    else
    {
        for(u16 i=0; i<featDescrStr.matchIdx1.size(); ++i)
        {
            corr_1->push_back(filteredKeyPts ->points[ featDescrStr.matchIdx1[i]]);
            corr_2->push_back(filteredKeyPts2->points[ featDescrStr.matchIdx2[i]]);
        }
    }
    pcl::io::savePCDFileASCII ("corr_ref.pcd", *corr_1);
    pcl::io::savePCDFileASCII ("corr_mot.pcd", *corr_2);
    drawMatches(corr_1, corr_2, keyPtsStr.viewColor);
    featDescrStr.lineDrawOn = 1;
}

///////////////////////////////////////////////////////////////////////////////////////
///// remove matching lines
///////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_removeLines_clicked()
{
    if( featDescrStr.lineDrawOn == 0)
    {
        return;
    }
    for(int idxLine = 0; idxLine<featDescrStr.lineIdx; ++idxLine)
    {
        QString lineName = QString::number(idxLine);
        viewer->removeShape(lineName.toStdString().c_str());
    }
    featDescrStr.lineIdx = 0;
    ui->qvtkWidget->update();
    featDescrStr.lineDrawOn = 0;
}

void PCLViewer::on_pclRansac_clicked()
{
    pcl::CorrespondencesPtr corrs (new pcl::Correspondences ());
    PointCloudT::Ptr corr_1 (new PointCloudT);
    PointCloudT::Ptr corr_2 (new PointCloudT);
    for(int i = 0; i<featDescrStr.matchIdx1.size();i++)
    {
        s16 idx_1 = (s16)featDescrStr.matchIdx1[i];
        s16 idx_2 = (s16)featDescrStr.matchIdx2[i];
        f32 x = filteredKeyPts->points[idx_1].x - filteredKeyPts2->points[idx_2].x;
        f32 y = filteredKeyPts->points[idx_1].y - filteredKeyPts2->points[idx_2].y;
        f32 z = filteredKeyPts->points[idx_1].z - filteredKeyPts2->points[idx_2].z;
        pcl::Correspondence corr (idx_1, idx_2, commonFunc::l2norm(x, y, z) );
        corrs->push_back( corr );
        corr_1->push_back(filteredKeyPts ->points[idx_1]);
        corr_2->push_back(filteredKeyPts2->points[idx_2]);
    }
    std::cout<<"before pcl ransac corr number: "<<corrs->size()<<std::endl;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
    rejector.setInputCloud(filteredKeyPts);
    rejector.setTargetCloud(filteredKeyPts2);
    rejector.setInputCorrespondences(corrs);
    rejector.getCorrespondences(*corrs);
    std::cout<<"after pcl ransac corr number: "<<corrs->size()<<std::endl;
    featDescrStr.matchIdx1.clear();
    featDescrStr.matchIdx2.clear();
    corr_1->clear();
    corr_2->clear();
    for(int i=0; i<corrs->size(); ++i)
    {
        featDescrStr.matchIdx1.push_back(corrs->at(i).index_query);
        featDescrStr.matchIdx2.push_back(corrs->at(i).index_match);
        corr_1->push_back(filteredKeyPts ->points[corrs->at(i).index_query]);
        corr_2->push_back(filteredKeyPts2->points[corrs->at(i).index_match]);
    }

    if(featDescrStr.lineIdx != 0)
    {
        ui->removeLines->click();
    }
    drawMatches(corr_1, corr_2, keyPtsStr.viewColor);
}

void PCLViewer::on_comboBox_activated(int index)
{
    cb_args.ptColor[0] = 0;
    cb_args.ptColor[1] = 0;
    cb_args.ptColor[2] = 0;
    switch(index)
    {
    case 0: // Red
    {   cb_args.ptColor[0] = 255;                             break;}
    case 1: // Green
    {   cb_args.ptColor[1] = 255;                             break;}
    case 2: // Blue
    {   cb_args.ptColor[2] = 255;                             break;}
    case 3: // Cyan
    {   cb_args.ptColor[0] = 255;
        cb_args.ptColor[2] = 255;                             break;}
    case 4: // Magenta
    {   cb_args.ptColor[1] = 255;
        cb_args.ptColor[2] = 255;                               break;}
    case 5: // Black
    {                                                          break;}
    case 6: // White
    {   cb_args.ptColor[0] = 255;
        cb_args.ptColor[1] = 255;
        cb_args.ptColor[2] = 255;                             break;}
    default:
    {   cb_args.ptColor[0] = 255;                             break;}
    }
}

void PCLViewer::on_loadSelectedFeat_clicked()
{
    // load *.pcd file
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home/jiang/CvData/kinect_textured/", tr("Files (*.pcd)"));
    if(fileName.size()<1)
    {
        return;
    }
    PointCloudT::Ptr feature_cloud (new PointCloudT);
    std::string featureName = fileName.toStdString();
    pcl::io::loadPCDFile<PointT>(featureName, *feature_cloud);

    std::string cloudName = featureName.substr(featureName.size()-8).c_str();

    //    viewer->addPointCloud(feature_cloud,cloudName);

    // draw clicked points in green:
    PointColor clickedColor (feature_cloud, cb_args.ptColor[0],
            cb_args.ptColor[1], cb_args.ptColor[2]);
    //    data->viewerPtr->removePointCloud("selected_features");
    viewer->addPointCloud(feature_cloud, clickedColor, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             15, cloudName);

}

void PCLViewer::on_loadMatchIdx_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home/jiang/bin/PosEstMovObj/", tr("Files (*.txt)"));
    if(fileName.size()<1)
    {
        return;
    }

    std::ifstream readMatchFile;
    readMatchFile.open(fileName.toStdString().c_str());
    featDescrStr.matchIdx1.clear();
    featDescrStr.matchIdx2.clear();
    if (readMatchFile.is_open())
    {
        while (!readMatchFile.eof())
        {
            u16 idx_tmp = 0;
            readMatchFile >> idx_tmp;
            featDescrStr.matchIdx1.push_back(idx_tmp);
            std::cout<< idx_tmp <<", ";
            readMatchFile >> idx_tmp;
            featDescrStr.matchIdx2.push_back(idx_tmp);
            std::cout<< idx_tmp <<"\n";
        }
    }
    readMatchFile.close();
}

void PCLViewer::on_transformPc_clicked()
{
    Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
    t(0,3) = this->shiftPC_X;
    t(1,3) = this->shiftPC_Y;
    t(2,3) = this->shiftPC_Z;

    pcl::transformPointCloud(*cloud, *cloud, t);

    viewer->removePointCloud("cloud");
    viewer->addPointCloud(cloud,"cloud");
    ui->qvtkWidget->update();
}

///////////////////////////////////////////////////////////////////////////////////////
///// func to clip the points further than threshold
///////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_clipPC_clicked()
{
    if(cloud->points.size()>1)
    {
        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, clipThd);
        pass.filter (*cloud);
        on_showCloud_1_clicked();
        on_showCloud_1_clicked();
    }
    if(cloud2->points.size()>1)
    {
        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud2);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, clipThd);
        pass.filter (*cloud2);
        on_showCloud_2_clicked();
        on_showCloud_2_clicked();
    }
    ui->outputMsg->appendPlainText( "Clipping is done.");
}

///////////////////////////////////////////////////////////////////////////////////////
///// func to load the point cloud sequence
///////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_loadPcSequence_clicked()
{
    loadSeqStr.pcdPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                           "/home/jiang/CvDataset/",
                                                           QFileDialog::ShowDirsOnly
                                                           | QFileDialog::DontResolveSymlinks);
    loadSeqStr.files = QDir(loadSeqStr.pcdPath).entryList(QDir::Files);
    loadSeqStr.pcSeq.reset(new PointCloudT);
    loadSeqStr.seqIdx = 0;
    loadSeqStr.fpsSeq = 0;
    loadSeqStr.repeatSeq = ui->loadSeqRepeatCkbox->checkState();

    QFile f(loadSeqStr.files.at(loadSeqStr.seqIdx));
    pcl::io::loadPCDFile<PointT>(
                (loadSeqStr.pcdPath+"/"+f.fileName()).toStdString().c_str(),
                *loadSeqStr.pcSeq);
    viewer->addPointCloud(loadSeqStr.pcSeq, "pointCloudSequence");
    ui->qvtkWidget->update();
}

void PCLViewer::on_clearSeq_clicked()
{
    loadSeqStr.pcSeq->points.clear();
    loadSeqStr.seqIdx = 0;
    loadSeqStr.loadFullSeq = false;
    loadSeqStr.fullSeq.clear();
    viewer->updatePointCloud(loadSeqStr.pcSeq, "pointCloudSequence");
    ui->qvtkWidget->update();
}

void PCLViewer::on_showPrevSeq_clicked()
{
    if(loadSeqStr.files.size()<1) return;

    if(loadSeqStr.seqIdx==0 && loadSeqStr.repeatSeq == false)
    {
        return;
    }
    else if(loadSeqStr.seqIdx==0 && loadSeqStr.repeatSeq == true)
    {
        loadSeqStr.seqIdx = loadSeqStr.files.size()-1;
    }
    else
    {
        loadSeqStr.seqIdx--;
    }
    QFile f(loadSeqStr.files.at(loadSeqStr.seqIdx));
    pcl::io::loadPCDFile<PointT>(
                (loadSeqStr.pcdPath+"/"+f.fileName()).toStdString().c_str(),
                *loadSeqStr.pcSeq);

    viewer->updatePointCloud(loadSeqStr.pcSeq, "pointCloudSequence");
    ui->qvtkWidget->update();
}
void PCLViewer::on_showNextSeq_clicked()
{
    if(loadSeqStr.files.size()<1) return;

    if(loadSeqStr.seqIdx==loadSeqStr.files.size()-1
            && loadSeqStr.repeatSeq == false)
    {
        return;
    }
    else if(loadSeqStr.seqIdx==loadSeqStr.files.size()-1
            && loadSeqStr.repeatSeq == true)
    {
        loadSeqStr.seqIdx = 0;
    }
    else
    {
        loadSeqStr.seqIdx++;
    }
    QFile f(loadSeqStr.files.at(loadSeqStr.seqIdx));
    pcl::io::loadPCDFile<PointT>(
                (loadSeqStr.pcdPath+"/"+f.fileName()).toStdString().c_str(),
                *loadSeqStr.pcSeq);
    viewer->updatePointCloud(loadSeqStr.pcSeq, "pointCloudSequence");
    ui->qvtkWidget->update();
}

void PCLViewer::on_loadSeqRepeatCkbox_stateChanged(int arg1)
{
    loadSeqStr.repeatSeq = ui->loadSeqRepeatCkbox->checkState();
}

void PCLViewer::on_loadSeqFps_editingFinished()
{
    loadSeqStr.fpsSeq = ui->loadSeqFps->text().toFloat();
}

void PCLViewer::on_loadFullSeq_clicked()
{
    loadSeqStr.loadFullSeq = true;
    loadSeqStr.fullSeq.reserve( loadSeqStr.files.size() );
    PointCloudT::Ptr pcSeqTmp(new PointCloudT);
    for(size_t i=0; i<loadSeqStr.files.size(); i++)
    {
        QFile f(loadSeqStr.files.at(i));
        pcl::io::loadPCDFile<PointT>((loadSeqStr.pcdPath+"/"+
                                      f.fileName()).toStdString().c_str(), *pcSeqTmp);
        loadSeqStr.fullSeq.push_back(*pcSeqTmp);
    }
    std::cout<<loadSeqStr.fullSeq.size();
    // Output message
    ui->outputMsg->appendPlainText(
                QString("Load full point cloud sequence done."));
}

void PCLViewer::on_showFullSequence_clicked()
{
    if(loadSeqStr.loadFullSeq = false) return;
    PointCloudT::Ptr pcSeqTmp(new PointCloudT);
    s16 sleepTime = 1000/loadSeqStr.fpsSeq;
    uc8 stopInfLoop = 10;
    do
    {
        for(size_t i=0; i<loadSeqStr.files.size(); i++)
        {
            *pcSeqTmp = loadSeqStr.fullSeq.at(i);
            viewer->updatePointCloud(pcSeqTmp, "pointCloudSequence");
            ui->qvtkWidget->repaint();
            boost::this_thread::sleep(
                        boost::posix_time::milliseconds(sleepTime) );
        }
        --stopInfLoop;
    }while(loadSeqStr.repeatSeq && stopInfLoop>0);
}
