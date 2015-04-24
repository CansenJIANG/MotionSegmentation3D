#include "trk3dfeatures.h"
#include "commonHeader.h"
#include "commonFunc.h"
trk3dFeatures::trk3dFeatures()
{
}


void
trk3dFeatures::trkRefMatches()
{

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

    pcl::io::savePCDFileASCII ("filteredKeyPts.pcd", *filteredKeyPts);
    pcl::io::savePCDFileASCII ("filteredKeyPts2.pcd", *filteredKeyPts2);

    uc8 matchesFound = 0;
    switch ( (int) this->featDescrStr.detectorIdx) {
    case 1: // RIFT descriptor
    {
        // construct descriptors
        pcl::PointCloud<RIFT32>::Ptr riftCloud_1 (new pcl::PointCloud<RIFT32>);
        pcl::PointCloud<RIFT32>::Ptr riftCloud_2 (new pcl::PointCloud<RIFT32>);

        riftCloud_1 = featureDetector->RIFTcolorDescriptor(cloud, filteredKeyPts,
                                                           this->featDescrStr.params);

        riftCloud_2 = featureDetector->RIFTcolorDescriptor(cloud2, filteredKeyPts2,
                                                           this->featDescrStr.params);

        matchesFound = featureDetector->crossMatching(riftCloud_1, riftCloud_2,
                                                      &featDescrStr.matchIdx1, &featDescrStr.matchIdx2);

        // output info
        ui->outputMsg->appendPlainText( QString("RIFT descriptor computed.") );
        break;
    }
    case 2:
    {
        // construct descriptors
        pcl::PointCloud<SHOT352>::Ptr Shot352Cloud_1;
        pcl::PointCloud<SHOT352>::Ptr Shot352Cloud_2;

        Shot352Cloud_1 = featureDetector->Shot352Descriptor(cloud, filteredKeyPts,
                                                            this->featDescrStr.params);

        Shot352Cloud_2 = featureDetector->Shot352Descriptor(cloud2, filteredKeyPts2,
                                                            this->featDescrStr.params);

        matchesFound = featureDetector->crossMatching(Shot352Cloud_1, Shot352Cloud_2,
                                                      &featDescrStr.matchIdx1, &featDescrStr.matchIdx2);

        // output info
        ui->outputMsg->appendPlainText( QString("SHOT352 descriptor computed.") );
        break;
    }
    case 3:
    {
        // construct descriptors
        pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_1;
        pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_2;

        Shot1344Cloud_1 = featureDetector->Shot1344Descriptor(cloud, filteredKeyPts,
                                                              this->featDescrStr.params);

        Shot1344Cloud_2 = featureDetector->Shot1344Descriptor(cloud2, filteredKeyPts2,
                                                              this->featDescrStr.params);

        // match keypoints
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
        return;
    }

}
