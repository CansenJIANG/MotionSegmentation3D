//#include <commonFunc.h>
//uc8
//extractFeatures::matchKeyPts(const pcl::PointCloud<RIFT32>::Ptr & desc_1,
//                             const pcl::PointCloud<RIFT32>::Ptr & desc_2,
//                             std::vector<u16> *idx_1, std::vector<u16> *idx_2)
//{
//    if(desc_1->points.size()<1 || desc_1->points.size()<1)
//    {
//        return 0;
//    }
//    pcl::KdTreeFLANN<RIFT32> matchSearch;
//    std::vector<s16> neighIndices (1);
//    std::vector<f32> neighDists (1);
//    std::vector<u16> idx_1to2_1;
//    std::vector<u16> idx_1to2_2;
//    std::vector<u16> idx_2to1_1;
//    std::vector<u16> idx_2to1_2;

//    { // match from key point 1 to key point 2
//        idx_1to2_1->clear();
//        idx_1to2_2->clear();
//        bool neighFound = false;
//        matchSearch.setInputCloud(desc_1);

//        s16 i = desc_2->points.size();
//        while(i--)
//        {
//            RIFT32 riftSearch = desc_2->at(i);

//            // find the close neighbors
//            neighFound = matchSearch.nearestKSearch(riftSearch, 1,
//                                                    neighIndices, neighDists);
//            if(neighFound)
//            {
//                idx_1to2_1.push_back(neighIndices.at(0));
//                idx_1to2_2.push_back(i);
//            }
//        }
//    }
//    {// match from key point 2 to key point 1
//        idx_2to1_1->clear();
//        idx_2to1_2->clear();
//        bool neighFound = false;
//        matchSearch.setInputCloud(desc_2);

//        s16 i = desc_1->points.size();
//        while(i--)
//        {
//            RIFT32 riftSearch = desc_1->at(i);

//            // find the close neighbors
//            neighFound = matchSearch.nearestKSearch(riftSearch, 1,
//                                                    neighIndices, neighDists);
//            if(neighFound)
//            {
//                idx_2to1_1.push_back(neighIndices.at(0));
//                idx_2to1_2.push_back(i);
//            }
//        }
//    }

//    // matches found
//    if( (idx_1->size()>1) && (idx_2->size()>1) )
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}




// build adjacency matrix
//u16 rows = desc_1->points.size();
//u16 cols = desc_2->points.size();

//    Eigen::MatrixXi adj1_2 = Eigen::MatrixXd::Zero(cols, rows);
//    Eigen::MatrixXi adj2_1 = Eigen::MatrixXd::Zero(rows, cols);

//    // fill 1 if connection exists
//    for(int i=0; i<rows; ++i)
//    {
//        adj1_2(i, idx_1to2.at(i)) = 1;
//    }
//    for(int j=0; j<cols; ++j)
//    {
//        adj2_1(j, idx_2to1.at(j)) = 1;
//    }
