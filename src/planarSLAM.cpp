#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud2.h"
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <math.h>
#include <fstream>
#include <iostream>

			//This was used for printing
            //numx << currP.x;
            //numy << currP.y;
            //numz << currP.z;
            //numx1 <<corrPointB(0);
            //numy1 <<corrPointB(1);
            //numz1 <<corrPointB(2);
            //numx2 <<corrPointA(0);
            //numy2 <<corrPointA(1);
            //numz2 <<corrPointA(2);
            //
            //pointstr = numx1.str() + "," + numy1.str() + "," + numz1.str() + "\n";
            //write << pointstr;
            //pointstr = numx2.str() + "," + numy2.str() + "," + numz2.str() + "\n";
            //write << pointstr;

            //std::cout << nextCloud.points[i] << std::endl;

class ICPSLAM {
private:
    ros::Publisher ICPpointcloud;
    ros::NodeHandle n;
    ros::Subscriber Cloudsub;

    time_t start, end;
public:
    pcl::PointCloud<pcl::PointXYZ> FirstCloud;
    pcl::PointCloud<pcl::PointXYZ> SecondCloud;
    pcl::PointCloud<pcl::PointXYZ> reducedCloud1;
    pcl::PointCloud<pcl::PointXYZ> reducedCloud2;

    pcl::PointCloud<pcl::PointXYZ> UnReducedFirstCloud;
    pcl::PointCloud<pcl::PointXYZ> UnReducedSecondCloud;

    //For centroid calculations
    Eigen::Vector3f FirstCentroid;
    Eigen::Vector3f SecondCentroid;

    //for Rotation calculations and Translation
    Eigen::Matrix3f Hmatrix;
    Eigen::Matrix3f Umatrix;
    Eigen::Vector3f Smatrix;
    Eigen::Matrix3f Vmatrix;
    //Used for keeping difference between the iterations
    Eigen::Matrix3f RotationMatrix =  Eigen::Matrix3f::Zero();
    Eigen::Vector3f TranslationVec =  Eigen::Vector3f::Zero();
    //Used for keeping total distance between two clouds
    Eigen::Matrix3f TotalRotationMatrix =  Eigen::Matrix3f::Zero();
    Eigen::Vector3f TotalTranslationVec =  Eigen::Vector3f::Zero();
    //Used for keepind total distance from first to last cloud
    Eigen::Matrix3f GlobalRotationMatrix =  Eigen::Matrix3f::Zero();
    Eigen::Vector3f GlobalTranslationVec =  Eigen::Vector3f::Zero();

	//Used for mapping
    pcl::PointCloud<pcl::PointXYZ> mergedMap;
    pcl::PointCloud<pcl::PointXYZ> secondmergeMap;
    pcl::PointCloud<pcl::PointXYZ> mergedMap3;


	std::vector<float> errorvec;

    //For ICP
    int iterations = 100;
    float limit = 1000;
	//float limit = 300.00;
	int rotationlock = 0;
    bool fCloud = true;
    int valueint = 0;
    bool sCloud = false;
    int counter = 0;
    bool firstCloudToFile = true;
    bool cloudlock = true;
	int filecounter = 0;

    void ICPCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if(this->fCloud==true) {
            this->FirstCloud = this->UnpackData(msg);
            this->fCloud = false;
        }
        else if(this->FirstCloud.header.seq != msg->header.seq && sCloud == false) {
            this->SecondCloud = this->UnpackData(msg);
            //this->ICPpointcloud.publish(SecondCloud);
            this->secondmergeMap += this->SecondCloud;
  			pcl::io::savePCDFileASCII("/home/martin/catkin_ws/src/planarSLAM/merged_map_cloud2.pcd", this->secondmergeMap);
			
            this->sCloud = true;
        }
        if(this->sCloud == true) {
            this->ICPCalc();
        }

    }

//constructor
    ICPSLAM() {
        ICPpointcloud = n.advertise<sensor_msgs::PointCloud2>("/ICP", 1000);
        Cloudsub = n.subscribe("/velodyne_points",1000,&ICPSLAM::ICPCallback, this);
    }

    static pcl::PointCloud<pcl::PointXYZ> UnpackData(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 currentPointCloud;
        pcl::PCLPointCloud2 p2ToPCL;
        pcl::PointCloud<pcl::PointXYZ> currentPCL;

        currentPointCloud.header = msg->header;
        currentPointCloud.height = msg->height;
        currentPointCloud.width = msg->width;
        currentPointCloud.fields = msg->fields;
        currentPointCloud.is_bigendian = msg->is_bigendian;
        currentPointCloud.point_step = msg->point_step;
        currentPointCloud.row_step = msg->row_step;
        currentPointCloud.data = msg->data;
        currentPointCloud.is_dense = msg->is_dense;

        pcl_conversions::toPCL(currentPointCloud, p2ToPCL);
        pcl::fromPCLPointCloud2(p2ToPCL, currentPCL);

        return currentPCL;
    }
    //For reducing points in cloud
    pcl::PointCloud<pcl::PointXYZ> pointReduction(pcl::PointCloud<pcl::PointXYZ> cloud) {
		pcl::PointCloud<pcl::PointXYZ> currentCloud;
        for(int i = 0; i < cloud.points.size(); i +=500) {
			pcl::PointXYZ Point;
			Point.x = cloud.points[i].x;
			Point.y = cloud.points[i].y;
			Point.z = cloud.points[i].z; 
			
			currentCloud.push_back(Point);
        }
        return currentCloud;
    }

    //For calculating centroids
    void centroidCalc(std::vector<pcl::PointXYZ> corr) {
        float x1 = 0;
        float y1 = 0;
        float z1 = 0;

        float x2 = 0;
        float y2 = 0;
        float z2 = 0;

        //The vector contains the corresponding points, therefore we iterate with 2.
        for(int i = 0; i < corr.size(); i+=2) {
            x1 +=corr[i].x;
            y1 +=corr[i].y;
            z1 +=corr[i].z;

            x2 +=corr[i+1].x;
            y2 +=corr[i+1].y;
            z2 +=corr[i+1].z;
        }
        this->FirstCentroid(0) = x1 / (corr.size()/2);
        this->FirstCentroid(1) = y1 / (corr.size()/2);
        this->FirstCentroid(2) = z1 / (corr.size()/2);

        this->SecondCentroid(0) = x2 / (corr.size()/2);
        this->SecondCentroid(1) = y2 / (corr.size()/2);
        this->SecondCentroid(2) = z2 / (corr.size()/2);
    }

    //For calculating SVD for rotation
    void svd(Eigen::Matrix3f A) //Might need further altercations, since column 1 and 2 seems to be flipped, both in position and if negative or positive
    {
        // U = AAT
        Eigen::Matrix3f W(3,3);
        W = A*A.transpose();
        Eigen::EigenSolver<Eigen::Matrix3f> eigv1(W);
        this->Umatrix = eigv1.pseudoEigenvectors();//std::cout << eigv.pseudoEigenvectors().col(0) << std::endl;//std::cout << eigv.eigenvectors().col(0) << std::endl;

        // V = ATA
        Eigen::Matrix3f Y(3,3);
        Y = A.transpose()*A;
        Eigen::EigenSolver<Eigen::Matrix3f> eigv2(Y);
        this->Vmatrix = eigv2.pseudoEigenvectors();

        // S = sqrt(eigenvalues(AAT)) || S = sqrt(eigenvalues(ATA))
        Eigen::Matrix3f Z(3,3);
        Eigen::Matrix3f sqrtmat(3,3);
        Eigen::EigenSolver<Eigen::Matrix3f> eigv3(Z);
        sqrtmat = eigv3.pseudoEigenvalueMatrix();
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                this->Smatrix(i,j) = sqrt(sqrtmat(i,j));
            }
        }
    }

    //For calculating rotation	
    void RotCalc(std::vector<pcl::PointXYZ> corr) {
        //Finding H
        int numorows = (corr.size());
        //Eigen::Vector3f cloud1;
        //Eigen::Vector3f cloud2;
        Eigen::Matrix3f cloudtest;
        //cloud1.resize(3,numorows);
        //cloud2.resize(3,numorows);
        //for(int i = 0; i < corr.size(); i+=2) {
        //    cloud1(0) = corr[i].x - this->FirstCentroid(0);
        //    cloud1(1) = corr[i].y - this->FirstCentroid(1);
        //    cloud1(2) = corr[i].z - this->FirstCentroid(2);
        //    cloud2(0) = corr[i+1].x - this->SecondCentroid(0);
        //    cloud2(1) = corr[i+1].y - this->SecondCentroid(1);
        //    cloud2(2) = corr[i+1].z - this->SecondCentroid(2);
        //    cloudtest += (cloud1* cloud2.transpose());
        //}
        Eigen::MatrixXf cloud1;
        Eigen::MatrixXf cloud2;
		cloud1.resize(3,numorows);
        cloud2.resize(3,numorows);
        for(int i = 0; i < corr.size(); i+=2) {
            cloud1(0,i) = corr[i].x - this->FirstCentroid(0);
            cloud1(1,i) = corr[i].y - this->FirstCentroid(1);
            cloud1(2,i) = corr[i].z - this->FirstCentroid(2);
            cloud2(0,i) = corr[i+1].x - this->SecondCentroid(0);
            cloud2(1,i) = corr[i+1].y - this->SecondCentroid(1);
            cloud2(2,i) = corr[i+1].z - this->SecondCentroid(2);
        }

        this->Hmatrix = cloud1 * cloud2.transpose();

        //this->Hmatrix = cloudtest;        //Ser ut att fungera också
		//svd(Hmatrix); //Doing SVD
		//this-> RotationMatrix = this->Vmatrix * this->Umatrix.transpose(); //Getting rotation-matrix

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(Hmatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        this->Vmatrix = svd.matrixV();
		this->Umatrix = svd.matrixU();
		this->Smatrix = svd.singularValues();
		//std::cout << svd.singularValues() << std::endl;
		this->RotationMatrix =  this->Umatrix*this->Vmatrix.transpose();
		if(this->RotationMatrix.determinant()<0){
			this->Vmatrix(0,2) = this->Vmatrix(0,2) * -1;
			this->Vmatrix(1,2) = this->Vmatrix(1,2) * -1;
			this->Vmatrix(2,2) = this->Vmatrix(2,2) * -1; 
			this->RotationMatrix = this->RotationMatrix*-1;
		}
		if(this->rotationlock == 0){
			this->TotalRotationMatrix = this->RotationMatrix;
			this->rotationlock =1;
		}
		else
		{
			this->TotalRotationMatrix *= this->RotationMatrix;
		}
    }

    //For calculating translation
    void TransCalc() {
        this->TranslationVec = this->FirstCentroid - (this->RotationMatrix * this->SecondCentroid);
        this->TotalTranslationVec = this->FirstCentroid - (this->TotalRotationMatrix * this->SecondCentroid);;
    }

    //For calculating corresponding points
    void CorrCalc(std::vector<pcl::PointXYZ> &Correspondence) {
        int mindist = 100000000;
        int currdist;
        pcl::PointCloud<pcl::PointXYZ> cloudToDestroy;
		cloudToDestroy = this->reducedCloud2;
		pcl::PointCloud<pcl::PointXYZ> cloudToKeep;
		cloudToKeep = this->reducedCloud1;
        for(int i = 0; i < cloudToKeep.points.size(); i++) {
            pcl::PointXYZ jpoint;
            for(int j = 0; j < cloudToDestroy.points.size(); j++) {
                currdist = std::sqrt(pow(cloudToKeep.points[i].x-cloudToDestroy.points[j].x,2) + pow(cloudToKeep.points[i].y-cloudToDestroy.points[j].y,2) + pow(cloudToKeep.points[i].z-cloudToDestroy.points[j].z,2));
                if(currdist < mindist) {
                    jpoint = cloudToDestroy.points[j];
                    mindist = currdist;
                }
            }
            mindist = 100000000;
            int checker = 0;
            pcl::PointCloud<pcl::PointXYZ> cloudToDestroy2;
            cloudToDestroy2 = cloudToDestroy;
            cloudToDestroy.clear();
            
            //This is used to clear the pointcloud of the point, so that we don't fill the correspondence with all the same points
            for(int check = 0; check < cloudToDestroy2.points.size(); check++)
            {
				if(jpoint.x == cloudToDestroy2.points[check].x && jpoint.y == cloudToDestroy2.points[check].y && jpoint.z == cloudToDestroy2.points[check].z && checker == 0)
				{
					checker++;
				}
				else
				{
					cloudToDestroy.push_back(cloudToDestroy2.points[check]);
				}
			}
			//std::cout << cloudToDestroy.points.size() << std::endl;
            Correspondence.push_back(cloudToKeep[i]);
            Correspondence.push_back(jpoint);
        }
    }

    //For calculating error of adjusted cloud
    float ErrorCalc(std::vector<pcl::PointXYZ> Correspondence, pcl::PointCloud<pcl::PointXYZ> &nextCloud) {
        float error;		
        float e;
		std::string filepath;
		std::ostringstream numstr;
		numstr << this->valueint;
        float firstpos = this->Smatrix(2);
        float secondpos = this->Smatrix(1);
        float thirdpos = this->Smatrix(0); 		
		filepath = "/home/martin/catkin_ws/src/planarSLAM/saved_clouds/corr" + numstr.str() + ".txt";
		std::ofstream write(filepath.c_str());
		if(this->valueint < 12){
		this->valueint++;
		}
        for(int i = 0; i < Correspondence.size(); i+=2) {
			std::ostringstream numx1, numy1,numz1;
			std::ostringstream numx2, numy2,numz2;
			std::string pointstr;
			Eigen::Vector3f corrPointA;
			Eigen::Vector3f corrPointB;
            Eigen::Vector3f currentPoint;
            pcl::PointXYZ currP;
            corrPointA(0) = Correspondence[i].x;
            corrPointA(1) = Correspondence[i].y;
            corrPointA(2) = Correspondence[i].z;
            corrPointB(0) = Correspondence[i+1].x- this->SecondCentroid(0);
            corrPointB(1) = Correspondence[i+1].y- this->SecondCentroid(1);
            corrPointB(2) = Correspondence[i+1].z- this->SecondCentroid(2);
           

            currentPoint = this->RotationMatrix*corrPointB+this->TranslationVec;
            currP.x = currentPoint(0);
            currP.y = currentPoint(1);
            currP.z = currentPoint(2);
            //e = (corrPointA - currentPoint);
            //error = error + e.transpose()*e;
            nextCloud.push_back(currP);
                       
            float transposedA;
            transposedA = abs(corrPointA.transpose()*corrPointA);
            float transposedCurr;
            transposedCurr = abs(currentPoint.transpose()*currentPoint);
            e = (transposedA + transposedCurr);
			error = e + error;
            
            numx1 <<Correspondence[i].x;
            numy1 <<Correspondence[i].y;
            numz1 <<Correspondence[i].z;
            numx2 <<Correspondence[i+1].x;
            numy2 <<Correspondence[i+1].y;
            numz2 <<Correspondence[i+1].z;
            
            pointstr = numx1.str() + "," + numy1.str() + "," + numz1.str() + "\n";
            write << pointstr;
            pointstr = numx2.str() + "," + numy2.str() + "," + numz2.str() + "\n";
            write << pointstr;
            }
        write.close();
        error = error - 2*(firstpos + secondpos + thirdpos);
        //error = error/Correspondence.size();
        return error;
    }

    //This is for applying the total rotation and translation acuired trough the multiple iterations
    void applyRotAndTrans(pcl::PointCloud<pcl::PointXYZ> &UnReducedCloud) {
			//std::cout << "applying rotation and translation" << std::endl;
			//std::cout << this->TotalRotationMatrix << std::endl;
			//std::cout << this->TotalTranslationVec << std::endl;
		this->TransCalc();//Get new translation matrix depending on total rotation
        for(int i = 0; i < UnReducedCloud.points.size(); i++) {
            Eigen::Vector3f currentPoint;
            Eigen::Vector3f TranslatedAndRotated;
            currentPoint(0) = UnReducedCloud.points[i].x;
            currentPoint(1) = UnReducedCloud.points[i].y;
            currentPoint(2) = UnReducedCloud.points[i].z;
            TranslatedAndRotated = this->TotalRotationMatrix*currentPoint+this->TranslationVec;
            //std::cout << TranslatedAndRotated << std::endl;
            UnReducedCloud.points[i].x = TranslatedAndRotated(0);
            UnReducedCloud.points[i].y = TranslatedAndRotated(1);
            UnReducedCloud.points[i].z = TranslatedAndRotated(2);
        }
    }
    void convertTog2oFormat(pcl::PointCloud<pcl::PointXYZ> cloud)
    {
		std::string filepath;
		std::ostringstream numstr;
		std::string row;
		numstr << this->filecounter;
		filepath = "/home/martin/catkin_ws/src/planarSLAM/saved_clouds/g2oFile" + numstr.str() + ".g2o";
		std::ofstream write(filepath.c_str());
		Eigen::Quaternionf quat(this->RotationMatrix);
		Eigen::Vector3f quaternion = quat.vec();
		std::ostringstream numi,numx,numy,numz,rotx,roty,rotz,rotw;
		numi << this->filecounter;
		numx << this->TranslationVec(0);
		numy << this->TranslationVec(1);
		numz << this->TranslationVec(2);
		rotx << quaternion(0);
		roty << quaternion(1);
		rotz << quaternion(2);
		rotw << quat.w();
		std::string pointString;
		pointString = "VERTEX_SE3:QUAT "+ numi.str() + " " + numx.str() + " "+ numy.str() + " " + numz.str() + " " + rotx.str() + " "+ roty.str() + " " + rotz.str() + " " + rotw.str();
		write << pointString;
		write << std::endl;
		//std::cout << quat.vec() << std::endl;
		//	Used for storing points for g2o
		std::cout << this->TotalRotationMatrix << std::endl;
		std::cout << this->TotalTranslationVec << std::endl;
		for(int i = 0; i < cloud.points.size(); i+=100)
		{
			std::ostringstream numi,numx,numy,numz;
			std::string pointString;
			numi << i;
			numx << cloud.points[i].x;
			numy << cloud.points[i].y;
			numz << cloud.points[i].z;
			pointString = "VERTEX_TRACKXYZ "+ numi.str() + " " + numx.str() + " "+ numy.str() + " " + numz.str();
			write << pointString;
			write << pointString;
			write << std::endl;
		}
		
		write.close();
	}
    void mergeCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
		
		this->filecounter++;
		std::string filepath;
		std::ostringstream numstr;
		std::ofstream write("/home/martin/catkin_ws/src/planarSLAM/saved_clouds/coordinates.txt");

		for(int i = 0; i < this->errorvec.size(); i++){
			write << errorvec[i];
			write <<",";
			write << i;
			write << "\n";
		}
		write.close();
		numstr << this->filecounter;
		if(this->firstCloudToFile == true){
			this->mergedMap = cloud;
			filepath = "/home/martin/catkin_ws/src/planarSLAM/saved_clouds/correctedcloud" + numstr.str() + ".pcd";
			convertTog2oFormat(cloud);
			pcl::io::savePCDFileASCII(filepath, cloud);
			this->firstCloudToFile = false;
		}
		else
		{
			filepath = "/home/martin/catkin_ws/src/planarSLAM/saved_clouds/correctedcloud" + numstr.str() + ".pcd";
			pcl::io::savePCDFileASCII(filepath, cloud);
			convertTog2oFormat(cloud);
		}
	}
    //For doing the ICP
    void ICPCalc() {
        sensor_msgs::PointCloud2 CloudToSend;
        this->UnReducedFirstCloud = this->FirstCloud;
        this->UnReducedSecondCloud = this->SecondCloud;
        this->reducedCloud1 = pointReduction(this->FirstCloud);
        this->reducedCloud2 = pointReduction(this->SecondCloud);
        float error;
        bool success = false;
        for(int currIt = 0; currIt < this->iterations; currIt++) {			
            pcl::PointCloud<pcl::PointXYZ> nextCloud;
            //calculate corresponding points
            std::vector<pcl::PointXYZ> Correspondence;
            CorrCalc(Correspondence);
            //calculating both cloud centroids.
            centroidCalc(Correspondence);

            //calculating optimal rotation
            RotCalc(Correspondence);

            //calculating translation
            TransCalc();

            //calculating error
            error = ErrorCalc(Correspondence,nextCloud);
            //std::cout << error << std::endl;
            //this->errorvec.push_back(error); //used for evaluation
            if(error < this->limit && error > -1000) {
				//for(int i = 0; i < nextCloud.points.size(); i++){
				//std::cout << nextCloud.points[i] << std::endl;	
				//}
				this->mergedMap3 += nextCloud;
				std::string filepath = "/home/martin/catkin_ws/src/planarSLAM/saved_clouds/merge3.pcd";
				pcl::io::savePCDFileASCII(filepath, this->mergedMap3);
                applyRotAndTrans(this->UnReducedSecondCloud);
                this->sCloud = false;
				this->FirstCloud = this->UnReducedSecondCloud;
				this->ICPpointcloud.publish(FirstCloud);
				//sensor_msgs::PointCloud2 object_msg;
				//pcl::toROSMsg(nextCloud,object_msg );
				//object_msg.header.frame_id = "ICP";
				//this->ICPpointcloud.publish(object_msg);
				//mergeCloud(this->FirstCloud); //used for comparing clouds
                this->errorvec.clear();
                this->GlobalRotationMatrix =this->GlobalRotationMatrix + this->TotalRotationMatrix;
                this->GlobalTranslationVec =this->GlobalTranslationVec + this->TotalTranslationVec;
                this->TotalRotationMatrix = Eigen::Matrix3f::Zero();
                this->TotalTranslationVec = Eigen::Vector3f::Zero();
                this->rotationlock = 0;
                success = true;
                currIt = this->iterations;
            }
            else{
				this->RotationMatrix = Eigen::Matrix3f::Zero();
                this->TranslationVec = Eigen::Vector3f::Zero();
				this->reducedCloud2 = nextCloud;
			}
            
        }
		if(success == false){
			std::cout << "We failed to find allignment" << std::endl;
			this->RotationMatrix = Eigen::Matrix3f::Zero();
            this->TranslationVec = Eigen::Vector3f::Zero();
			this->rotationlock = 0;
			this->FirstCloud = this->UnReducedSecondCloud;
			
		}
        
        //this->FirstCloud = this->SecondCloud;
    }
};

//////////////////////////////////////////////////////////////
//
// main fn
//
/////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
    // Init the connection with the ROS system
    ros::init(argc, argv, "icpnode");

    ICPSLAM icpHandler;
	ros::Time time = ros::Time::now();

    ros::Rate loop_rate(1000);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

